// src/collision_spawner.cpp

#include <rclcpp/rclcpp.hpp>
#include <object_manager/srv/add_collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <cmath> // For M_PI
#include <tf2/LinearMath/Quaternion.h> // For quaternion operations
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <exception>

class CollisionSpawner : public rclcpp::Node
{
public:
    struct ObjectSpec
    {
        std::string name;
        std::string type;
        std::vector<double> dimensions;
        bool has_pose;
        geometry_msgs::msg::Pose pose;
    };

    CollisionSpawner()
    : Node("collision_spawner"), rng_(std::random_device{}())
    {
        // Declare and get the config_file parameter
        this->declare_parameter<std::string>("config_file", "");
        config_file_ = this->get_parameter("config_file").as_string();

        if (config_file_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'config_file' is empty. Cannot proceed.");
            rclcpp::shutdown();
            return;
        }

        // Load and parse the YAML configuration file
        if (!loadObjectsFromYAML(config_file_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load objects from YAML file: %s", config_file_.c_str());
            rclcpp::shutdown();
            return;
        }

        if (objects_to_spawn_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid objects to spawn.");
            return;
        }

        // Initialize PlanningSceneInterface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Create a client for the AddCollisionObject service
        add_object_client_ = this->create_client<object_manager::srv::AddCollisionObject>("/add_collision_object");

        // Wait for the AddCollisionObject service to be available
        while (!add_object_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the /add_collision_object service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service /add_collision_object not available, waiting...");
        }

        // Remove existing spawner objects
        remove_existing_spawner_objects();

        // Spawn all specified objects with a slight delay between each
        spawn_objects_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CollisionSpawner::spawnNextObject, this));
    }

private:
    bool loadObjectsFromYAML(const std::string &file_path)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(file_path);
            if (!config["objects"])
            {
                RCLCPP_ERROR(this->get_logger(), "YAML file does not contain 'objects' key.");
                return false;
            }

            for (const auto &obj_node : config["objects"])
            {
                ObjectSpec obj_spec;
                // Mandatory fields: name, type, dimensions
                if (!obj_node["name"] || !obj_node["type"] || !obj_node["dimensions"])
                {
                    RCLCPP_WARN(this->get_logger(), "Object specification missing 'name', 'type', or 'dimensions'. Skipping.");
                    continue;
                }

                obj_spec.name = obj_node["name"].as<std::string>();
                obj_spec.type = obj_node["type"].as<std::string>();
                obj_spec.dimensions = obj_node["dimensions"].as<std::vector<double>>();

                // Optional pose
                if (obj_node["pose"])
                {
                    if (obj_node["pose"]["position"] && obj_node["pose"]["orientation"])
                    {
                        obj_spec.pose.position.x = obj_node["pose"]["position"]["x"].as<double>();
                        obj_spec.pose.position.y = obj_node["pose"]["position"]["y"].as<double>();
                        obj_spec.pose.position.z = obj_node["pose"]["position"]["z"].as<double>();

                        double roll = obj_node["pose"]["orientation"]["roll"].as<double>();
                        double pitch = obj_node["pose"]["orientation"]["pitch"].as<double>();
                        double yaw = obj_node["pose"]["orientation"]["yaw"].as<double>();
                        obj_spec.pose.orientation = rpyToQuaternion(roll, pitch, yaw);

                        obj_spec.has_pose = true;
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Object '%s' has incomplete 'pose' specification. Will be placed randomly.", obj_spec.name.c_str());
                        obj_spec.has_pose = false;
                    }
                }
                else
                {
                    obj_spec.has_pose = false;
                }

                objects_to_spawn_.push_back(obj_spec);
            }

            return true;
        }
        catch (const YAML::BadFile &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", e.what());
            return false;
        }
        catch (const YAML::ParserException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML ParserException: %s", e.what());
            return false;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while loading YAML file: %s", e.what());
            return false;
        }
    }

    void remove_existing_spawner_objects()
    {
        // Get all current collision objects
        std::map<std::string, moveit_msgs::msg::CollisionObject> current_objects_map = planning_scene_interface_->getObjects();

        // Collect IDs of objects that end with '_spawner'
        std::vector<std::string> objects_to_remove;
        for (const auto &pair : current_objects_map) {
            const std::string &id = pair.first;
            if (id.size() >= 8 && id.substr(id.size() - 8) == "_spawner") {
                objects_to_remove.push_back(id);
            }
        }

        if (objects_to_remove.empty()) {
            RCLCPP_INFO(this->get_logger(), "No existing spawner objects to remove.");
            return;
        }

        // Remove the filtered objects
        planning_scene_interface_->removeCollisionObjects(objects_to_remove);
        RCLCPP_INFO(this->get_logger(), "Requested removal of %zu existing spawner objects.", objects_to_remove.size());
    }

    void spawnNextObject()
    {
        if (current_spawn_index_ >= objects_to_spawn_.size()) {
            // All objects spawned; cancel the timer
            if (spawn_objects_timer_) {
                spawn_objects_timer_->cancel();
                spawn_objects_timer_.reset();
                RCLCPP_INFO(this->get_logger(), "All objects have been spawned.");
            }
            return;
        }

        const ObjectSpec &obj = objects_to_spawn_[current_spawn_index_];

        auto request = std::make_shared<object_manager::srv::AddCollisionObject::Request>();
        request->id = obj.name + "_spawner"; // Append '_spawner' postfix
        request->shape = obj.type;
        request->dimensions = obj.dimensions;

        if (obj.has_pose) {
            request->pose = obj.pose;
        }
        else {
            // Generate a random pose within predefined bounds
            request->pose = generateRandomPose();
        }

        // Send the service request asynchronously with a lambda callback
        auto callback = [this, obj_name = obj.name](rclcpp::Client<object_manager::srv::AddCollisionObject>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Object '%s_spawner' added successfully.", obj_name.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to add object '%s_spawner': %s", obj_name.c_str(), response->message.c_str());
                }
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception while adding object '%s_spawner': %s", obj_name.c_str(), e.what());
            }
        };

        add_object_client_->async_send_request(request, callback);

        current_spawn_index_++;
    }

    geometry_msgs::msg::Pose generateRandomPose()
    {
        geometry_msgs::msg::Pose pose;

        // Define spawn area boundaries (adjust as needed)
        std::uniform_real_distribution<double> dist_x(-1.0, 1.0);
        std::uniform_real_distribution<double> dist_y(-1.0, 1.0);
        std::uniform_real_distribution<double> dist_z(0.0, 1.0);
        std::uniform_real_distribution<double> dist_angle(0, 2 * M_PI);

        pose.position.x = dist_x(rng_);
        pose.position.y = dist_y(rng_);
        pose.position.z = dist_z(rng_);

        // Random orientation
        pose.orientation = rpyToQuaternion(dist_angle(rng_), dist_angle(rng_), dist_angle(rng_));

        return pose;
    }

    // Helper function to convert RPY to geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize(); // Ensure the quaternion is normalized

        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
        return q_msg;
    }

    rclcpp::Client<object_manager::srv::AddCollisionObject>::SharedPtr add_object_client_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::TimerBase::SharedPtr spawn_objects_timer_;
    std::mt19937 rng_;

    std::vector<ObjectSpec> objects_to_spawn_;
    size_t current_spawn_index_ = 0;

    std::string config_file_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CollisionSpawner>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
