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

class CollisionSpawner : public rclcpp::Node
{
public:
    CollisionSpawner()
    : Node("collision_spawner"), rng_(std::random_device{}())
    {
        // Create parameter for fixed or random graspable position
        this->declare_parameter<bool>("graspable_is_random", false);
        bool graspable_is_random_ =  this->get_parameter("graspable_is_random").as_bool();

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

        // Spawn new objects after removal
        // Spawn obstacle after 0.5 second
        spawn_obstacle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CollisionSpawner::spawn_obstacle, this));

        // Spawn graspable cylinder after 1 seconds
        spawn_graspable_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(graspable_is_random_ ? &CollisionSpawner::spawn_graspable_random : &CollisionSpawner::spawn_graspable, this));
    }

private:
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

    void spawn_obstacle()
    {
        auto request = std::make_shared<object_manager::srv::AddCollisionObject::Request>();
        request->id = "obstacle_wall_spawner"; // Append '_spawner' postfix
        request->shape = "box";
        request->dimensions = {0.5, 0.02, 0.6}; // x, y, z dimensions

        // Define the pose of the obstacle
        request->pose.position.x = 0.0;
        request->pose.position.y = 0.4;
        request->pose.position.z = 0.3;

        // Generate a random orientation using tf2::Quaternion
        request->pose.orientation = rpyToQuaternion(0.0, 0.0, 0.0);

        // Send the service request asynchronously
        add_object_client_->async_send_request(request,
            std::bind(&CollisionSpawner::spawn_obstacle_response_callback, this, std::placeholders::_1));
    }

    void spawn_graspable()
    {
        auto request = std::make_shared<object_manager::srv::AddCollisionObject::Request>();
        request->id = "graspable_cylinder_spawner"; // Append '_spawner' postfix
        request->shape = "cylinder";
        request->dimensions = {0.1, 0.005}; // height, radius

        // Define the pose of the obstacle
        request->pose.position.x = 0.0;
        request->pose.position.y = 0.2;
        request->pose.position.z = 0.005;

        // Generate a random orientation using tf2::Quaternion
        request->pose.orientation = rpyToQuaternion(0.0, M_PI / 2, 0.0);

        // Send the service request asynchronously
        add_object_client_->async_send_request(request,
            std::bind(&CollisionSpawner::spawn_graspable_response_callback, this, std::placeholders::_1));
    }

    void spawn_graspable_random()
    {
        auto request = std::make_shared<object_manager::srv::AddCollisionObject::Request>();
        request->id = "graspable_cylinder_spawner"; // Append '_spawner' postfix
        request->shape = "cylinder";
        request->dimensions = {0.1, 0.005}; // height, radius

        // Define a random pose within the spawn area
        std::uniform_real_distribution<double> dist_x(0.2, 0.3);
        std::uniform_real_distribution<double> dist_y(0.1, 0.25);
        std::uniform_real_distribution<double> dist_z(0.0, 0.2);
        std::uniform_real_distribution<double> dist_angle(0, 2 * M_PI);

        request->pose.position.x = dist_x(rng_);
        request->pose.position.y = dist_y(rng_);
        request->pose.position.z = dist_z(rng_);

        // Generate a random orientation using tf2::Quaternion
        request->pose.orientation = rpyToQuaternion(dist_angle(rng_), dist_angle(rng_), dist_angle(rng_));

        // Send the service request asynchronously
        add_object_client_->async_send_request(request,
            std::bind(&CollisionSpawner::spawn_graspable_response_callback, this, std::placeholders::_1));
    }

    void spawn_obstacle_response_callback(rclcpp::Client<object_manager::srv::AddCollisionObject>::SharedFuture future)
    {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Obstacle '%s' added successfully.", "obstacle_wall_spawner");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to add obstacle '%s': %s", "obstacle_wall_spawner", response->message.c_str());
            }

            // Cancel the obstacle timer after spawning
            if (spawn_obstacle_timer_) {
                spawn_obstacle_timer_->cancel();
                spawn_obstacle_timer_.reset();
            }
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while adding obstacle: %s", e.what());
        }
    }

    void spawn_graspable_response_callback(rclcpp::Client<object_manager::srv::AddCollisionObject>::SharedFuture future)
    {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Graspable cylinder '%s' added successfully.", "graspable_cylinder_spawner");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to add graspable cylinder '%s': %s", "graspable_cylinder_spawner", response->message.c_str());
            }

            // Cancel the graspable timer after spawning
            if (spawn_graspable_timer_) {
                spawn_graspable_timer_->cancel();
                spawn_graspable_timer_.reset();
            }
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while adding graspable cylinder: %s", e.what());
        }
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
    rclcpp::TimerBase::SharedPtr spawn_obstacle_timer_;
    rclcpp::TimerBase::SharedPtr spawn_graspable_timer_;
    std::mt19937 rng_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CollisionSpawner>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
