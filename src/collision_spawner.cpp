// src/collision_spawner.cpp

#include <rclcpp/rclcpp.hpp>
#include "object_manager/object_manager.hpp"  // Ensure this include is necessary
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <chrono>
#include <thread>
#include <cmath> // For M_PI
#include <tf2/LinearMath/Quaternion.h> // For quaternion operations

class CollisionSpawner : public rclcpp::Node
{
public:
    CollisionSpawner()
    : Node("collision_spawner"), rng_(std::random_device{}())
    {
        // Create a client for the AddCollisionObject service
        client_ = this->create_client<object_manager::srv::AddCollisionObject>("/add_collision_object");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service /add_collision_object not available, waiting...");
        }

        // Create a timer to spawn obstacle after 2 seconds
        spawn_obstacle_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CollisionSpawner::spawn_obstacle, this));

        // Create a timer to spawn graspable after 4 seconds
        spawn_graspable_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&CollisionSpawner::spawn_graspable, this));
    }

private:
    void spawn_obstacle()
    {
        auto request = std::make_shared<object_manager::srv::AddCollisionObject::Request>();
        request->id = "obstacle_wall";
        request->shape = "box";
        request->dimensions = {0.5, 0.02, 0.2}; // x, y, z dimensions

        // Define the pose of the obstacle
        request->pose.position.x = 0.35;
        request->pose.position.y = -0.1;
        request->pose.position.z = 0.1;
        request->pose.orientation.x = 0.0;
        request->pose.orientation.y = 0.0;
        request->pose.orientation.z = 0.0;
        request->pose.orientation.w = 1.0;

        // Send the service request asynchronously
        client_->async_send_request(request,
            std::bind(&CollisionSpawner::spawn_obstacle_response_callback, this, std::placeholders::_1));
    }

    void spawn_graspable()
    {
        auto request = std::make_shared<object_manager::srv::AddCollisionObject::Request>();
        request->id = "graspable_cylinder";
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
        double roll = dist_angle(rng_);
        double pitch = dist_angle(rng_);
        double yaw = dist_angle(rng_);
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize(); // Ensure the quaternion is normalized

        request->pose.orientation.x = q.x();
        request->pose.orientation.y = q.y();
        request->pose.orientation.z = q.z();
        request->pose.orientation.w = q.w();

        // Send the service request asynchronously
        client_->async_send_request(request,
            std::bind(&CollisionSpawner::spawn_graspable_response_callback, this, std::placeholders::_1));
    }

    void spawn_obstacle_response_callback(rclcpp::Client<object_manager::srv::AddCollisionObject>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Obstacle '%s' added successfully.", "obstacle_wall");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to add obstacle '%s': %s", "obstacle_wall", response->message.c_str());
        }

        // Cancel the obstacle timer after spawning
        spawn_obstacle_timer_->cancel();
    }

    void spawn_graspable_response_callback(rclcpp::Client<object_manager::srv::AddCollisionObject>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Graspable cylinder '%s' added successfully.", "graspable_cylinder");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to add graspable cylinder '%s': %s", "graspable_cylinder", response->message.c_str());
        }

        // Cancel the graspable timer after spawning
        spawn_graspable_timer_->cancel();
    }

    rclcpp::Client<object_manager::srv::AddCollisionObject>::SharedPtr client_;
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
