#ifndef OBJECT_MANAGER__OBJECT_MANAGER_HPP_
#define OBJECT_MANAGER__OBJECT_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/empty.hpp>
#include <object_manager/srv/add_collision_object.hpp>
#include <object_manager/srv/remove_collision_object.hpp>

#include <string>
#include <vector>
#include <unordered_set>
#include <memory>
#include <random>

namespace object_manager
{

class ObjectManagerNode : public rclcpp::Node
{
public:
    ObjectManagerNode();

private:
    // Callback for adding a collision object
    void addCollisionObjectCallback(
        const std::shared_ptr<srv::AddCollisionObject::Request> request,
        std::shared_ptr<srv::AddCollisionObject::Response> response);

    // Callback for removing a collision object
    void removeCollisionObjectCallback(
        const std::shared_ptr<srv::RemoveCollisionObject::Request> request,
        std::shared_ptr<srv::RemoveCollisionObject::Response> response);

    // Callback to remove all collision objects added by this node
    void removeAllObjectsCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    // Utility function to create a CollisionObject
    moveit_msgs::msg::CollisionObject createCollisionObject(
        const std::string &id, 
        const std::string &shape, 
        const std::vector<double> &dimensions, 
        const geometry_msgs::msg::Pose &pose);

    // Utility function to validate shape and dimensions
    bool validateShapeAndDimensions(
        const std::string &shape, 
        const std::vector<double> &dimensions) const;

    // Member variables
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Service<srv::AddCollisionObject>::SharedPtr add_object_service_;
    rclcpp::Service<srv::RemoveCollisionObject>::SharedPtr remove_object_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr remove_all_objects_service_;

    std::unordered_set<std::string> added_object_ids_; // Track added object IDs
    std::mt19937 rng_; // Random number generator

    std::string frame_id_; // Configurable frame ID
};

} // namespace object_manager

#endif // OBJECT_MANAGER__OBJECT_MANAGER_HPP_
