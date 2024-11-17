#include "object_manager/object_manager.hpp"

#include <tf2/LinearMath/Quaternion.h> // Added for quaternion operations

namespace object_manager
{

ObjectManagerNode::ObjectManagerNode()
    : Node("object_manager_node"), rng_(std::random_device{}())
{
    // Declare and get the frame_id parameter
    this->declare_parameter<std::string>("frame_id", "world");
    frame_id_ = this->get_parameter("frame_id").as_string();

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // Services
    add_object_service_ = this->create_service<srv::AddCollisionObject>(
        "/add_collision_object",
        std::bind(&ObjectManagerNode::addCollisionObjectCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_object_service_ = this->create_service<srv::RemoveCollisionObject>(
        "/remove_collision_object",
        std::bind(&ObjectManagerNode::removeCollisionObjectCallback, this, std::placeholders::_1, std::placeholders::_2));

    remove_all_objects_service_ = this->create_service<std_srvs::srv::Empty>(
        "/remove_all_objects",
        std::bind(&ObjectManagerNode::removeAllObjectsCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Object Manager Node initialized with frame_id: %s.", frame_id_.c_str());
}

void ObjectManagerNode::addCollisionObjectCallback(
    const std::shared_ptr<srv::AddCollisionObject::Request> request,
    std::shared_ptr<srv::AddCollisionObject::Response> response)
{
    if (!validateShapeAndDimensions(request->shape, request->dimensions))
    {
        response->success = false;
        response->message = "Invalid shape or dimensions.";
        return;
    }

    auto collision_object = createCollisionObject(request->id, request->shape, request->dimensions, request->pose);
    planning_scene_interface_->applyCollisionObject(collision_object);
    added_object_ids_.insert(request->id);

    response->success = true;
    response->message = "Object added successfully.";
}

void ObjectManagerNode::removeCollisionObjectCallback(
    const std::shared_ptr<srv::RemoveCollisionObject::Request> request,
    std::shared_ptr<srv::RemoveCollisionObject::Response> response)
{
    if (added_object_ids_.find(request->id) == added_object_ids_.end())
    {
        response->success = false;
        response->message = "Object not managed by this node.";
        return;
    }

    planning_scene_interface_->removeCollisionObjects({request->id});
    added_object_ids_.erase(request->id);

    response->success = true;
    response->message = "Object removed successfully.";
}

void ObjectManagerNode::removeAllObjectsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    planning_scene_interface_->removeCollisionObjects(std::vector<std::string>(added_object_ids_.begin(), added_object_ids_.end()));
    added_object_ids_.clear();
}

moveit_msgs::msg::CollisionObject ObjectManagerNode::createCollisionObject(
    const std::string &id,
    const std::string &shape,
    const std::vector<double> &dimensions,
    const geometry_msgs::msg::Pose &pose)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id_; // Use the configurable frame_id
    collision_object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    if (shape == "box")
    {
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        for (size_t i = 0; i < 3; ++i)
        {
            primitive.dimensions[i] = dimensions[i];
        }
    }
    else if (shape == "cylinder")
    {
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        for (size_t i = 0; i < 2; ++i)
        {
            primitive.dimensions[i] = dimensions[i];
        }
    }
    else if (shape == "sphere")
    {
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = dimensions[0];
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Unsupported shape type: %s. Defaulting to box.", shape.c_str());
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.1;
        primitive.dimensions[1] = 0.1;
        primitive.dimensions[2] = 0.1;
    }

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

bool ObjectManagerNode::validateShapeAndDimensions(
    const std::string &shape,
    const std::vector<double> &dimensions) const
{
    if (shape == "box" && dimensions.size() == 3)
        return true;
    if (shape == "cylinder" && dimensions.size() == 2)
        return true;
    if (shape == "sphere" && dimensions.size() == 1)
        return true;
    return false;
}

} // namespace object_manager

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<object_manager::ObjectManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
