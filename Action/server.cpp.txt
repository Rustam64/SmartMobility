#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/mapping_and_exploration.hpp"

using MappingAndExploration = action_tutorials_interfaces::action::MappingAndExploration;
namespace rclcpp = rclcpp;

class MappingAndExplorationServerNode : public rclcpp::Node
{
public:
    explicit MappingAndExplorationServerNode()
        : Node("mapping_and_exploration_server")
    {
        action_server_ = rclcpp_action::create_server<MappingAndExploration>(
            this, "mapping_and_exploration",
            std::bind(&MappingAndExplorationServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MappingAndExplorationServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&MappingAndExplorationServerNode::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Mapping and Exploration Action Server is ready.");
    }

private:
    rclcpp_action::Server<MappingAndExploration>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MappingAndExploration::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received a new goal request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MappingAndExploration>> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received a goal cancellation request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MappingAndExploration>> goal_handle)
    {
        using namespace std::placeholders;
        RCLCPP_INFO(get_logger(), "Goal has been accepted");
        // Perform the mapping and exploration action here

        // Simulate a successful result
        MappingAndExploration::Result result;
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingAndExplorationServerNode>());
    rclcpp::shutdown();
    return 0;
}
