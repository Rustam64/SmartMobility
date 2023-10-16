#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_tutorials_interfaces/action/mapping_and_exploration.hpp"

using MappingAndExploration = action_tutorials_interfaces::action::MappingAndExploration;
namespace rclcpp = rclcpp;

class MappingAndExplorationClientNode : public rclcpp::Node
{
public:
    explicit MappingAndExplorationClientNode()
        : Node("mapping_and_exploration_client")
    {
        action_client_ = rclcpp_action::create_client<MappingAndExploration>(
            this, "mapping_and_exploration");

        // Wait for the action server to become available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(get_logger(), "Action server not available. Exiting.");
            return;
        }

        send_goal();
    }

private:
    rclcpp_action::Client<MappingAndExploration>::SharedPtr action_client_;

    void send_goal()
    {
        auto goal = MappingAndExploration::Goal();

        // Populate the goal message with the desired data
        // For example, set the map and other parameters in the goal

        RCLCPP_INFO(get_logger(), "Sending goal request");
        auto send_goal_options = rclcpp_action::Client<MappingAndExploration>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MappingAndExplorationClientNode::goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&MappingAndExplorationClientNode::result_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MappingAndExplorationClientNode::feedback_callback, this, _1, _2);

        action_client_->async_send_goal(goal, send_goal_options);
    }

    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<MappingAndExploration>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server.");
            return;
        }
        RCLCPP_INFO(get_logger(), "Goal accepted by the action server.");
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<MappingAndExploration>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Action succeeded");
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Action failed with result code %d", static_cast<int>(result.code));
            break;
        }
    }

    void feedback_callback(
        const rclcpp_action::ClientGoalHandle<MappingAndExploration>::SharedPtr,
        const std::shared_ptr<const MappingAndExploration::Feedback> feedback)
    {
        // Handle feedback from the action server
        // Feedback typically contains information about the action's progress
        // You can process and display this feedback as needed
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingAndExplorationClientNode>());
    rclcpp::shutdown();
    return 0;
}
