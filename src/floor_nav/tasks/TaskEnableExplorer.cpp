#include <math.h>
#include "TaskEnableExplorer.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskEnableExplorer::initialise() 
{
    RCLCPP_INFO(node->get_logger(),"Enabling explorer ...");

    cfg->exploring = false;

    // Trigger the explorer service to enable exploration
    enable_explorer_service(true);

    cfg->exploring = true;
    
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskEnableExplorer::iterate()
{
    // check if we are already exploring.if not, write a log message
    if (!cfg->exploring) {
        RCLCPP_INFO(node->get_logger(),"The explorer should be enabled, but it is not yet. Waiting for it to be enabled...");
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskEnableExplorer::terminate()
{
    // Trigger the explorer service to disable exploration
    enable_explorer_service(false);

    cfg->exploring = false;

    return TaskStatus::TASK_TERMINATED;
}


void enable_explorer_service(bool enable) {

    // create a client for the enable explorer service
    auto client = node->create_client<std_srvs::srv::SetBool>("~/enable_explorer");
    
    // create a request with the desired state (enable or disable)
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;

    // wait for the service to be available by checking every second
    while (!client->wait_for_service(1s)) {
        RCLCPP_WARN(node->get_logger(), "Waiting for the enable explorer service to be available...");
    }

    //send an asynchronous request to the service and store the future result
    auto result_future = client->async_send_request(request);

    // service response handling
    try {
        auto result = result_future.get();
        if (result->success) {
            RCLCPP_INFO(node->get_logger(), "Enable explorer service response: %s", result->message.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Failed to enable explorer: %s", result->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Service call failed (enable explorer): %s", e.what());
    }
}



DYNAMIC_TASK(TaskEnableExplorer)