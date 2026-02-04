#include <math.h>
#include "TaskWaitForFace.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::initialise() 
{
    RCLCPP_INFO(node->get_logger(),"Waiting for a face to be detected ...");

    cfg->face_detected = false;

    // Subscribing to face detection topic
    face_sub = node->create_subscription<region_of_interest_message::msg::Faces>(
        "/detected_faces_roi",
        10,
        [this](const region_of_interest_message::msg::Faces::SharedPtr msg) {
            if (!msg->rois.empty()) {
                cfg->face_detected = true;
                RCLCPP_INFO(node->get_logger(),"Face detected !!");
            }
        }
    );
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskWaitForFace::iterate()
{
    if (cfg->face_detected) {
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskWaitForFace::terminate()
{
    // Unsubscribe from face detection topic
// face_sub.reset();
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryWaitForFace)