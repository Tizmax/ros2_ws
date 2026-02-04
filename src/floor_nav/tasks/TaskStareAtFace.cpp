#include <math.h>
#include "TaskStareAtFace.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskStareAtFace::initialise() 
{
    RCLCPP_INFO(getNode()->get_logger(),"TaskStareAtFace: center_face=%d", cfg->center_face);
    
    face_offset = 0.0;
    face_in_view = false;
    
    if (cfg->center_face) {
        // Subscribe to face detection topic
        face_sub = getNode()->create_subscription<region_of_interest_message::msg::Faces>(
            "/detected_faces_roi", 10,
            [this](const region_of_interest_message::msg::Faces::SharedPtr msg) {
                if (!msg->rois.empty()) {
                    const auto& roi = msg->rois[0];
                    double face_center_x = roi.x_offset + roi.width / 2.0;
                    double image_center_x = cfg->image_width / 2.0;
                    face_offset = (face_center_x - image_center_x) / image_center_x;
                    face_in_view = true;
                } else {
                    face_in_view = false;
                }
            }
        );
    } else {
        // Heading mode like TaskSetHeading
        if (cfg->relative) {
            const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
            initial_heading = tpose.theta;
        } else {
            initial_heading = 0.0;
        }
    }
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
    if (cfg->center_face) {
        if (!face_in_view) {
            return TaskStatus::TASK_RUNNING;
        }
        
        if (fabs(face_offset) < cfg->face_center_threshold) {
            return TaskStatus::TASK_COMPLETED;
        }
        
        double rot = -cfg->k_theta * face_offset;  // Negate to correct direction
        if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
        if (rot < -cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;
        
        env->publishVelocity(0.0, rot);
        return TaskStatus::TASK_RUNNING;
    } else {
        // Same as TaskSetHeading
        const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
        double alpha = remainder(initial_heading+cfg->target-tpose.theta,2*M_PI);
        if (fabs(alpha) < cfg->angle_threshold) {
            return TaskStatus::TASK_COMPLETED;
        }
        double rot = cfg->k_theta*alpha;
        if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
        if (rot <-cfg->max_angular_velocity) rot =-cfg->max_angular_velocity;
        env->publishVelocity(0.0, rot);
        return TaskStatus::TASK_RUNNING;
    }
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
