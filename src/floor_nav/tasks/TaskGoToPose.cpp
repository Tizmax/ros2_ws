#include <math.h>
#include "TaskGoToPose.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoToPose::initialise() 
{
    if (cfg->smart_control) {
        RCLCPP_INFO(getNode()->get_logger(),"Going to %.2f %.2f with an angle %.2f (smart)",cfg->goal_x,cfg->goal_y,cfg->goal_theta);
    } else {
        RCLCPP_INFO(getNode()->get_logger(),"Going to %.2f %.2f with an angle %.2f (dumb)",cfg->goal_x,cfg->goal_y,cfg->goal_theta);
    }

    if (cfg->relative) {
        const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    double r = hypot(y_init + cfg->goal_y-tpose.y,x_init + cfg->goal_x-tpose.x);
    double theta_error = remainder(theta_init + cfg->goal_theta - tpose.theta, 2*M_PI);
    
    if (r < cfg->dist_threshold && fabs(theta_error) < cfg->angle_threshold) {
        return TaskStatus::TASK_COMPLETED;
    }
    
    double alpha = remainder(atan2((y_init + cfg->goal_y-tpose.y),x_init + cfg->goal_x-tpose.x)-tpose.theta,2*M_PI);
#ifdef DEBUG_GOTO
    printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f theta_error %.1f\n",
            tpose.x, tpose.y, tpose.theta*180./M_PI,
            cfg->goal_x,cfg->goal_y,r,alpha*180./M_PI,theta_error*180./M_PI);
#endif
    
    if (cfg->smart_control) {
        // Smart control: move towards position while orienting towards goal_theta
        // Combine heading to goal position with orientation constraint
        double alpha_theta = remainder(alpha - theta_error, 2*M_PI);
        
        if (fabs(alpha) > M_PI/9) {
            // if far from goal, prioritize heading towards it
            double rot = cfg->k_alpha * alpha;
            if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
            if (rot < -cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;
            env->publishVelocity(0, rot);
        } else {
            // if close to goal, balance velocity and orientation
            double vel = cfg->k_v * r;
            double rot = cfg->k_alpha * theta_error;
            if (vel > cfg->max_velocity) vel = cfg->max_velocity;
            if (vel < -cfg->max_velocity) vel = -cfg->max_velocity;
            if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
            if (rot < -cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;
#ifdef DEBUG_GOTO
            printf("Smart Cmd v %.2f r %.2f\n",vel,rot);
#endif
            env->publishVelocity(vel, rot);
        }
    } else {
        // Dumb control: reach position first and then rotate to orientation
        if (r < cfg->dist_threshold) {
            // if the position is reached: rotate to orientation
            double rot = ((theta_error>0)?+1:-1)*cfg->max_angular_velocity;
#ifdef DEBUG_GOTO
            printf("Dumb (orienting) Cmd v 0 r %.2f\n",rot);
#endif
            env->publishVelocity(0, rot);
        } else {
            // if the position is not yet reached:go to position using original TaskGoTo logic
            if (fabs(alpha) > M_PI/9) {
                double rot = ((alpha>0)?+1:-1)*cfg->max_angular_velocity;
#ifdef DEBUG_GOTO
                printf("Dumb (turning) Cmd v %.2f r %.2f\n",0.,rot);
#endif
                env->publishVelocity(0,rot);
            } else {
                double vel = cfg->k_v * r;
                double rot = std::max(std::min(cfg->k_alpha*alpha,cfg->max_angular_velocity),-cfg->max_angular_velocity);
                if (vel > cfg->max_velocity) vel = cfg->max_velocity;
                if (vel <-cfg->max_velocity) vel = -cfg->max_velocity;
                if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
                if (rot <-cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;
#ifdef DEBUG_GOTO
                printf("Dumb (moving) Cmd v %.2f r %.2f\n",vel,rot);
#endif
                env->publishVelocity(vel, rot);
            }
        }
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);
