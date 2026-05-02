#ifndef TASK_PlanToEssential_H
#define TASK_PlanToEssential_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskPlanToEssentialConfig : public TaskConfig {
        TaskPlanToEssentialConfig() {
            define("goal_x",  0.,"X coordinate of destination",false, goal_x);
            define("goal_y",  0.,"Y coordinate of destination",false, goal_y);
            define("goal_theta",  0.,"Theta coordinate of destination",false, goal_theta);
            define("dist_threshold",  0.1,"Distance at which a the target is considered reached",false, dist_threshold);
        }

        // convenience aliases, updated by update from the config data
        double goal_x,goal_y,goal_theta;
        double dist_threshold;

    };

    class TaskPlanToEssential : public TaskInstance<TaskPlanToEssentialConfig,SimTasksEnv>
    {
        protected:
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub;
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr trackSub;
            geometry_msgs::msg::Pose2D error;
            rclcpp::Time publish_time;
            void error_cb(geometry_msgs::msg::Pose2D::SharedPtr msg);
        public:
            TaskPlanToEssential(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskPlanToEssential() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();
    };
    class TaskFactoryPlanToEssential : public TaskDefinition<TaskPlanToEssentialConfig, SimTasksEnv, TaskPlanToEssential>
    {

        public:
            TaskFactoryPlanToEssential(TaskEnvironmentPtr env) : 
                Parent("PlanToEssential","Reach a desired destination using the path planner",true,env) {}
            virtual ~TaskFactoryPlanToEssential() {};
    };
};

#endif // TASK_PlanToEssential_H
