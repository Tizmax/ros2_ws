#ifndef TASK_GOTO_POSE_H
#define TASK_GOTO_POSE_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskGoToPoseConfig : public TaskConfig {
        TaskGoToPoseConfig() {
            define("goal_x",  0.,"X coordinate of destination",false, goal_x);
            define("goal_y",  0.,"Y coordinate of destination",false, goal_y);
            define("goal_theta",  0.,"Orientation at destination",false, goal_theta);
            define("k_v",  1.0,"Gain for velocity control",false, k_v);
            define("k_alpha",  1.0,"Gain for angular control",false, k_alpha);
            define("max_velocity",  1.0,"Max allowed velocity",false, max_velocity);
            define("max_angular_velocity",  1.0,"Max allowed angular velocity",false, max_angular_velocity);
            define("dist_threshold",  0.1,"Distance at which a the target is considered reached",false, dist_threshold);
            define("angle_threshold",  M_PI/9,"Angle error at which the target orientation is considered reached",false, angle_threshold);
            define("relative",  false,"Is the target pose relative or absolute",true, relative);
            define("smart_control",  true,"Use smart control to avoid oscillations",true, smart_control);
        }

        // convenience aliases, updated by update from the config data
        double goal_x,goal_y, goal_theta;
        double k_v,k_alpha;
        double max_velocity;
        double max_angular_velocity;
        double dist_threshold;
        double angle_threshold;
        bool relative;
        bool smart_control;

    };


    class TaskGoToPose : public TaskInstance<TaskGoToPoseConfig,SimTasksEnv>
    {
        protected:
            double x_init,y_init, theta_init;
        public:
            TaskGoToPose(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskGoToPose() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryGoToPose : public TaskDefinition<TaskGoToPoseConfig, SimTasksEnv, TaskGoToPose>
    {

        public:
            TaskFactoryGoToPose(TaskEnvironmentPtr env) : 
                Parent("GoToPose","Reach a desired destination with an orientation constraint",true,env) {}
            virtual ~TaskFactoryGoToPose() {};
    };
};

#endif // TASK_GOTO_POSE_H
