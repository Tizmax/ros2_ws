#ifndef TASK_STARE_AT_FACE_H
#define TASK_STARE_AT_FACE_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"
#include "region_of_interest_message/msg/faces.hpp"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskStareAtFaceConfig : public TaskConfig {
        TaskStareAtFaceConfig() {
            define("target",  0.,"Target angle",false, target);
            define("max_angular_velocity",  1.0,"Maximum angular velocity",false,max_angular_velocity);
            define("k_theta",  1.0,"Gain for angular control",false,k_theta);
            define("angle_threshold",  0.01,"Angular error at which the target is considered reached",false, angle_threshold);
            define("relative",  false,"Is the target pose relative or absolute",true, relative);
            define("face_detected",  false,"Whether a face has been detected",true, face_detected);
            define("center_face",  true,"Center detected face in image instead of reaching heading angle",true, center_face);
            define("face_center_threshold",  0.05,"Normalized face offset threshold for centering (0-1)",false, face_center_threshold);
            define("image_width",  640.,"Width of camera image in pixels",false, image_width);
        }

        // convenience aliases, updated by update from the config data
        double target;
        double k_theta;
        double max_angular_velocity;
        double angle_threshold;
        bool relative;
        bool face_detected;
        bool center_face;
        double face_center_threshold;
        double image_width;
    };

    class TaskStareAtFace : public TaskInstance<TaskStareAtFaceConfig,SimTasksEnv>
    {
        protected: 
            double initial_heading;
            rclcpp::Subscription<region_of_interest_message::msg::Faces>::SharedPtr face_sub;
            double face_offset;  // Normalized offset of face center from image center (-1 to 1)
            bool face_in_view;

        public:
            TaskStareAtFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env), face_offset(0.0), face_in_view(false) {}
            virtual ~TaskStareAtFace() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryStareAtFace : public TaskDefinition<TaskStareAtFaceConfig, SimTasksEnv, TaskStareAtFace>
    {

        public:
            TaskFactoryStareAtFace(TaskEnvironmentPtr env) : 
                Parent("StareAtFace","Stare at a desired face",true,env) {}
            virtual ~TaskFactoryStareAtFace() {};
    };
};

#endif // TASK_STARE_AT_FACE_H
