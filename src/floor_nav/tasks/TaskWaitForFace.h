#ifndef TASK_WAIT_FOR_FACE_H
#define TASK_WAIT_FOR_FACE_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

#include "region_of_interest_message/msg/faces.hpp"

using namespace task_manager_lib;


namespace floor_nav {
    struct TaskWaitForFaceConfig : public TaskConfig {
        TaskWaitForFaceConfig() {
            define("face_detected",  false,"Whether a face has been detected",true, face_detected);
        }

        // convenience aliases, updated by update from the config data
        bool face_detected;
    };

    class TaskWaitForFace : public TaskInstance<TaskWaitForFaceConfig,SimTasksEnv>
    {
        protected:
            rclcpp::Subscription<region_of_interest_message::msg::Faces>::SharedPtr face_sub;

        public:
            TaskWaitForFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForFace() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
    class TaskFactoryWaitForFace : public TaskDefinition<TaskWaitForFaceConfig, SimTasksEnv, TaskWaitForFace>
    {
        public:
            TaskFactoryWaitForFace(TaskEnvironmentPtr env) : 
                Parent("WaitForFace","Do nothing until we detect a face",true,env) {}
            virtual ~TaskFactoryWaitForFace() {};
    };
}


#endif // TASK_WAIT_FOR_FACE_H