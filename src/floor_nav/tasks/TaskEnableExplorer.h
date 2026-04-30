#ifndef TASK_ENABLE_EXPLORER_H
#define TASK_ENABLE_EXPLORER_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskEnableExplorerConfig : public TaskConfig {
        TaskEnableExplorerConfig() {
            define("enable the explorer",  false,"Whether we are exploring",true, exploring);
        }

        // convenience aliases, updated by update from the config data
        bool exploring;
    };

    class TaskEnableExplorer : public TaskInstance<TaskEnableExplorerConfig,SimTasksEnv>
    {
        protected:
            rclcpp::Subscription<region_of_interest_message::msg::Faces>::SharedPtr face_sub;

        public:
            TaskEnableExplorer(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskEnableExplorer() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();

    };
    class TaskFactoryEnableExplorer : public TaskDefinition<TaskEnableExplorerConfig, SimTasksEnv, TaskEnableExplorer>
    {
        public:
            TaskFactoryEnableExplorer(TaskEnvironmentPtr env) : 
                Parent("EnableExplorer","Enable the explorer",true,env) {}
            virtual ~TaskFactoryEnableExplorer() {};
    };
}


#endif // TASK_ENABLE_EXPLORER_H