#ifndef PR2_DEMO_LIB_TASK_H
#define PR2_DEMO_LIB_TASK_H

#include "pr2_demo_lib/Actions.h"
#include "pr2_demo_lib/Pr2Robot.h"

typedef std::vector<action_t> Task;

void executeAction(Pr2Robot* robot, const action_t& action);

void executeTask(Pr2Robot* robot, const std::vector<action_t>& task);

#endif // PR2_DEMO_LIB_TASK_H