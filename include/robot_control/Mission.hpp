#ifndef MISSION_HPP
#define MISSION_HPP

#include <vector>
#include "checkers_msgs/msg/piece.hpp" 

enum class Task {
    NONE = 0,
    ATTACH = 1,
    DETACH = 2,
};

class Mission {
public:
    int row, col;
    Task task;

    // Constructor
    Mission(int row, int col, Task task);
};

#endif // MISSION_HPP
