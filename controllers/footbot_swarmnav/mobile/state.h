#ifndef STATE_H
#define STATE_H

enum EState {
    RANDOM_EXPLORATION = 0,
    MOVE_TO_GOAL = 1,
    RESTING  = 2,
    AVOIDING_COLLISION = 3,
    STATE_SIZE,
    CONTROL_LOOP_START = -1,
    MESSAGE_QUEUE_LENGTH = -2
};

#endif // !STATE_H
