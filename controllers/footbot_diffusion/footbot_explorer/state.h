/*
 * AUTHOR: Darryl Tan <dtzl.added@gmail.com>
 * Enumerates of all possible states for a mobile robot.
 * 
 * RANDOM_EXPLORATION -> Moving randomly by changing direction after a fixed period with fixed wheel speed.
 * AGGRESSIVE_EXPLORATION -> RANDOM_EXPLORATION but with longer cooldown before changing direction.
 * MOVE_TO_GOAl -> Once found the designated goal, move towards the goal.
 * RESTING -> ONce reached the goal's location within certain distance, maintained a fixed distance with
 *              the target robot and remained static.
 * AVOIDING_COLLISION -> Once detected obstacle within proximity, steer or maintain direction accordinly.
 * 
 */

#ifndef STATE_H
#define STATE_H

enum EState {
    RANDOM_EXPLORATION = 0,
    AGGRESSIVE_EXPLORATION = 1,
    FOUND_GOAL = 2,
    MOVE_TO_GOAL = 3,
    RESTING = 4,
    AVOIDING_COLLISION = 5,
    // The size for an array indexed by message types
    STATE_SIZE,
    CONTROL_LOOP_START = -1,
    MESSAGE_QUEUE_LENGTH = -2
};


#endif // !STATE_H

