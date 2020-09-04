#ifndef STATE_H
#define STATE_H

enum EState {
  EXPLORING               = 0,
  PICKING_UP_ITEM         = 1,
  RETURNING_TO_NEST       = 2,
  DROPPING_ITEM           = 3,
  SEARCHING_RESTING_PLACE = 4,
  RESTING                 = 5,
  AVOIDING_COLLISION      = 6,
  // The size for an array indexed by (non-debugging) message types.
  STATE_SIZE,
  // Some debugging messages
  CONTROL_LOOP_START = -1,
  MESSAGE_QUEUE_LENGTH = -2
};

#endif
