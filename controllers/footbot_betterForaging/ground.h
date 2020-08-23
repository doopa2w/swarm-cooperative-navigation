#ifndef GROUND_H
#define GROUND_H

// The information we pick up from our ground sensors.
enum EGroundSensorInfo {
  OVER_EMPTY_GROUND = 1 << 0,
  OVER_FOOD_ITEM    = 1 << 1,
  OVER_NEST         = 1 << 2
};

#endif
