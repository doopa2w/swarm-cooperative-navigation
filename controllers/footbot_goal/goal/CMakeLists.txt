add_library(goal MODULE footbot_static.h footbot_static.cpp)
target_link_libraries(goal
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
