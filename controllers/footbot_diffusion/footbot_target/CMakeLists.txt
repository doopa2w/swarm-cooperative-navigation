add_library(footbot_target MODULE footbot_static.h  footbot_static.cpp)
target_link_libraries(footbot_target
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
