add_library(footbot_dg MODULE footbot_dg.h footbot_dg.cpp)
target_link_libraries(footbot_dg
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
