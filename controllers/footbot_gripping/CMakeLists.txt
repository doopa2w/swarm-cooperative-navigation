add_library(footbot_gripping MODULE footbot_gripping.h footbot_gripping.cpp)
target_link_libraries(footbot_gripping
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
