add_library(footbot_manualcontrol SHARED footbot_manualcontrol.h footbot_manualcontrol.cpp)
target_link_libraries(footbot_manualcontrol
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
