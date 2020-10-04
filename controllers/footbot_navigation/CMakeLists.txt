add_library(footbot_navigation MODULE footbot_navigation.h footbot_navigation.cpp)
target_link_libraries(footbot_navigation
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
