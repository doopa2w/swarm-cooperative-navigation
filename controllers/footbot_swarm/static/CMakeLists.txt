add_library(static MODULE footbot_static.h footbot_static.cpp)
target_link_libraries(static
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
