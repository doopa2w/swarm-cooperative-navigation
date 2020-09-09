add_library(mobile MODULE footbot_diffusion.h footbot_diffusion.cpp)
target_link_libraries(mobile
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
