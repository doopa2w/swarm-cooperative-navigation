add_library(explorer MODULE footbot_diffusion.h footbot_diffusion.cpp)
target_link_libraries(explorer
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
