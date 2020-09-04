add_library(footbot_diffusion MODULE footbot_diffusion.h  footbot_diffusion.cpp trace_message.h trace_message.cpp)
target_link_libraries(footbot_diffusion
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
