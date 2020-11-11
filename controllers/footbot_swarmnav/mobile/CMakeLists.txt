add_library(footbot_mobile SHARED footbot_mobile.h footbot_mobile.cpp trace_message.h trace_message.cpp state.h)
target_link_libraries(footbot_mobile
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

