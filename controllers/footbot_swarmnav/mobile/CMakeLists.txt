add_library(mobile MODULE footbot_mobile.h footbot_mobile.cpp trace_message.h trace_message.cpp state.h)
target_link_libraries(mobile
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

