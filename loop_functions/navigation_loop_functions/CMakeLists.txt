link_directories(${CMAKE_BINARY_DIR}/controllers/footbot_swarmnav/mobile)
set(navigation_loop_functions_SOURCES navigation_loop_functions.cpp)



add_library(navigation_loop_functions MODULE ${navigation_loop_functions_SOURCES})
target_link_libraries(navigation_loop_functions
  footbot_mobile
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

