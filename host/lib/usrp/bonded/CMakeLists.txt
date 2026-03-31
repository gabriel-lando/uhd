#
# Bonded helper library for multi-device coordination
#

# This file included, use CMake directory variables
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

LIBUHD_APPEND_SOURCES(
    ${CMAKE_CURRENT_SOURCE_DIR}/sync_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/freq_planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/drift_monitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bonded_rx_streamer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/spectrum_stitcher.cpp
)
