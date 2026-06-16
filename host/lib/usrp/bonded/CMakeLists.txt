#
# Copyright 2026 Gabriel Lando
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# This file included, use CMake directory variables
########################################################################

LIBUHD_APPEND_SOURCES(
    ${CMAKE_CURRENT_SOURCE_DIR}/bonded_usrp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bonded_receiver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/frequency_plan.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/spectrum_stitcher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/overlap_reconstructor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/band_splitter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bonded_transmitter.cpp
)
