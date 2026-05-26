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
)
