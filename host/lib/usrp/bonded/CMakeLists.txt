#
# Copyright 2026 Gabriel
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Bonded PPS-only multi-USRP sources
########################################################################
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3F fftw3f REQUIRED)
pkg_check_modules(VOLK volk REQUIRED)

LIBUHD_APPEND_SOURCES(
    ${CMAKE_CURRENT_SOURCE_DIR}/multi_usrp_bonding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bonded_receiver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/pps_drift_estimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/phase_corrector.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/spectrum_stitcher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/bonded_rx_streamer.cpp
)

LIBUHD_APPEND_LIBS(${FFTW3F_LIBRARIES} ${VOLK_LIBRARIES})

include_directories(
    ${FFTW3F_INCLUDE_DIRS}
    ${VOLK_INCLUDE_DIRS}
)
