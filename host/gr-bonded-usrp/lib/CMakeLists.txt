list(APPEND bonded_usrp_sources
    bonded_source_impl.cc
    overlap_reconstructor_impl.cc
    band_splitter_impl.cc
    bonded_sink_impl.cc)

set(bonded_usrp_sources
    "${bonded_usrp_sources}"
    PARENT_SCOPE)

add_library(gnuradio-bonded_usrp SHARED ${bonded_usrp_sources})

target_link_libraries(gnuradio-bonded_usrp
    gnuradio::gnuradio-runtime
    ${UHD_LIBRARIES})

target_include_directories(gnuradio-bonded_usrp
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
    PRIVATE ${UHD_INCLUDE_DIRS}
    PRIVATE ${PROJECT_SOURCE_DIR}/../lib/usrp/bonded)

target_compile_definitions(gnuradio-bonded_usrp
    PRIVATE gnuradio_bonded_usrp_EXPORTS)

set_target_properties(gnuradio-bonded_usrp PROPERTIES
    OUTPUT_NAME gnuradio-bonded_usrp)

install(TARGETS gnuradio-bonded_usrp
    EXPORT gnuradio-bonded_usrpTargets
    DESTINATION ${GR_LIBRARY_DIR})

install(EXPORT gnuradio-bonded_usrpTargets
    FILE gnuradio-bonded_usrpTargets.cmake
    NAMESPACE gnuradio::
    DESTINATION ${GR_CMAKE_DIR})
