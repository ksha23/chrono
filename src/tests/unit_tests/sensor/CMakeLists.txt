# Sensor module unit tests.
#
# The tests below are split by what they actually depend on, NOT by which backend happens
# to be the historical default. Anything that only touches the public sensor API
# (ChSensorManager, ChSensor, the filter graph, the buffer types) runs on whichever
# ray-tracing backend the build selected -- OptiX, Vulkan RT or Metal RT.

# --- backend-agnostic: public sensor API only, no OptiX headers, no OptiX-only buffer
# --- formats. These build and run on any backend.
set(TESTS
    utest_SEN_gps
    utest_SEN_data_access      # background colour reaches the user buffer intact
    utest_SEN_interface        # sensor bookkeeping + dynamic scene rebuild (box/sphere/cyl/mesh)
    utest_SEN_threadsafety     # buffer ownership handover out of the filter graph
    utest_SEN_radar            # radar clustering utilities
)

# --- Metal RT specific ------------------------------------------------------------
if(CH_USE_SENSOR_METAL_RT)
  set(TESTS ${TESTS}
    utest_SEN_metal_stochastic  # tier 2: convergence / variance of the stochastic features
  )
endif()

# --- OptiX specific ---------------------------------------------------------------
# utest_SEN_optix{engine,geometry,pipeline} drive OptiX pipeline internals directly.
# utest_SEN_camera_convergence asks for a ChFilterRGBA16Access buffer and exercises
# Integrator::PATH plus ChPhysCameraSensor supersampling; the RGBA16 camera format and
# the PATH integrator are OptiX-only today, so the Metal build gets its own equivalent
# (utest_SEN_metal_stochastic) rather than a weakened copy of this one.
if(CH_USE_SENSOR_OPTIX)
  set(TESTS ${TESTS}
    utest_SEN_camera_convergence
    utest_SEN_optixengine
    utest_SEN_optixgeometry
    utest_SEN_optixpipeline
  )
endif()

set(LIBS Chrono_core Chrono_sensor)

message(STATUS "Add unit test programs for SENSOR module")
build_utests(YES "${TESTS}" "sensor" "${LIBS}")

# --- standalone verification programs -------------------------------------------------
# These print measurements and return non-zero on failure, so they have their own main()
# and cannot go through build_utests (which links gtest_main). They are registered with
# CTest directly, exactly as build_utests does, and run under the same label so that
#     ctest -L sensor
# is the single entry point for the whole module -- no bespoke runner script.
#
# WORKING_DIRECTORY matters: Chrono resolves its data directory relative to the working
# directory, so a test launched from anywhere else silently loses every texture and HDR map.
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(VERIFY_WORKING_DIR "${EXECUTABLE_OUTPUT_PATH}/Release")
else()
    set(VERIFY_WORKING_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

set(VERIFY_PROGS verify_render_math)
if(CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
  set(VERIFY_PROGS ${VERIFY_PROGS} verify_dynamic_sensors)
endif()

foreach(PROGRAM ${VERIFY_PROGS})
  add_executable(${PROGRAM} ${PROGRAM}.cpp)
  set_target_properties(${PROGRAM} PROPERTIES FOLDER tests)
  target_include_directories(${PROGRAM} PRIVATE "${CMAKE_SOURCE_DIR}/src/chrono_thirdparty")
  target_link_libraries(${PROGRAM} Chrono_core Chrono_sensor)
  if(CH_ENABLE_MODULE_VEHICLE)
    target_link_libraries(${PROGRAM} Chrono_vehicle)
    if(CH_ENABLE_MODULE_VEHICLE_MODELS)
      target_link_libraries(${PROGRAM} ChronoModels_vehicle)
    endif()
  endif()
  install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
  add_test(${PROGRAM} ${PROJECT_BINARY_DIR}/bin/${PROGRAM})
  set_tests_properties(${PROGRAM} PROPERTIES WORKING_DIRECTORY ${VERIFY_WORKING_DIR} LABELS "sensor")
endforeach()
