#===============================================================================
# CMake configuration file for the Chrono::Scenario module
#
# Road network (ASAM OpenDRIVE) and driving scenario (ASAM OpenSCENARIO) support,
# backed by esmini (https://github.com/esmini/esmini).
#===============================================================================

option(CH_ENABLE_MODULE_SCENARIO "Enable the Chrono Scenario module" OFF)

# Return now if this module is not enabled
if(NOT CH_ENABLE_MODULE_SCENARIO)
   return()
endif()

message(STATUS "\n==== Chrono SCENARIO module ====\n")

# The module builds on ChTerrain and ChWorldFrame.
if(NOT CH_ENABLE_MODULE_VEHICLE)
  message("Error: Chrono::Scenario requires Chrono::Vehicle. Enable CH_ENABLE_MODULE_VEHICLE.")
  message(STATUS "Chrono::Scenario disabled.")
  set(CH_ENABLE_MODULE_SCENARIO OFF CACHE BOOL "Enable the Chrono Scenario module" FORCE)
  return()
endif()

#-------------------------------------------------------------------------------
# Find esmini
#
# esminiRMLib (OpenDRIVE road manager) is required; esminiLib (OpenSCENARIO
# scenario engine) is optional and enables the ChScenarioPlayer classes.
#-------------------------------------------------------------------------------

set(Esmini_ROOT "" CACHE PATH "Root of an esmini install, release archive, or source+build tree")

find_package(Esmini COMPONENTS ScenarioEngine)

if(NOT Esmini_RM_FOUND)
  message("Error: esminiRMLib not found. Set Esmini_ROOT to an esmini install or source tree.")
  message(STATUS "Chrono::Scenario disabled.")
  set(CH_ENABLE_MODULE_SCENARIO OFF CACHE BOOL "Enable the Chrono Scenario module" FORCE)
  return()
endif()

if(Esmini_SE_FOUND)
  set(CHRONO_SCENARIO_HAS_OPENSCENARIO ON)
  message(STATUS "Chrono::Scenario: OpenDRIVE and OpenSCENARIO support enabled")
else()
  set(CHRONO_SCENARIO_HAS_OPENSCENARIO OFF)
  message(STATUS "Chrono::Scenario: OpenDRIVE support only (esminiLib not found)")
endif()

#-------------------------------------------------------------------------------
# Generate and install the configuration header
#-------------------------------------------------------------------------------

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ChConfigScenario.h.in
               ${PROJECT_BINARY_DIR}/chrono_scenario/ChConfigScenario.h)

install(FILES ${PROJECT_BINARY_DIR}/chrono_scenario/ChConfigScenario.h
        DESTINATION include/chrono_scenario)

#-------------------------------------------------------------------------------
# List files in the Chrono_scenario library
#-------------------------------------------------------------------------------

set(Chrono_SCENARIO_FILES
    ChApiScenario.h
    ChOpenDriveNetwork.h
    ChOpenDriveNetwork.cpp
    ChOpenDriveTerrain.h
    ChOpenDriveTerrain.cpp
)

if(Esmini_SE_FOUND)
  set(Chrono_SCENARIO_FILES ${Chrono_SCENARIO_FILES}
      ChScenarioPlayer.h
      ChScenarioPlayer.cpp
  )
endif()

source_group("" FILES ${Chrono_SCENARIO_FILES})

#-------------------------------------------------------------------------------
# Create the Chrono_scenario library
#-------------------------------------------------------------------------------

add_library(Chrono_scenario ${Chrono_SCENARIO_FILES})
add_library(Chrono::scenario ALIAS Chrono_scenario)

set_target_properties(Chrono_scenario PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_scenario PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_scenario PROPERTIES LINK_FLAGS "/LTCG")
endif()

if(CH_STATIC)
  set_target_properties(Chrono_scenario PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_scenario PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_compile_definitions(Chrono_scenario PRIVATE $<$<COMPILE_LANGUAGE:CXX>:CH_API_COMPILE_SCENARIO>)

# The generated ChConfigScenario.h lives in the build tree.
target_include_directories(Chrono_scenario PUBLIC
                           $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
                           $<INSTALL_INTERFACE:include>)

target_link_libraries(Chrono_scenario PRIVATE Chrono_core)
target_link_libraries(Chrono_scenario PUBLIC Chrono_vehicle)

# esmini exposes a C ABI, so it is an implementation detail of this library except for the
# headers, which the imported targets carry as usage requirements.
target_link_libraries(Chrono_scenario PRIVATE Esmini::RoadManager)

if(Esmini_SE_FOUND)
  target_link_libraries(Chrono_scenario PRIVATE Esmini::ScenarioEngine)
endif()

install(TARGETS Chrono_scenario
        EXPORT ChronoTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include/chrono_scenario)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_scenario
        FILES_MATCHING PATTERN "*.h")

# Copy the esmini shared libraries next to the built binaries so that demos run from the build
# tree without the user having to set PATH / LD_LIBRARY_PATH / DYLD_LIBRARY_PATH.
if(Esmini_RM_FOUND)
  add_custom_command(TARGET Chrono_scenario POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                             $<TARGET_FILE:Esmini::RoadManager>
                             ${CMAKE_BINARY_DIR}/lib
                     COMMENT "Copying esminiRMLib next to the Chrono libraries")
endif()

if(Esmini_SE_FOUND)
  add_custom_command(TARGET Chrono_scenario POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                             $<TARGET_FILE:Esmini::ScenarioEngine>
                             ${CMAKE_BINARY_DIR}/lib
                     COMMENT "Copying esminiLib next to the Chrono libraries")
endif()
