#--------------------------------------------------------------
# List of all Chrono::Scenario demos
#
# These need a visual system to be useful, so they are only built when Chrono::VSG is available.
# The scenario engine (esmini's ScenarioEngine, as opposed to its road manager) is optional, so
# the OpenSCENARIO demos are gated separately.

set(DEMOS
    demo_SCEN_roadnetwork
    demo_SCEN_mcity
)

if(CH_SCENARIO_HAS_SCENARIO_ENGINE)
  set(DEMOS ${DEMOS}
      demo_SCEN_openscenario
      demo_SCEN_intersection
  )
endif()

#--- app bundles get in the way of running these from a build tree
if(APPLE)
    set(CMAKE_MACOSX_BUNDLE OFF)
endif()

#--------------------------------------------------------------
# Libraries

list(APPEND LIBS Chrono_core)
list(APPEND LIBS Chrono_vehicle)
list(APPEND LIBS ChronoModels_vehicle)
list(APPEND LIBS Chrono_scenario)
list(APPEND LIBS Chrono_vsg)
list(APPEND LIBS Chrono_vehicle_vsg)

#--------------------------------------------------------------
# Add executables

foreach(PROGRAM ${DEMOS})
    add_executable(${PROGRAM} ${PROGRAM}.cpp)
    source_group("" FILES ${PROGRAM}.cpp)

    set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    set_property(TARGET ${PROGRAM} PROPERTY CXX_STANDARD 17)
    target_link_libraries(${PROGRAM} ${LIBS})

    if(MSVC)
      set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
    endif()

    install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})
endforeach()

#--------------------------------------------------------------
# The Mcity import pipeline is data tooling, not a build artefact: it fetches and converts a
# third-party scene. Copy it next to the demos so it can be run from a build tree.

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mcity
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
