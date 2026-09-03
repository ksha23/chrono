#--------------------------------------------------------------
# List of all sensor demos

set(DEMOS
  demo_SEN_GPSIMU
)

if(CH_USE_SENSOR_VULKAN_RT)
    set(DEMOS ${DEMOS}
        demo_SEN_JSON
        demo_SEN_camera
        demo_SEN_cornell_box
        demo_SEN_phys_cam
        demo_SEN_lidar
        demo_SEN_radar
        demo_SEN_radar1
        demo_SEN_tachometer
    )
endif()

if(CH_USE_SENSOR_OPTIX)

  set(DEMOS ${DEMOS}
      demo_SEN_buildtest
      demo_SEN_JSON
      demo_SEN_camera
      demo_SEN_cornell_box
      demo_SEN_phys_cam
      demo_SEN_lidar
      demo_SEN_radar
      demo_SEN_radar1
      demo_SEN_tachometer  
  )

  # Demos that combine sensor simulation with vehicle dynamics
  if(CH_ENABLE_MODULE_IRRLICHT AND CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
      set(DEMOS ${DEMOS}
        demo_SEN_HMMWV
        demo_SEN_Gator
        demo_SEN_deformableSoil
      )
  endif()

  if(CH_ENABLE_MODULE_FSI_SPH AND CH_ENABLE_MODULE_VEHICLE)
      set(DEMOS ${DEMOS}
        demo_SEN_CRM_Rendering
      )
  endif()
  
endif()

# --------------------------------------------------------------
# Feature showcase.
#
# One demo per feature, each headless: it orbits a fixed camera around a parked vehicle,
# saves a PNG per frame and returns. They exist so that a change to a rendering backend can
# be checked against a picture rather than only against a number, and so that the module's
# capabilities can be seen without reading code.
#
# They configure the scene only through the public scene API and the standard sensor classes,
# with no backend-specific call anywhere, so they build against whichever ray-tracing backend
# the configuration selected. They drive a vehicle, so they additionally need the vehicle
# module and its models.
if(CH_USE_SENSOR_OPTIX OR CH_USE_SENSOR_VULKAN_RT)
    if(CH_ENABLE_MODULE_VEHICLE AND CH_ENABLE_MODULE_VEHICLE_MODELS)
        set(DEMOS ${DEMOS}
            demo_SEN_showcase_camera_rgb
            demo_SEN_showcase_camera_depth
            demo_SEN_showcase_camera_normal
            demo_SEN_showcase_camera_segmentation
            demo_SEN_showcase_lens_fisheye
            demo_SEN_showcase_lens_radial
            demo_SEN_showcase_physcam_dof
            demo_SEN_showcase_physcam_grain
            demo_SEN_showcase_gi
            demo_SEN_showcase_arealights
            demo_SEN_showcase_envmap
            demo_SEN_showcase_night_headlights
            demo_SEN_showcase_lidar
            demo_SEN_showcase_radar
            demo_SEN_showcase_multisensor
        )

        # The exception to the paragraph above. Scene fog is part of the scene API on
        # ChOptixScene but not on ChVulkanRTScene, which has no SetFogColor or
        # SetFogScattering, so this one demo is genuinely backend-specific until that
        # gap closes rather than being gated out of caution.
        if(CH_USE_SENSOR_OPTIX)
            set(DEMOS ${DEMOS} demo_SEN_showcase_fog)
        endif()
    endif()
endif()

# 1:1 Vulkan-vs-OptiX camera validation. Needs Vulkan RT; with OptiX also enabled it does the
# in-process comparison, with Vulkan alone it captures the Vulkan image for a cross-GPU diff.
if(CH_USE_SENSOR_VULKAN_RT)
    set(DEMOS ${DEMOS}
        demo_SEN_vulkan_validation
    )
endif()

list(REMOVE_DUPLICATES DEMOS)

#--------------------------------------------------------------
# Dependency libraries

list(APPEND LIBRARIES Chrono_core)
list(APPEND LIBRARIES ChronoModels_robot)
list(APPEND LIBRARIES Chrono_sensor)

if(CH_ENABLE_MODULE_VEHICLE) 
   list(APPEND LIBRARIES Chrono_vehicle)
   if(CH_ENABLE_MODULE_VEHICLE_MODELS)
      list(APPEND LIBRARIES ChronoModels_vehicle)
   endif()
endif()

if(CH_ENABLE_MODULE_IRRLICHT)
   list(APPEND LIBRARIES Chrono_irrlicht)
   if(CH_ENABLE_MODULE_VEHICLE) 
      list(APPEND LIBRARIES Chrono_vehicle_irrlicht)
   endif()
endif()

if(CH_ENABLE_MODULE_VSG)
    list(APPEND LIBRARIES Chrono_vsg)
    if(CH_ENABLE_MODULE_VEHICLE) 
       list(APPEND LIBRARIES Chrono_vehicle_vsg)
    endif()
    if(CH_ENABLE_MODULE_FSI_SPH)
       list(APPEND LIBRARIES Chrono_fsisph_vsg)
    endif()
endif()

#--------------------------------------------------------------
# Add executables

message(STATUS "Add demo programs for SENSOR module")

foreach(PROGRAM ${DEMOS})
  # MESSAGE(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES  "${PROGRAM}.cpp")

  # Visual Studio needs to have the debugger working directory set explicitly to the output directory
  # (specifically, so that the data directory is found when running demos in the debugger)
  set_property(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
  target_link_libraries(${PROGRAM} ${LIBRARIES})
  if(MSVC)
    set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
  endif()

  install(TARGETS ${PROGRAM} CONFIGURATIONS Release DESTINATION ${CH_INSTALL_DEMO})

endforeach()
