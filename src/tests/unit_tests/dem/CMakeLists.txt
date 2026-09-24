SET(TESTS
    #utest_DEM_mini
    utest_DEM_frictionrolling
    utest_DEM_meshrolling
    utest_DEM_ballistic
    utest_DEM_stack
    utest_DEM_pyramid
)

SET(LIBS
    Chrono_core
    Chrono_dem
)

MESSAGE(STATUS "Add unit test programs for DEM module")
build_utests(YES "${TESTS}" "dem" "${LIBS}")

# With CUDA, also run the mesh test under compute-sanitizer memcheck. The triangle broadphase hands CUB scratch storage
# to its device algorithms, and an undersized buffer shows up only as out-of-bounds device writes.
if(CHRONO_DEM_BACKEND STREQUAL "CUDA")
    find_program(CH_COMPUTE_SANITIZER compute-sanitizer HINTS ${CUDAToolkit_BIN_DIR})
    mark_as_advanced(CH_COMPUTE_SANITIZER)
    if(CH_COMPUTE_SANITIZER)
        add_test(NAME utest_DEM_meshrolling_memcheck
                 COMMAND ${CH_COMPUTE_SANITIZER} --tool memcheck --error-exitcode 1 $<TARGET_FILE:utest_DEM_meshrolling>
                 WORKING_DIRECTORY $<TARGET_FILE_DIR:utest_DEM_meshrolling>)
        set_tests_properties(utest_DEM_meshrolling_memcheck PROPERTIES LABELS "dem")
    endif()
endif()
