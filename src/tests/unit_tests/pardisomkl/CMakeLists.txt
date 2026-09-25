set(TESTS
    utest_PardisoMKL_threads
)

set(LIBS Chrono_core Chrono_pardisomkl)

message(STATUS "Add unit test programs for PardisoMKL module")
build_utests(YES "${TESTS}" "pardisomkl" "${LIBS}")
