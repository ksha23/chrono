#=============================================================================
# CMake configuration file for the Chrono PardisoMKL module
# 
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================

option(CH_ENABLE_MODULE_PARDISO_MKL "Enable the Chrono PardisoMKL module" OFF)

if(NOT CH_ENABLE_MODULE_PARDISO_MKL)
    return()
endif()

message(STATUS "\n==== Chrono PardisoMKL module ====\n")

# ------------------------------------------------------------------------------
# Dependencies for MKL module
# ------------------------------------------------------------------------------

message(STATUS "Find MKL libraries")
#-------------
# Main options
#-------------
# MKL_ROOT: oneMKL root directory (May be required for non-standard install locations. Optional otherwise.)
#    Default: use location from MKLROOT environment variable or <Full path to this file>/../../../ if MKLROOT is not defined
# MKL_ARCH
#    Values:  ia32 [ia32 support is deprecated]
#             intel64
#    Default: intel64
# MKL_LINK
#    Values:  static, dynamic, sdl
#    Default: dynamic
#       Exceptions:- SYCL doesn't support sdl
# MKL_THREADING
#    Values:  sequential,
#             intel_thread (Intel OpenMP),
#             gnu_thread (GNU OpenMP),
#             pgi_thread (PGI OpenMP) [PGI support is deprecated],
#             tbb_thread
#    Default: gnu_thread if Chrono uses the GNU OpenMP runtime (libgomp), intel_thread otherwise
#       (see the note on MKL_THREADING below)
# MKL_INTERFACE (for MKL_ARCH=intel64 only)
#    Values:  lp64, ilp64
#       GNU or INTEL interface will be selected based on Compiler.
#    Default: ilp64
# MKL_MPI
#    Values:  intelmpi, mpich, openmpi, msmpi, mshpc
#    Default: intelmpi
#-----------------------------------
# Special options (OFF by default)
#-----------------------------------
# ENABLE_BLAS95:      Enables BLAS Fortran95 API in MKL::MKL
# ENABLE_LAPACK95:    Enables LAPACK Fortran95 API in MKL::MKL
# ENABLE_BLACS:       Enables cluster BLAS library in MKL::MKL
# ENABLE_CDFT:        Enables cluster DFT library in MKL::MKL
# ENABLE_SCALAPACK:   Enables cluster LAPACK library in MKL::MKL
# ENABLE_OMP_OFFLOAD: Enables OpenMP Offload functionality in MKL::MKL

set(MKL_ARCH "intel64" CACHE STRING "MKL architecture")
set_property(CACHE MKL_ARCH PROPERTY STRINGS intel64 ia32)

set(MKL_LINK "dynamic" CACHE STRING "MKL linking mode")
set_property(CACHE MKL_LINK PROPERTY STRINGS static dynamic sdl)

# The MKL threading layer must use the same OpenMP runtime as the rest of Chrono. intel_thread requires the Intel
# (or LLVM) OpenMP runtime (libiomp5/libomp), while a GCC build of Chrono uses the GNU runtime (libgomp). If both
# runtimes end up in the same process, the omp_* symbols are resolved from whichever one is loaded first; when
# that is libgomp, multithreaded Pardiso fails (e.g. ANALYZE returns InvalidInput) or crashes. Therefore, default
# to gnu_thread when Chrono's OpenMP support links libgomp, and to intel_thread otherwise.
set(CH_MKL_THREADING_DEFAULT "intel_thread")
if(CH_ENABLE_OPENMP AND "gomp" IN_LIST OpenMP_CXX_LIB_NAMES AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CH_MKL_THREADING_DEFAULT "gnu_thread")
endif()

set(MKL_THREADING "${CH_MKL_THREADING_DEFAULT}" CACHE STRING "MKL threading lib")
set_property(CACHE MKL_THREADING PROPERTY STRINGS intel_thread sequential gnu_thread pgi_thread tbb_thread)

set(MKL_INTERFACE "lp64" CACHE STRING "MKL interface")
set_property(CACHE MKL_INTERFACE PROPERTY STRINGS ilp64 lp64)

set(MKL_MPI "intelmpi" CACHE STRING "MKL MPI library")
set_property(CACHE MKL_MPI PROPERTY STRINGS intelmpi mpich openmpi msmpi mshpc)


mark_as_advanced(FORCE MKL_ARCH)
mark_as_advanced(FORCE MKL_LINK)
mark_as_advanced(FORCE MKL_THREADING)
mark_as_advanced(FORCE MKL_INTERFACE)
mark_as_advanced(FORCE MKL_MPI)

message(STATUS "MKL threading layer: ${MKL_THREADING} (default for this configuration: ${CH_MKL_THREADING_DEFAULT})")
if(MKL_THREADING STREQUAL "intel_thread" AND CH_MKL_THREADING_DEFAULT STREQUAL "gnu_thread")
  message(WARNING "MKL_THREADING=intel_thread loads the Intel/LLVM OpenMP runtime next to the GNU OpenMP runtime "
                  "used by Chrono; multithreaded Pardiso may fail or crash. Set MKL_THREADING=gnu_thread "
                  "(or sequential) unless you know both runtimes can coexist.")
endif()

if(NOT TARGET MKL::MKL)
  find_package(MKL CONFIG REQUIRED)
else()
  message(STATUS "MKL::MKL target already exists. Skip search for MKL")
endif()

if(NOT MKL_FOUND)
  set(MKL_ROOT "C:/Program Files (x86)/Intel/oneAPI/mkl/latest" CACHE PATH "Location of MKL installation (e.g. \"C:/Program Files (x86)/Intel/oneAPI/mkl/latest\"")
  mark_as_advanced(CLEAR MKL_ROOT)
else()
  set(MKL_ROOT "${MKL_DIR}/../../../")
  set(MKL_ROOT "${MKL_ROOT}" PARENT_SCOPE)
  mark_as_advanced(FORCE MKL_ROOT)
endif()


# ------------------------------------------------------------------------------
# List all files in the Chrono mkl module
# ------------------------------------------------------------------------------

set(Chrono_PardisoMKL_HEADERS
  ChApiPardisoMKL.h
  ChSolverPardisoMKL.h
)

set(Chrono_PardisoMKL_SOURCES
  ChSolverPardisoMKL.cpp
)

source_group("" FILES ${Chrono_PardisoMKL_HEADERS} ${Chrono_PardisoMKL_SOURCES})

# ------------------------------------------------------------------------------
# Add the Chrono_pardisomkl library
# ------------------------------------------------------------------------------

add_library(Chrono_pardisomkl
            ${Chrono_PardisoMKL_SOURCES}
            ${Chrono_PardisoMKL_HEADERS})
add_library(Chrono::pardisomkl ALIAS Chrono_pardisomkl)

set_target_properties(Chrono_pardisomkl PROPERTIES DEBUG_POSTFIX ${CH_DEBUG_POSTFIX})

if(CH_WHOLE_PROG_OPT)
  set_target_properties(Chrono_pardisomkl PROPERTIES COMPILE_FLAGS "/GL")
  set_target_properties(Chrono_pardisomkl PROPERTIES LINK_FLAGS "/LTCG")
endif()

if (CH_STATIC)
  set_target_properties(Chrono_pardisomkl PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

if(MSVC)
  set_target_properties(Chrono_pardisomkl PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()

target_link_libraries(Chrono_pardisomkl
                      Chrono_core
                      MKL::MKL
                      )

target_compile_definitions(Chrono_pardisomkl PRIVATE "CH_API_COMPILE_PARDISOMKL")
target_compile_definitions(Chrono_pardisomkl PRIVATE "EIGEN_USE_MKL_ALL")
target_compile_definitions(Chrono_pardisomkl PRIVATE "MKL_LP64") # who needs this? Eigen?


install(TARGETS Chrono_pardisomkl
  EXPORT ChronoTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include/chrono_pardisomkl)

#-------------------------------------------------------------------------------
# Install files
#-------------------------------------------------------------------------------

# Old way
install(FILES ${Chrono_PardisoMKL_HEADERS}
        DESTINATION include/chrono_pardisomkl)