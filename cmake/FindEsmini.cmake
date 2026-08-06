
#-------------------------------------------------------------------------------
# Find esmini (https://github.com/esmini/esmini)
#
# esmini ships two independent shared libraries:
#   esminiRMLib - OpenDRIVE road manager (road/lane queries). Always required.
#   esminiLib   - OpenSCENARIO scenario engine. Optional; request with
#                 find_package(Esmini COMPONENTS ScenarioEngine)
#
# This script accepts either of the two layouts esmini is normally used from:
#
#   1. Release/install layout (esmini-bin_<platform>.zip, or `cmake --install`)
#        <root>/include/esminiRMLib.hpp
#        <root>/lib/libesminiRMLib.so
#
#   2. In-place source + build tree (git clone; cmake -B build)
#        <root>/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp
#        <root>/build/EnvironmentSimulator/Libraries/esminiRMLib/libesminiRMLib.so
#
# Input variables (all optional):
#   Esmini_ROOT           - root of either layout above
#   Esmini_BUILD_DIR      - build tree, if not <Esmini_ROOT>/build
#   Esmini_INCLUDE_DIR    - explicit override for the header directory
#   Esmini_RM_LIBRARY     - explicit override for the esminiRMLib library file
#   Esmini_SE_LIBRARY     - explicit override for the esminiLib library file
#
# Output:
#   Esmini_FOUND          - all requested components were located
#   Esmini_RM_FOUND       - esminiRMLib was located
#   Esmini_SE_FOUND       - esminiLib was located
#   Esmini::RoadManager   - imported target for esminiRMLib
#   Esmini::ScenarioEngine- imported target for esminiLib
#
# esmini exposes a pure C ABI from both libraries, so no compiler or C++ standard
# matching against the esmini build is required.
#-------------------------------------------------------------------------------

if(NOT Esmini_BUILD_DIR AND Esmini_ROOT)
  set(Esmini_BUILD_DIR "${Esmini_ROOT}/build")
endif()

# Candidate directories, in priority order, for both supported layouts.
set(_esmini_inc_hints
    "${Esmini_INCLUDE_DIR}"
    "${Esmini_ROOT}/include"
    "${Esmini_ROOT}/EnvironmentSimulator/Libraries/esminiRMLib"
    "${Esmini_ROOT}/EnvironmentSimulator/Libraries/esminiLib"
)

set(_esmini_lib_hints
    "${Esmini_ROOT}/lib"
    "${Esmini_ROOT}/bin"
    "${Esmini_BUILD_DIR}/EnvironmentSimulator/Libraries/esminiRMLib"
    "${Esmini_BUILD_DIR}/EnvironmentSimulator/Libraries/esminiLib"
)

#-------------------------------------------------------------------------------
# Headers
#
# In the release layout both headers sit in one include directory; in the build
# tree each library keeps its own. Locate them independently and de-duplicate.
#-------------------------------------------------------------------------------

find_path(Esmini_RM_INCLUDE_DIR
          NAMES esminiRMLib.hpp
          HINTS ${_esmini_inc_hints}
          PATH_SUFFIXES include esmini
)

find_path(Esmini_SE_INCLUDE_DIR
          NAMES esminiLib.hpp
          HINTS ${_esmini_inc_hints}
          PATH_SUFFIXES include esmini
)

#-------------------------------------------------------------------------------
# Libraries
#
# find_library resolves the platform-correct file itself: .so on Linux, .dylib on
# macOS, and the .lib import library on Windows. The matching .dll is located
# separately below, since Windows needs both.
#-------------------------------------------------------------------------------

if(Esmini_RM_LIBRARY)
  set(Esmini_RM_LIB "${Esmini_RM_LIBRARY}" CACHE FILEPATH "esminiRMLib library" FORCE)
else()
  find_library(Esmini_RM_LIB NAMES esminiRMLib HINTS ${_esmini_lib_hints})
endif()

if(Esmini_SE_LIBRARY)
  set(Esmini_SE_LIB "${Esmini_SE_LIBRARY}" CACHE FILEPATH "esminiLib library" FORCE)
else()
  find_library(Esmini_SE_LIB NAMES esminiLib HINTS ${_esmini_lib_hints})
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
  find_file(Esmini_RM_DLL NAMES esminiRMLib.dll HINTS ${_esmini_lib_hints})
  find_file(Esmini_SE_DLL NAMES esminiLib.dll HINTS ${_esmini_lib_hints})
endif()

#-------------------------------------------------------------------------------
# Resolve component status
#-------------------------------------------------------------------------------

set(Esmini_RM_FOUND FALSE)
if(Esmini_RM_INCLUDE_DIR AND Esmini_RM_LIB)
  set(Esmini_RM_FOUND TRUE)
endif()

set(Esmini_SE_FOUND FALSE)
if(Esmini_SE_INCLUDE_DIR AND Esmini_SE_LIB)
  set(Esmini_SE_FOUND TRUE)
endif()

# RoadManager is mandatory; ScenarioEngine only if it was requested.
set(Esmini_FOUND ${Esmini_RM_FOUND})
if(Esmini_FOUND AND "ScenarioEngine" IN_LIST Esmini_FIND_COMPONENTS AND NOT Esmini_SE_FOUND)
  set(Esmini_FOUND FALSE)
endif()

if(NOT Esmini_FIND_QUIETLY)
  if(Esmini_RM_FOUND)
    message(STATUS "Found esminiRMLib: ${Esmini_RM_LIB}")
  else()
    message(STATUS "esminiRMLib not found (set Esmini_ROOT, or Esmini_INCLUDE_DIR + Esmini_RM_LIBRARY)")
  endif()
  if(Esmini_SE_FOUND)
    message(STATUS "Found esminiLib: ${Esmini_SE_LIB}")
  elseif("ScenarioEngine" IN_LIST Esmini_FIND_COMPONENTS)
    message(STATUS "esminiLib not found; OpenSCENARIO support unavailable")
  endif()
endif()

mark_as_advanced(FORCE
                 Esmini_RM_INCLUDE_DIR Esmini_SE_INCLUDE_DIR
                 Esmini_RM_LIB Esmini_SE_LIB
                 Esmini_RM_DLL Esmini_SE_DLL)

#-------------------------------------------------------------------------------
# Imported targets
#
# Both libraries are built shared by default. On Windows an imported SHARED
# target requires IMPORTED_LOCATION to name the .dll and IMPORTED_IMPLIB the
# .lib; elsewhere IMPORTED_LOCATION is the library itself.
#-------------------------------------------------------------------------------

function(_esmini_add_target target_name lib_file dll_file include_dir)
  if(TARGET ${target_name})
    return()
  endif()

  add_library(${target_name} SHARED IMPORTED)
  set_target_properties(${target_name} PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES "${include_dir}")

  if(CMAKE_SYSTEM_NAME MATCHES "Windows" AND dll_file)
    set_target_properties(${target_name} PROPERTIES
                          IMPORTED_LOCATION "${dll_file}"
                          IMPORTED_IMPLIB "${lib_file}")
  else()
    set_target_properties(${target_name} PROPERTIES
                          IMPORTED_LOCATION "${lib_file}")
  endif()
endfunction()

if(Esmini_RM_FOUND)
  _esmini_add_target(Esmini::RoadManager "${Esmini_RM_LIB}" "${Esmini_RM_DLL}" "${Esmini_RM_INCLUDE_DIR}")
endif()

if(Esmini_SE_FOUND)
  _esmini_add_target(Esmini::ScenarioEngine "${Esmini_SE_LIB}" "${Esmini_SE_DLL}" "${Esmini_SE_INCLUDE_DIR}")
endif()
