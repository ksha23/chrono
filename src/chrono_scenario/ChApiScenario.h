// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_API_SCENARIO_H
#define CH_API_SCENARIO_H

#include "chrono/ChVersion.h"
#include "chrono/core/ChPlatform.h"

// When compiling this library, remember to define CH_API_COMPILE_SCENARIO so that symbols tagged with
// 'ChApiScenario' will be marked as exported. Otherwise, just do not define it if you link the library
// to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_SCENARIO)
    #define ChApiScenario ChApiEXPORT
#else
    #define ChApiScenario ChApiIMPORT
#endif

/**
    @defgroup scenario_module Scenario module
    @brief Road network and driving scenario support (ASAM OpenDRIVE, ASAM OpenSCENARIO).

    This module gives Chrono a representation of a *road network* — lanes, lane widths, junction
    topology, speed limits — and of *driving scenarios* defined over it. Both are delegated to
    [esmini](https://github.com/esmini/esmini), a BSD-licensed OpenDRIVE/OpenSCENARIO engine, which
    supplies the road model and scenario execution while Chrono supplies vehicle dynamics and sensing.

    This complements, rather than replaces, ChVehicle's CRGTerrain: ASAM OpenCRG describes the road
    *surface* (elevation, roughness, friction), while ASAM OpenDRIVE describes the road *network*
    (the logical lane graph). The two standards are designed to be used together.
*/

#endif
