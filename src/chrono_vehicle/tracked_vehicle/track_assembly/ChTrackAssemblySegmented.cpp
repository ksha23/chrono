// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for segmented track assemblies.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#include <cmath>

#include "chrono_vehicle/tracked_vehicle/track_assembly/ChTrackAssemblySegmented.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeSegmented.h"

namespace chrono {
namespace vehicle {

ChTrackAssemblySegmented::ChTrackAssemblySegmented(const std::string& name, VehicleSide side)
    : ChTrackAssembly(name, side), m_torque_funct(nullptr), m_bushing_data(nullptr) {}

void ChTrackAssemblySegmented::EnableTrackBendingStiffness(bool val) {
    if (!m_torque_funct)
        return;

    for (size_t i = 0; i < GetNumTrackShoes(); i++) {
        auto shoe = std::static_pointer_cast<ChTrackShoeSegmented>(GetTrackShoe(i));
        shoe->EnableTrackBendingStiffness(val);
    }
}

double ChTrackAssemblySegmented::TrackBendingFunctor::evaluate(double time,
                                                               double rest_angle,
                                                               double angle,
                                                               double vel,
                                                               const ChLinkRSDA& link) {
    // Wrap angle into [-pi, +pi].
    // ChLinkRSDA passes its turn-adjusted relative angle, which can be an arbitrary number of full turns
    // outside the range -- measured at 2.99 turns on an M113 road wheel after the link unwinds.
    //
    // Done branch-free rather than by looping. A loop does not terminate on every input this can receive:
    // at -inf the addition is a fixed point, and beyond about 1e17 a step of 2*pi is below half an ulp, so
    // it is a no-op. Both spin forever. This form is O(1), returns the same value everywhere a loop would
    // have converged, and propagates a non-finite input instead of hanging on it.
    angle -= CH_2PI * std::round(angle / CH_2PI);
    // Linear spring-damper (assume 0 rest angle)
    return m_t - m_k * angle - m_c * vel;
}

}  // end namespace vehicle
}  // end namespace chrono
