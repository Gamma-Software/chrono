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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Front and Rear HMMWV suspension subsystems (double A-arm).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/Trailor/Trailor_RigidSuspension.h"

namespace chrono {
namespace vehicle {
namespace trailor {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;

const double Trailor_RigidSuspensionFront::m_spindleMass = 14.705;

const double Trailor_RigidSuspensionFront::m_spindleRadius = 0.10;
const double Trailor_RigidSuspensionFront::m_spindleWidth = 0.06;

// TODO: Fix these values
const ChVector<> Trailor_RigidSuspensionFront::m_spindleInertia(0.04117, 0.07352, 0.04117);

const double Trailor_RigidSuspensionFront::m_axleInertia = 0.4;

// -----------------------------------------------------------------------------

const double Trailor_RigidSuspensionRear::m_spindleMass = 14.705;

const double Trailor_RigidSuspensionRear::m_spindleRadius = 0.10;
const double Trailor_RigidSuspensionRear::m_spindleWidth = 0.06;

// TODO: Fix these values
const ChVector<> Trailor_RigidSuspensionRear::m_spindleInertia(0.04117, 0.07352, 0.04117);

const double Trailor_RigidSuspensionRear::m_axleInertia = 0.4;

// -----------------------------------------------------------------------------
// Constructors
// -----------------------------------------------------------------------------
Trailor_RigidSuspensionFront::Trailor_RigidSuspensionFront(const std::string& name) : ChRigidSuspension(name) {
}

Trailor_RigidSuspensionRear::Trailor_RigidSuspensionRear(const std::string& name) : ChRigidSuspension(name) {
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Trailor_RigidSuspensionFront::~Trailor_RigidSuspensionFront() {
}

Trailor_RigidSuspensionRear::~Trailor_RigidSuspensionRear() {
}

// -----------------------------------------------------------------------------
// Implementations of the getLocation() virtual methods.
// -----------------------------------------------------------------------------

const ChVector<> Trailor_RigidSuspensionFront::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(-1.59, 35.815, -1.035);
        default:
            return ChVector<>(0, 0, 0);
    }
}

const ChVector<> Trailor_RigidSuspensionRear::getLocation(PointId which) {
    switch (which) {
        case SPINDLE:
            return in2m * ChVector<>(1.40, 35.815, -1.035);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace Trailor
}  // end namespace vehicle
}  // end namespace chrono
