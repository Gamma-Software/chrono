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
// HMMWV rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/tract/Tract_RackPinion.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Tract_RackPinion::Tract_RackPinion(const std::string& name, const SteeringData& data) : ChRackPinion(name) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
