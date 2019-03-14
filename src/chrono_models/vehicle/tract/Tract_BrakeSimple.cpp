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
// Authors: Alessandro Tasora
// =============================================================================
//
// HMMWV simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/tract/Tract_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Tract_BrakeSimple::Tract_BrakeSimple(const std::string& name, const BrakeData& data) : ChBrakeSimple(name), m_data(data) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
