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
// HMMWV simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/tract/Tract_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Constructor of HMMWV_SimpleDriveline.
// -----------------------------------------------------------------------------
Tract_SimpleDriveline::Tract_SimpleDriveline(const std::string& name, const DriveLineData& data) : ChSimpleDriveline(name), m_data(data) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
