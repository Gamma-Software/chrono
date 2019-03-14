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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// HMMWV wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/tract/Tract_Wheel.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Tract_Wheel::Tract_Wheel(const std::string& name, const WheelData& data) : ChWheel(name), m_data(data){}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Tract_Wheel::AddVisualizationAssets(VisualizationType vis) {
    ChWheel::AddVisualizationAssets(vis);
}

void Tract_Wheel::RemoveVisualizationAssets() {
    ChWheel::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by HMMWV_Wheel::AddVisualizationAssets.
    // This is important for the ChWheel object because a tire may add its own assets
    // to the same body (the spindle).
    auto it = std::find(m_spindle->GetAssets().begin(), m_spindle->GetAssets().end(), m_trimesh_shape);
    if (it != m_spindle->GetAssets().end())
        m_spindle->GetAssets().erase(it);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
