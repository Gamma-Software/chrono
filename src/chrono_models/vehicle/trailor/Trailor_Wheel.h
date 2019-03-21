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

#pragma once

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace trailor {

struct WheelData
{
    double mass = 18.8;
    double radius = 0.39;
    double width = 0.22;
    ChVector<> inertia = ChVector<>(0.113, 0.113, 0.113);
};

/// @addtogroup vehicle_models_hmmwv
/// @{

/// HMMWV wheel base class.
class CH_MODELS_API Trailor_Wheel : public ChWheel {
  public:
    Trailor_Wheel(const std::string& name, const WheelData& data);
    ~Trailor_Wheel() {}

    virtual double GetMass() const override { return m_data.mass; }
    virtual ChVector<> GetInertia() const override { return m_data.inertia; }
    virtual double GetRadius() const override { return m_data.radius; }
    virtual double GetWidth() const override { return m_data.width; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override;

  protected:
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;

    WheelData m_data;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
