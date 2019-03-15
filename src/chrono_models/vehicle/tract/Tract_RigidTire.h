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
// HMMWV rigid tire subsystem
//
// =============================================================================

#pragma once

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace tract {


struct TireData
{
    double width = 0.254;
    double radius = 0.49;

    double mass = 37.6;
    ChVector<> inertia = ChVector<>(3.84, 6.69, 3.84);

    std::string meshName = "tract_tire_POV_geom";
    std::string meshFile = "tract/tract_tire.obj";
};

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Rigid tire model for the HMMWV vehicle.
class CH_MODELS_API Tract_RigidTire : public ChRigidTire {
  public:
    Tract_RigidTire(const std::string& name, const TireData& data, bool use_mesh = false);
    ~Tract_RigidTire() {}

    virtual double GetRadius() const override { return m_data.radius; }
    virtual double GetWidth() const override { return m_data.width; }
    virtual double GetMass() const override { return m_data.mass; }
    virtual ChVector<> GetInertia() const override { return m_data.inertia; }

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    TireData m_data;

    static const std::string m_meshName;
    static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
