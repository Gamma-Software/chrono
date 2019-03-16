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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Rear Tract suspension subsystems (simple leafspring work a like).
//
// These concrete suspension subsystems are defined with respect to right-handed
// frames with X pointing towards the front, Y to the left, and Z up (as imposed
// by the base class ChDoubleWishbone) and origins at the midpoint between the
// lower control arms' connection points to the chassis.
//
// All point locations are provided for the left half of the suspension.
//
// =============================================================================

#include "chrono_models/vehicle/tract/Tract_LeafspringAxle.h"

namespace chrono {
namespace vehicle {
namespace tract {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;
static const double lbfpin2Npm = 175.12677;


// ---------------------------------------------------------------------------------------
// Tract spring functor class - implements a linear spring + bump stop + rebound stop
// ---------------------------------------------------------------------------------------
class Tract_SpringForceRear : public ChLinkSpringCB::ForceFunctor {
  public:
    Tract_SpringForceRear(double spring_constant, double min_length, double max_length);

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override;

  private:
    double m_spring_constant;
    double m_min_length;
    double m_max_length;

    ChFunction_Recorder m_bump;

};

Tract_SpringForceRear::Tract_SpringForceRear(double spring_constant, double min_length, double max_length) :
    m_spring_constant(spring_constant),
    m_min_length(min_length),
    m_max_length(max_length)  {

    // From ADAMS/Car
    m_bump.AddPoint(0.0,          0.0);
    m_bump.AddPoint(2.0e-3,     200.0);
    m_bump.AddPoint(4.0e-3,     400.0);
    m_bump.AddPoint(6.0e-3,     600.0);
    m_bump.AddPoint(8.0e-3,     800.0);
    m_bump.AddPoint(10.0e-3,   1000.0);
    m_bump.AddPoint(20.0e-3,   2500.0);
    m_bump.AddPoint(30.0e-3,   4500.0);
    m_bump.AddPoint(40.0e-3,   7500.0);
    m_bump.AddPoint(50.0e-3,  12500.0);

}

double Tract_SpringForceRear::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
    /*
     *
    */

    double force = 0;

    double defl_spring  = rest_length - length;
    double defl_bump    = 0.0;
    double defl_rebound = 0.0;

    if(length < m_min_length) {
        defl_bump = m_min_length - length;
    }

    if(length > m_max_length) {
        defl_rebound = length - m_max_length;
    }

    force = defl_spring * m_spring_constant + m_bump.Get_y(defl_bump) - m_bump.Get_y(defl_rebound);

    return force;
}

// -----------------------------------------------------------------------------
// Tract shock functor class - implements a nonlinear damper
// -----------------------------------------------------------------------------
class Tract_ShockForceRear : public ChLinkSpringCB::ForceFunctor {
  public:
    Tract_ShockForceRear(double compression_slope,
                     double compression_degressivity,
                     double expansion_slope,
                     double expansion_degressivity);

    virtual double operator()(double time,
                              double rest_length,
                              double length,
                              double vel,
                              ChLinkSpringCB* link) override;

  private:
    double m_slope_compr;
    double m_slope_expand;
    double m_degres_compr;
    double m_degres_expand;
};

Tract_ShockForceRear::Tract_ShockForceRear(double compression_slope,
                                   double compression_degressivity,
                                   double expansion_slope,
                                   double expansion_degressivity)
    : m_slope_compr(compression_slope),
      m_degres_compr(compression_degressivity),
      m_slope_expand(expansion_slope),
      m_degres_expand(expansion_degressivity) {}

double Tract_ShockForceRear::operator()(double time, double rest_length, double length, double vel, ChLinkSpringCB* link) {
    /*
     * Simple model of a degressive damping characteristic
    */

    double force = 0;

    // Calculate Damping Force
    if (vel >= 0) {
        force = -m_slope_expand / (1.0 + m_degres_expand * std::abs(vel)) * vel;
    } else {
        force = -m_slope_compr / (1.0 + m_degres_compr * std::abs(vel)) * vel;
    }

    return force;
}


Tract_LeafspringAxle::Tract_LeafspringAxle(const std::string& name,
    const SuspensionData& data) : ChLeafspringAxle(name), m_data(data) {
/*
    m_springForceCB = new LinearSpringForce(m_springCoefficient  // coefficient for linear spring
                                            );

    m_shockForceCB = new LinearDamperForce(m_damperCoefficient  // coefficient for linear damper
                    );
*/
    m_springForceCB = new Tract_SpringForceRear(m_data.springCoefficient,
        m_data.springMinLength,
        m_data.springMaxLength);

    m_shockForceCB = new Tract_ShockForceRear(m_data.damperCoefficient,
        m_data.damperDegressivityCompression,
        m_data.damperCoefficient,
        m_data.damperDegressivityExpansion);
}

// -----------------------------------------------------------------------------
// Destructors
// -----------------------------------------------------------------------------
Tract_LeafspringAxle::~Tract_LeafspringAxle() {
    delete m_springForceCB;
    delete m_shockForceCB;
}

const ChVector<> Tract_LeafspringAxle::getLocation(PointId which) {
    switch (which) {
        case SPRING_A:
            return ChVector<>(0.0, 0.5142, m_data.axleTubeRadius);
        case SPRING_C:
            return ChVector<>(0.0, 0.5142, m_data.axleTubeRadius+m_data.springDesignLength);
        case SHOCK_A:
            return ChVector<>(-0.125, 0.441, -0.0507);
        case SHOCK_C:
            return ChVector<>(-0.3648,  0.4193, 0.4298);
        case SPINDLE:
            return ChVector<>(0.0, m_data.wheelTrack / 2, 0.0);
        case UPRIGHT:
            return ChVector<>(0.0, 0.76 / 2, 0.0);
        case TIEROD_U:
            return ChVector<>(-0.23, 0.654 / 2, 0.0);
        case TIEROD_C:
            return ChVector<>(-0.23, 0.0, 0.0);
        default:
            return ChVector<>(0, 0, 0);
    }
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
