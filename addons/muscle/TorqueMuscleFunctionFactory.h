#ifndef TORQUEMUSCLEFUNCTIONFACTORY_H_
#define TORQUEMUSCLEFUNCTIONFACTORY_H_
/*
 * Muscle addon for RBDL
 * Copyright (c) 2016 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */
#include "../geometry/SmoothSegmentedFunction.h"
#include "../geometry/SegmentedQuinticBezierToolkit.h"

#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>

namespace RigidBodyDynamics {
  namespace Addons {
  namespace Muscle{

class TorqueMuscleFunctionFactory
{
  public:


  /**
  This is a function that will produce a C2 (continuous to the second
  derivative) active torque angle curve. This Bezier curve has been
  fitted to match the active-torque-angle curve described in

  Anderson, Dennis E., Michael L. Madigan, and Maury A. Nussbaum. 
  "Maximum voluntary joint torque as a function of joint angle and 
  angular velocity: model development and application to the lower 
  limb." Journal of biomechanics 40, no. 14 (2007): 3105-3113.

  but note that its range is normalized to [0,1].

  @param c2 (radians)
   The active-torque-angle width parameter. The parameter c2
   is defined by Anderson et al. as

  c2 = pi/(theta_max - theta_min). 

  @param c3 : (radians)
  Then angle which has the largest active-torque.

  @param curveName The name of the joint torque this curve applies to. This
    curve name should have the name of the joint and the
    direction (e.g. hipExtensionTorqueMuscle) so that if
    this curve ever causes an exception, a user friendly
    error message can be displayed to the end user to help
    them debug their model.

  @param smoothSegmentedFunctionToUpdate
      A SmoothSegmentedFunction object that will be erased and filled with 
      the coefficients that are defined by this curve.


*/
static void createAnderson2007ActiveTorqueAngleCurve(
    double c2, 
    double c3,
    const std::string& curveName,
    RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
      smoothSegmentedFunctionToUpdate);

  /**
  This is a function that will produce a C2 (continuous to the second
  derivative) active torque (angular) velocity curve. This Bezier curve 
  has been fitted to match the active-torque-angle curve described in

  Anderson, Dennis E., Michael L. Madigan, and Maury A. Nussbaum. 
  "Maximum voluntary joint torque as a function of joint angle and 
  angular velocity: model development and application to the lower 
  limb." Journal of biomechanics 40, no. 14 (2007): 3105-3113.

  While the concentric side of the Bezier curve and the original 
  formulation match, the eccentric side does not: the equations 
  Anderson et al. chose decrease down to 0 rapidly. Since Anderson
  et al. did not collect data at the higher eccentric velocities the
  oddities in their chosen curves are likely due to the parameterization
  they chose. The eccentric side of the Bezier curve will be fitted
  so that, if possible, it passes close to the value of the original
  curves for theta = -60 deg/s within the limits imposed by 
  minEccentricMultiplier and maxEccentricMultiplier.

  @param c4 (rads/s)
  Angular velocity when the torque is 75% of the maximum 
  isometric torque.

  @param c5 (rads/s)
  Angular velocity when the torque is 50% of the maximum 
  isometric torque.

  @param c6 
  Multiplier that Anderson et al. uses to describe the 
  change in slope of the curve as the contraction velocity
  changes sign from + to -.

  @param minEccentricMultiplier
  The minimum value of the torque-(angular)-velocity curve 
  tends to at large eccentric contraction velocities. Note
  minEccentricMultiplier > 1.0

  @param maxEccentricMultiplier
  The value of the torque-(angular)-velocity curve tends
  to at large eccentric contraction velocities. Note
  maxEccentricMultiplier > minEccentricMultiplier.

  @param curveName The name of the joint torque this curve applies to. This
    curve name should have the name of the joint and the
    direction (e.g. hipExtensionTorqueMuscle) so that if
    this curve ever causes an exception, a user friendly
    error message can be displayed to the end user to help
    them debug their model.

  @param smoothSegmentedFunctionToUpdate
      A SmoothSegmentedFunction object that will be erased and filled with 
      the coefficients that are defined by this curve.

  */
  static void createAnderson2007ActiveTorqueVelocityCurve(
      double c4, 
      double c5,
      double c6,
      double minEccentricMultiplier,
      double maxEccentricMultiplier,
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate);

  /**
  This is a function that will produce a C2 (continuous to the second
  derivative) passive torque angle curve described in

  Anderson, Dennis E., Michael L. Madigan, and Maury A. Nussbaum. 
  "Maximum voluntary joint torque as a function of joint angle and 
  angular velocity: model development and application to the lower 
  limb." Journal of biomechanics 40, no. 14 (2007): 3105-3113.

  Note the following differences between this implementation and
  the original equations presented in Anderson et al.:
  
  1. This function will return a curve that is fitted to the
  positive side of the curve defined by the coefficients 
  b1, k1, b2, and k2. Because of the sign convention employed by
  Anderson et al. the positive side of the curve corresponds to
  the passive curve generated by the torque actuator associated
  with the rest of the coefficients.

  2. This function has been normalized so that a value of 1.0 
   corresponds to one-maximum-isometric-active-contraction
   torque, or c1*subjectWeightInNewtons*subjectHeightInMeters.

  @param scale
   The scaling factor used on the c1 column in Table 3 of 
   Anderson et al.:

   scale = subjectWeightInNewtons * subjectHeightInMeters

  @param c1
   The normalized c1 parameter listed in Tabel 3 of 
   Anderson et al.

  @param b1
   The passive torque angle curve parameter used in 
   Anderson et al.'s Eqn. 1:

   torquePassive = b1*exp(k1*theta) + b2*exp(k2*theta)

  @param k1
   The term k1 in Anderson et al.'s Eqn. 1.

  @param b2
   The term b2 in Anderson et al.'s Eqn. 1.

  @param k2
   The term k2 in Anderson et al.'s Eqn. 1.

  @param curveName The name of the joint torque this curve applies to. This
     curve name should have the name of the joint and the
     direction (e.g. hipExtensionTorqueMuscle) so that if
     this curve ever causes an exception, a user friendly
     error message can be displayed to the end user to help
     them debug their model.

  @param smoothSegmentedFunctionToUpdate
      A SmoothSegmentedFunction object that will be erased and filled with 
      the coefficients that are defined by this curve. 
  */
  static void createAnderson2007PassiveTorqueAngleCurve(
      double scale,
      double c1,
      double b1,
      double k1,
      double b2,
      double k2,      
      const std::string& curveName,
      RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
        smoothSegmentedFunctionToUpdate);

};
}
}
}
#endif //TORQUEMUSCLEFUNCTIONFACTORY_H_
