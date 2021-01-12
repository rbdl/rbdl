/*
 * 
 * Copyright (c) 2020 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */


#ifndef BALANCETOOLKIT_H_
#define BALANCETOOLKIT_H_


#include <rbdl/rbdl_math.h>
#include <rbdl/Model.h>
#include <limits>

namespace RigidBodyDynamics {
  namespace Addons {
    namespace Balance{

class BalanceToolkit
{
  public:

  /**
    This struct contains the location of the balance restoring step location 
    evaluated by the 3DFPE algorithm described by Millard et al. and all
    of the additional information needed to judge if the assumptions of the
    FPE3D are being met, and if not, how much error is introduced. All 
    quantities are expressed in MKS units. If you cannot get access to Millard
    et al., note that a similar derivation appears in Sloot et al. but that
    some variable names have changed (the u & v-directions in Millard et al. are
    the s & t directions in Sloot et al.).

    Millard, M., McPhee, J., & Kubica, E. (2012). Foot placement and balance 
     in 3D. Journal of Computational and Nonlinear Dynamics, 7(2), 021015.

    Sloot LH, Millard M, Werner C, Mombaur K. Slow but Steady: Similar 
    Sit-to-Stand Balance at Seat-Off in Older vs. Younger Adults. Frontiers in 
    sports and active living. 2020;2.

    A small apology for the terse variable names. These terse 
    names are needed to do write these functions without making obvious errors.
    Nice human-readable variable names could have been used, however, time and 
    memory would have to have been spent to copy these over. In case you need to 
    modify the guts of this code a description of the variable convention appears
    below.

    The variables follow a notation convention that is similar to that used in 
    multibody-dynamics. 
                                                  
    Global quantities           : [ Descriptor ] [Resolved in frame]
     e.g. g0 : gravity vector in K0, the inertial frame

    Single-point/body quantites : [ Descriptor ] [About Point      ] [Resolved in frame] [*optional* operator]
    Two-point quantites         : [ Descriptor ] [From Point       ] [To Point         ] [Resolved in frame] [*optional* operator]

    Global descriptors
      g : gravity

    Single Point/Body Descriptors 

      e  : unit vector
      rm : rotation matrix
      K  : a frame: consists of a point in space and a rotation matrix
      J  : inertia
      H  : angular momentum

      (specific to this code)
      u : vector in the direction of a balancing step (See Fig. 6 of Millard et al.)
      v : vertical vector
      n : vector normal to the the plane
      P : the projection frame: centered at the CoM ground projection and 
          with a u, k direction vectors that are perpendicular to the 
          horizontal component of the angular momentum vector (when taken)
          about the CoM ground projection.

    Two-Point Descriptors:
      r: position
      v: velocity
      w: angular velocity

    Point
      C: the whole-body-center of mass (COM)
      G: the projection plane origin: the ground projection of the COM      
      0: the origin of the lab/inertial frame
      S: the contact surface frame origin
      
    Frames
      0: the lab/inertial frame
      S: the contact surface frame origin


    (optional) Operator
    x:  Cross-product matrix. For example r0C0 is the vector, r0C0x is the
        cross-product matrix of r0C0.
    p:  projection

    Examples:
      r0F0 : (r) position vector 
             (0) from the base frame origin
             (C) to the foot placement estimator location
             (0) resolved in the coordinates of the base frame

      JC0 :  (J) inertia matrix
             (C) about the center-of-mass (CoM) 
             (0) resolved in the coordinates of the lab frame

    */
    struct FootPlacementEstimatorInfo{
      /** Numerical 3DFPE constraint error (Eqn. 45 of Millard et al.)
      */
      double f               ;
      /** The foot contact angle: the angle between the gravity vector and the 
          vector between the CoM and the contact location (See Fig. 6)
      */
      double phi             ;
      /** The location of the foot-placement-estimator in the base frame.
      */
      Math::Vector3d r0F0    ;
      
      /** The percentage of HP0 (the angular momentum of the whole body when 
      evaluated by the center-of-mass ground projection point) that is in the 
      vertical direction. Momentum in this direction is assumed to be zero by 
      the FPE.
      */      
      double projectionError ;

      /** Direction vector that is normal to the projection plane (Eqn. 43), 
          and is parallel to the horizontal component of HP0.
      */
      Math::Vector3d  n      ;
      /** Direction vector defined by n cross k, where k is the vertical 
          direction vector. */
      Math::Vector3d  u      ;
      /** Vertical direction vector */
      Math::Vector3d  k      ;
      /** Vector to the COM*/
      Math::Vector3d r0C0    ;
      /** Vector to the COM ground projection*/
      Math::Vector3d r0G0    ;
      /** Whole body angular momentum about the center of mass*/
      Math::Matrix3d HC0     ;
      /** Whole body moment of inertia the center of mass*/
      Math::Matrix3d IC0     ;
      /** Average angular velocity of the center of mass in the n direction*/
      Math::Vector3d w0C0    ;
      /** Whole body angular momentum about the center of mass*/
      Math::Matrix3d HG0     ;
      /** Whole body moment of inertia about the center of mass ground
       * projection*/
      Math::Matrix3d IG0     ;
      /** Average angular velocity about the center-of-mass ground projection*/      
      Math::Vector3d w0G0    ;
      /** Height of the center-of-mass*/      
      double h               ;
      Math::Vector3d w0C0plus;      
      /** Whole body moment of inertia about the center of mass in the 
      n direction*/
      double IC0n            ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. phi */
      double Df_Dphi         ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. the pre-contact
      whole body angular velocity about the center of mass in the n direction*/      
      double Df_Dw0C0n       ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. COM height*/            
      double Df_Dh           ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. the pre-contact
      COM velocity in the u direction*/                  
      double Df_Dv0C0u       ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. the pre-contact
      COM velocity in the k direction*/                        
      double Df_Dv0C0k       ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. the whole 
      body moment of inertia about the center of mass in the n direction*/                        
      double Df_DJ           ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. the total system mass*/
      double Df_Dm           ;
      /** Partial derivative of the FPE constraint equation  
      (Eqn. 45 of Millard et al.) f w.r.t. to the force of gravity*/      
      double Df_Dg           ;
      /** The sensitivity of the FPE step length w.r.t. small changes in the
      distance l between the COM and the FPE.*/            
      double Ds_Dl           ;
      /** The sensitivity of the FPE step length w.r.t. small changes in J.*/                  
      double Ds_DJ           ;
      /** The sensitivity of the FPE step length w.r.t. small changes in E,
      the peak potential energy obtained by the system as it comes to balance
      over its foot after contact.*/                       
      double Ds_DE           ;
      /** The sensitivity of the FPE step length w.r.t. small changes in v0C0
      the velocity of the center of mass in the u direction*/
      double Ds_Dv0C0u       ;
      /** The sensitivity of the FPE step length w.r.t. small changes in v0C0
      the velocity of the center of mass in k, the vertical direction*/
      double Ds_Dv0C0k       ;
      /** The sensitivity of the FPE step length w.r.t. small changes in the
      whole body angular velocity about the center of mass in the n direction.*/
      double Ds_Dw0C0n       ;
      /** The sensitivity of the FPE angle phi w.r.t. small changes in l, the 
      distance between the COM and the FPE location*/      
      double Dphi_Dl         ;
      /** The sensitivity of the FPE angle phi w.r.t. small changes in J, the 
      moment of inertia of the body about the COM in the n direction*/            
      double Dphi_DJ         ;
      /** The sensitivity of the FPE angle phi w.r.t. small changes in E, the
      post-contact sytem energy*/                  
      double Dphi_DE         ;
      /** The sensitivity of the FPE angle phi w.r.t. small changes in 
      v0C0 the velocity of the center of mass in the u-direction*/      
      double Dphi_Dv0C0u     ;
      /** The sensitivity of the FPE angle phi w.r.t. small changes in v0C0
      the velocity of the center of mass in k, the vertical direction*/      
      double Dphi_Dv0C0k     ;
      /** The sensitivity of the FPE angle phi w.r.t. small changes in the
      whole body angular velocity about the center of mass in the n direction.*/
      double Dphi_Dw0C0n     ; 
      /** The leg length of the FPE model: the distance from the COM to the 
      FPE*/
      double l               ;
      /** The post-contact system energy of the FPE model*/
      double E               ;

      FootPlacementEstimatorInfo():
        f(std::numeric_limits<double>::signaling_NaN()),
        phi(std::numeric_limits<double>::signaling_NaN()),
        r0F0( Math::Vector3dZero ),
        projectionError(std::numeric_limits<double>::signaling_NaN()),
        n( Math::Vector3dZero ),
        u( Math::Vector3dZero ),
        k( Math::Vector3dZero ),
        r0C0( Math::Vector3dZero ),
        r0G0( Math::Vector3dZero ),
        HC0( Math::Matrix3dZero ),
        IC0( Math::Matrix3dZero ),
        w0C0( Math::Vector3dZero ),
        HG0( Math::Matrix3dZero ),
        IG0( Math::Matrix3dZero ),
        w0G0( Math::Vector3dZero ),      
        h(std::numeric_limits<double>::signaling_NaN()),
        w0C0plus(Math::Vector3dZero),
        I(std::numeric_limits<double>::signaling_NaN()),
        Df_Dphi(std::numeric_limits<double>::signaling_NaN()),
        Df_Dw0C0n(std::numeric_limits<double>::signaling_NaN()),
        Df_Dh(std::numeric_limits<double>::signaling_NaN()),
        Df_Dv0C0u(std::numeric_limits<double>::signaling_NaN()),
        Df_Dv0C0k(std::numeric_limits<double>::signaling_NaN()),
        Df_DJ(std::numeric_limits<double>::signaling_NaN()),
        Df_Dm(std::numeric_limits<double>::signaling_NaN()),
        Df_Dg(std::numeric_limits<double>::signaling_NaN()),
        Ds_Dl(std::numeric_limits<double>::signaling_NaN()),
        Ds_DJ(std::numeric_limits<double>::signaling_NaN()),
        Ds_DE(std::numeric_limits<double>::signaling_NaN()),
        Ds_Dv0C0u(std::numeric_limits<double>::signaling_NaN()),
        Ds_Dv0C0k(std::numeric_limits<double>::signaling_NaN()),
        Ds_Dw0C0n(std::numeric_limits<double>::signaling_NaN()),
        Dphi_Dl(std::numeric_limits<double>::signaling_NaN()),
        Dphi_DJ(std::numeric_limits<double>::signaling_NaN()),
        Dphi_DE(std::numeric_limits<double>::signaling_NaN()),
        Dphi_Dv0C0u(std::numeric_limits<double>::signaling_NaN()),
        Dphi_Dv0C0k(std::numeric_limits<double>::signaling_NaN()),
        Dphi_Dw0C0n(std::numeric_limits<double>::signaling_NaN()),
        l(std::numeric_limits<double>::signaling_NaN()),
        E(std::numeric_limits<double>::signaling_NaN()){};

    };

              

    /**


      Millard M, McPhee J, Kubica E. Foot placement and balance in 3D. 
      Journal of computational and nonlinear dynamics. 2012 Apr 1;7(2).
    */
    void CalculateFootPlacementEstimator(
      Model &model,
      Math::VectorNd& q,
      Math::VectorNd& qdot,
      Math::Vector3d& pointOnGroundPlane,
      Math::Vector3d& groundPlaneNormal,
      FootPlacementEstimatorInfo& fpeInfo,
      bool evaluate_derivatives=false,
      bool update_kinematics=false);

};

    }
  }
}

#endif 
