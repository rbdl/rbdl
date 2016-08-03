#ifndef MILLARD2016TORQUEMUSCLE_H_
#define MILLARD2016TORQUEMUSCLE_H_

/* 
 * RBDL - Rigid Body Dynamics Library: Addon : forceElements
 * Copyright (c) 2016 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <vector>
#include <rbdl/rbdl_math.h>
#include "../geometry/SmoothSegmentedFunction.h"


namespace RigidBodyDynamics {
    namespace Addons {
      namespace Muscle{

        /**
          This struct contains an enumerated list of the data sets which
          constain torque muscles. Thus far this list includes

          -Anderson2007: Table 3 from Anderson et al.       
          -Gymnast: A set of torque muscles for the whole body (in progress)
                    
          For details of these data sets please refer to the main description 
          of this class.

        */
        const static struct DataSet{
          enum item{
              Anderson2007 = 0,
              Gymnast,
              Last
          };
          const static char* names[];
          DataSet(){}
        } DataSet;


        /**
          This struct contains an enumerated list of the genders for which 
          data torque data has been reported.
        */
        const static struct GenderSet{
          enum item {
              Male = 0,
              Female,
              Last
          };
          const static char* names[];
          GenderSet(){}
        } GenderSet;

        /**
          This struct contains an enumerated list of the age groups for which 
          data torque data has been reported.
        */
        const static struct AgeGroupSet{
          enum item {
              Young18To25 = 0,
              Middle55To65,
              SeniorOver65,
              Last
          };
          const static char* names[];
          AgeGroupSet(){}
        } AgeGroupSet;

        /**
          This struct contains an enumerated list of the joint-torque-directions 
          for which data torque data has been reported.
        */
        const static struct JointTorqueSet{
          enum item{
            HipExtension                  = 0,
            HipFlexion                    = 1,
            KneeExtension                 = 2,
            KneeFlexion                   = 3,
            AnkleExtension                = 4,
            AnkleFlexion                  = 5,
            ElbowExtension                = 6,
            ElbowFlexion                  = 7,
            ShoulderExtension             = 8,
            ShoulderFlexion               = 9,
            WristExtension                = 10,
            WristFlexion                  = 11,
            ShoulderHorizontalAdduction   = 12,
            ShoulderHorizontalAbduction   = 13,
            ShoulderInternalRotation      = 14,
            ShoulderExternalRotation      = 15,
            WristUlnarDeviation           = 16,
            WristRadialDeviation          = 17,
            WristPronation                = 18,
            WristSupination               = 19,
            Last                          = 20
          };
          const static char* names[];
          JointTorqueSet(){}
        } JointTorqueSet;

        /**
          This struct contains 3 enumerated lists (Gender, AgeGroup, 
          JointTorque) that lists the genders, age groups, and 
          joint-torque-directions for which the Anderson2007 data set contains
          data. Please refer to the class description for more details about 
          this data set and how to use it.
        */
        const static struct Anderson2007{
          enum Gender {
              Male   = GenderSet::Male,
              Female = GenderSet::Female,
              LastGender
          };
          enum AgeGroup {
              Young18To25   = AgeGroupSet::Young18To25,
              Middle55To65  = AgeGroupSet::Middle55To65,
              SeniorOver65  = AgeGroupSet::SeniorOver65,
              LastAgeGroup
          };
          enum JointTorque{
            HipExtension                  = JointTorqueSet::HipExtension  ,
            HipFlexion                    = JointTorqueSet::HipFlexion    ,
            KneeExtension                 = JointTorqueSet::KneeExtension ,
            KneeFlexion                   = JointTorqueSet::KneeFlexion   ,
            AnkleExtension                = JointTorqueSet::AnkleExtension,
            AnkleFlexion                  = JointTorqueSet::AnkleFlexion,
            LastJointTorque
          };
          const static char* GenderNames[];
          const static char* AgeGroupNames[];
          const static char* JointTorqueNames[];
          Anderson2007(){}
        } Anderson2007;

        /**
          This struct contains 3 enumerated lists (Gender, AgeGroup, 
          JointTorque) that lists the genders, age groups, and 
          joint-torque-directions for which the Gymnast data set contains
          data. At the present time Gymnast data set only contains data 
          appropriate for a young (18-25) elite male gymnast. Please refer
          to the main class description for details on this data set and how 
          to use it.
        */
        const static struct Gymnast{
          enum Gender {
              Male   = GenderSet::Male,
              LastGender
          };
          enum AgeGroup {
              Young18To25   = AgeGroupSet::Young18To25,
              LastAgeGroup
          };
          enum JointTorque {
            HipExtension                = JointTorqueSet::HipExtension       ,
            HipFlexion                  = JointTorqueSet::HipFlexion         ,
            KneeExtension               = JointTorqueSet::KneeExtension      ,
            KneeFlexion                 = JointTorqueSet::KneeFlexion        ,
            AnkleExtension              = JointTorqueSet::AnkleExtension     ,
            AnkleFlexion                = JointTorqueSet::AnkleFlexion       ,
            ElbowExtension              = JointTorqueSet::ElbowExtension     ,
            ElbowFlexion                = JointTorqueSet::ElbowFlexion       ,
            ShoulderExtension           = JointTorqueSet::ShoulderExtension  ,
            ShoulderFlexion             = JointTorqueSet::ShoulderFlexion    ,
            WristExtension              = JointTorqueSet::WristExtension     ,
            WristFlexion                = JointTorqueSet::WristFlexion       ,
            ShoulderHorizontalAdduction = 
              JointTorqueSet::ShoulderHorizontalAdduction,
            ShoulderHorizontalAbduction = 
              JointTorqueSet::ShoulderHorizontalAbduction,
            ShoulderInternalRotation    = 
              JointTorqueSet::ShoulderInternalRotation   ,
            ShoulderExternalRotation    = 
              JointTorqueSet::ShoulderExternalRotation   ,
            WristUlnarDeviation         = JointTorqueSet::WristUlnarDeviation ,
            WristRadialDeviation        = JointTorqueSet::WristRadialDeviation,
            WristPronation              = JointTorqueSet::WristPronation      ,
            WristSupination             = JointTorqueSet::WristSupination     ,
            LastJointTorque
          };
          const static char* GenderNames[];
          const static char* AgeGroupNames[];
          const static char* JointTorqueNames[];
          Gymnast(){}
        } Gymnast;

        /**
          This is a struct that contains subject-specific information that
          does not change for a given subject.

          @param gender
                 Male/Female. Selecting a gender that
                 is not present in a data set will result in
                 the program aborting and an error message
                 being printed to the terminal.

          @param ageGroup:
                 Presently the age group options include Young (18-25),
                 middle aged (55-65), and senior (>65). Selecting an
                 age group that is not present in a data set will result
                 in the program aborting and an error message
                 being printed to the terminal.

          @param subjectHeightInMeters
                  This parameter is used to scale from the normalized
                  curves reported by Anderson et al.
                  See the class description for details.

          @param subjectMassInKg
                  This parameter is used to scale from the normalized
                  curves reported by Anderson et al.
                  See the class description for details.
        */
        struct SubjectInformation{
          GenderSet::item gender;
          AgeGroupSet::item ageGroup;
          double heightInMeters;
          double massInKg;          
        };



        struct TorqueMuscleInfo{

            /**The angle of the joint (radians)*/
            double jointAngle;

            /**The angular velocity of the joint, where the sign convention
               is chosen by the user at the time the torque muscle is created 
               (radians/sec)*/
            double jointAngularVelocity;

            /**The angle that the muscle fiber spans (radians)*/   
            double fiberAngle;

            //The angle that the tendon spans (radians)              
            //double tendonAngle;

            /**The rate-of-angular-lengthening of the fiber.
               A positive sign is for a concentric contraction, 
               that is where the fibers are shortening. (radians/sec)*/   
            double fiberAngularVelocity;

            /*The rate-of-angular-lengthening of the tendon.
               A positive sign is for a concentric contraction, 
               that is where the tendon is shortening. (radians/sec)*/   
            //double tendonAngularVelocity;

            /** The normalized value of the passive-torque-angle curve. 
             Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            double fiberPassiveTorqueAngleMultiplier;

            /**The normalized value of the active-torque-angle curve. 
             Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            double fiberActiveTorqueAngleMultiplier;

            /**The normalized value of the torque-angular-velocity curve. 
               Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            double fiberActiveTorqueAngularVelocityMultiplier;

            /*The normalized value of the tendon-torque-angle curve. 
               Here a value of 1 means 1 maximum-isometric-torque. (Nm/Nm)*/
            //double tendonTorqueAngleMultiplier;
            
            /**The activation of the muscle*/
            double activation;

            /**The torque generated by the active element of the muscle
            fiber (Nm)*/
            double fiberActiveTorque;

            /**The torque generated by the passive element of the muscle 
            fiber (Nm)*/
            double fiberPassiveTorque;            

            /**The torque generated by the entire fiber (Nm)*/            
            double fiberTorque;

            /*The torque transmitted through the tendon across the joint (Nm)*/
            //double tendonTorque;

            /**The joint torque developed by the muscle. This is signed so 
            that it is consistent with the sign convention of the joint
            chosen by the user. (Nm)*/
            double jointTorque;

            /**The stiffness of the fiber (Nm/rad)*/
            double fiberStiffness;

            /*The stiffness of the tendon (Nm/rad)*/
            //double tendonStiffness;

            /** The stiffness of the joint. This is signed so 
            that it is consistent with the sign convention of the joint
            chosen by the user. (Nm/rad)*/
            double jointStiffness;
            
            /**The power output of the active fiber element. A positive
            power means that the fiber is contracting concentrically. 
            (Watts - Nm/s)*/
            double fiberActivePower;

            /**The power output of the passive fiber element. A positive
            power means that the passive element is recoiling concentrically
            (Watts - Nm/s)*/            
            double fiberPassivePower;

            /**The total power output of the fiber element.(Watts - Nm/s)*/  
            double fiberPower;
            
            /**The partial derivative of joint torque w.r.t the joint angle*/
            double DjointTorqueDjointAngle;

            /**The partial derivative of joint torque w.r.t the joint 
            angular velocity*/            
            double DjointTorqueDjointAngularVelocity;

            /**The partial derivative of joint torque w.r.t activation*/
            double DjointTorqueDactivation;

	    /*The power output of the tendon. A positive power means that 
            the tendon is physically shortening. (Watts - Nm/s)*/
            //double tendonPower;

            /** The power output by this muscle at the joint. This is 
            signed so that it is consistent with the sign convention of the 
            joint chosen by the user. (Nm/rad)*/
            double jointPower;

            TorqueMuscleInfo():
                jointAngle(nan("1")),
                jointAngularVelocity(nan("1")),
                fiberAngle(nan("1")),
                fiberAngularVelocity(nan("1")),                
                fiberPassiveTorqueAngleMultiplier(nan("1")),
                fiberActiveTorqueAngleMultiplier(nan("1")),
                fiberActiveTorqueAngularVelocityMultiplier(nan("1")),                
                activation(nan("1")),
                fiberActiveTorque(nan("1")),
                fiberPassiveTorque(nan("1")),
                fiberTorque(nan("1")),
                jointTorque(nan("1")),
                fiberStiffness(nan("1")),
                jointStiffness(nan("1")),
                fiberActivePower(nan("1")),
                fiberPassivePower(nan("1")),
                fiberPower(nan("1")),
                jointPower(nan("1")){}

        };

        /**
            This class implements a rigid-tendon torque muscle that has
            its characteristic curves fitted to the ones described in
            Anderson et al. Please refer to Anderson et al. and Millard 
            et al. if you are new to muscle modeling. There is a lot of
            standard language used in the description of this model that
            will not make sense to you without the basic background these
            two papers provide.



             <b>Data Set: Anderson2007</b>

            This data set uses the mean value of the coefficients published
            in Anderson et al. The standard deviation table has also been
            entered. However, since it is unclear how to use the standard
            deviation in a consistent way across all joints/parameters this
            table is not yet accessible through the constructor. This data
            set includes coefficients for the following

            -Number of subjects: 34
            -Gender: male and female
            -Age: young (18-25, 14 subjects), middle-aged (55-65, 14 subjects), senior (> 65, 6 subjects)
            -Joint: hip/knee/ankle
            -Direction: extension/flexion

            <b>Notes</b>
             -# Angles are plotted using units of degrees for readability. The
              actual curves are described in units of radians
             -# See Anderson et al. for further details.

           \image html fig_Anderson2007AllPostiveSigns.png "Characteristic from Anderson et al. 2007 [1]"


            <b>Data Set: Gymnast</b>

            This data set is an attempt at making enough torque muscles for a
            whole body. Since no single source in the literature comes close to
            measuring the characteristics of all of the joints, data from
            Jackson et al., Kentel et al., and Anderson et al.
            have been combined. Since the subjects used in these various studies
            are wildly different (Jackson et al. measured an elite male gymnast;
            Kentel et al. measured an elite tennis player; Anderson et al. measured,
            in the category of young male, a selection of active undergraduate
            students) scaling has been used to make the strength of the subject
            consistent. Scaling coefficients for the lower body, shoulders and
            elbow, and forearm/wrist using measurements that overlapped between
            datasets. Presently this data set includes curves for 10 joints:

            - Number of subjects: 1 elite gymnast (69.6 kg, 1.732 m)
            - Gender: male
            - Age: 21 years old
            - Joint and Directions available
              -# Ankle: flexion/extension (scaled from Anderson)
              -# Knee: flexion/extension (from Jackson)
              -# Hip: flexion/extension (from Jackson)
              -# Shoulder: flexion/extension (from Jackson)
              -# Shoulder: horizontal adduction/abduction (from Kentel, scaled to Jackson's subject)
              -# Shoulder: internal rotation/external rotation (from Kentel, scaled to Jackson's subject)
              -# Elbow: flexion/extension (from Kentel, scaled to Jackson's subject)
              -# Wrist: pronation/supination (from Kentel, scaled to Jackson's subject)
              -# Wrist: extension/flextion (from Jackson)
              -# Wrist: ulnar/radial deviation (from Kentel, scaled to Jackson's subject)
            - Missing Joint and directions
                -# Ankle inversion/eversion
                -# Hip adduction/abduction
                -# Hip internal rotation/external rotation
                -# Lumbar extension/flexion
                -# Lumbar bending
                -# Lumbar twisting
                -# Shoulder Adduction
                -# Shoulder Abduction
                -# Scapular elevation/depression
                -# Scapular adduction/abduction
                -# Scapular upward/downward rotation

              In all cases the curves have been fitted to Bezier curves that closesly
              follow the parametric curves described by Anderson et al. See 'Differences
              from Anderson et al.' for further details on these curves.

            <b>Notes</b>
            -# Angles are plotted using units of degrees for readability. The actual curves are described in units of radians
            -# Hip and Knee characteristics taken from Jackson. Ankle extension is from Anderson et al., scaled using Jackson-to-Anderson hip/knee strength ratios from Jackson ratios
            -# Shoulder horizontal adduction/abduction and internal/external rotation is a scaled version of the Kentel. Strength was scaled using the Jackson-to-Kentel shoulder flex/ext ratios.
            -# Elbow extension/flexion and forearm pronation/supination. Elbow strength scaled from Kentel using the ratio of maximum isometric shoulder ext/flextion between Kentel and Jackson. Forearm pronation/supination scaled using the maximum torque strength ratio of wrist extension/flextion between Kentel and Jackson
            -# Wrist ext/flexion directly from Jackson, while the curves for ulnar and radial deviation have been scaled (using the maximum isometric torque ratios of wrist extension and flexion from both models) from Kentel et al.
            -# All of these curves have been translated from their original form into the parameterization that Anderson et al. used in their paper.
            
            


           \image html fig_Gymnast_HipKneeAnkle.png " Hip/Knee/Ankle "
           \image html fig_Gymnast_Shoulder3Dof.png " Shoulder 3 DoF torques "
           \image html fig_Gymnast_ElbowForearm.png " Elbow flexion/extension and wrist pronation/supination"
           \image html fig_Gymnast_Wrist3Dof.png " Wrist ext/flex and ulnar/radial deviation"


            <b>Differences from Anderson et al. Parameterized Curves</b>

            The curves used in this implementation are 2nd order 2-dimensional 
            Bezier curves. The curves described in Anderson et al. were not 
            directly used because they are not continuous to the second
            derivative (a requirement for most gradient based optimization 
            routines). In general the fitted Bezier curves closely follow Anderson
            et al.'s. However there are some important locations where 
            the two curve sets deviate:

            -# Anderson et al.'s torque-velocity curve tends to large
            negative values for fast eccentric contractions. This is 
            in contrast to the literature which says that at large
            eccentric contractions the torque-velocity curve (or the 
            force-velocity-curve) tends to a value between 1.0 and 1.4.
            -# Anderson et al.'s torque-velcity curve for ankle extension
            did not cross the x-axis on the concentric side of the curve.
            This would endow the plantar flexors with super-human abilities.
            This error has been fixed by ensuring that the torque-velocity
            curve has a value of 0 for high-velocity concentric contractions.
            However, since there were not any actual measurments at these
            high velocities, we cannot be certain where the zero crossing is.

             <b>Coordinate Mapping</b>

            Anderson et al. chose a particular convention for measuring
            the angles of the hip, knee, and ankle joint --- see the figure
            for details. You will need to use the constructors appropriately
            so that

            -# the joint angle of your model is correctly mapped to the 
               fiber angle of the Millard2016TorqueMuscle;
            -# the sign of the muscle's output torque matches the 
               sign associated with your model.

            To map from your model's joint coordinates to the joint coordines
            used in Anderson et al.'s model (see the figure in the description)
            the followinq equation is used at the torque level

            \f$ jointTorque = signOfJointTorque*fiberTorque \f$

            where fiberTorque is the torque produced by Anderson et al.'s curves, 
            which is always positive. At the position level, the angles from your
            models joint angle to Anderson et al.'s joint angle (called fiberAngle)
            are mapped using

            \f$ fiberAngle = signOfJointAngleRelativeToAnderson2007*(jointAngle-jointAngleOffsetRelativeToAnderson2007). \f$
            
            Internally the sign of the fiber velocity follows signOfJointTorque so
            that the signs of joint power and muscle power are consistent.

            <b>Strength Scaling</b>

            The strength predicted by Anderson et al.'s curves should be taken
            as a good first approximation. While Anderson et al.'s data set is 
            the most comprehensive in the literature, they only measured torques 
            from active people: they did not include people at the extremes 
            (both very weak, and very strong), nor did they include children. 
            Finally, the torques produced by each subject were normalized by 
            subjectMassInKg*subjectHeightInM*accelerationDueToGravity. Strength
            is a strange phenomena which is not nicely normalized by just these
            quantites, and so the strength predicted by Anderson et al.'s curves
            might not fit your subject even if they are represented in Anderson 
            et al.'s data set.

            If you happen to know the maximum-isometric-active-torque (note this
            does not include the passive component) that your subject can produce,
            you can update the strength of the torque-muscle using the functions
            getMaximumActiveIsometricTorque(), and setMaximumActiveIsometricTorque().

            <b>Limitations</b>

            This rigid-tendon torque muscle has some limitations that you should
            be aware of:

            -# There are no elastic tendons. That means that the mapping between
            the mechanical work done by this torque actuator will greatly differ
            from the positive mechanical work done by a torque actuator that
            includes an elastic tendon. This difference is greatest for those
            muscles with long tendons - namely the Achilles tendon. If you are
            interested in fiber kinematics, fiber work, or metabolic energy 
            consumption you cannot use this model especially for muscles that
            have long tendons.
            -# This model formulation predicts torque well, but does a poor job
            of predicting joint stiffness. In this model stiffness is given
            by the partial derivative of torque w.r.t. joint angle. Since the
            active-torque-angle curve fits a cosine function, it is possible
            to construct a torque muscle that has a region of physically
            impossible negative stiffness. Real muscle, in constrast, always
            has a positive stiffness even on the descending limb of the
            active-torque-angle curve (see Rassier et al. for details).
            -# Muscles that cross 2 joints (e.g. the hamstrings) produce coupled
            torques at both of those joints. In this model there is no coupling
            between joints. Furthermore, because of the lack of coupling the 
            curves used here are only valid for the posture that Anderson et al. 
            used when they made their data collection. If you are interested 
            in simulating postures that are very different from those described
            in Anderson et al., then the results produced by this model should
            be treated as very rough. 
            -# Because this is a joint-torque muscle, none of the joint contact
            forces predicted will come close to matching what is produced by
            line-type muscles. If you are interested in joint-contact forces you
            cannot use this model.

            This simple model is a fast approximate means to constrain the joint
            torque developed in the lower limb to something that is physiologically
            possible at the hip, knee, and ankle (in the sagittal plane) for
            the subset of people tested. That is it.

            <b>Units</b>
            Although the figure in this description has angles in units
            of degrees, this is only to help intuitition: when using
            the model, use radians. This model uses MKS:

                -Distance: m                
                -Angles: radians
                -Angular velocity: radians/s
                -Mass: kg
                -Torque: Nm
                -Time: second
                -Power: Nm/second


             <b>References</b>
           
            -# Anderson, D. E., Madigan, M. L., & Nussbaum, M. A. (2007). 
            Maximum voluntary joint torque as a function of joint angle 
            and angular velocity: model development and application to 
            the lower limb. Journal of biomechanics, 40(14), 3105-3113.

            -# Jackson, M.I. (2010). The mechanics of the Table Contact
            Phase of Gymnastics Vaulting. Doctoral Thesis, Loughborough
            University.

            -# Kentel, B.B., King, M.A., & Mitchell, S.R. (2011).
            Evaluation of a subject-specific torque-driven computer simulation
            model of one-handed tennis backhand ground strokes. Journal of
            Applied Biomechanics, 27(4),345-354.

            -# Millard, M., Uchida, T., Seth, A., & Delp, S. L. (2013). 
            Flexing computational muscle: modeling and simulation of 
            musculotendon dynamics. Journal of biomechanical engineering, 
            135(2), 021005.

            -# Rassier, D. E., Herzog, W., Wakeling, J., & Syme, D. A. (2003).
            Stretch-induced, steady-state force enhancement in single
            skeletal muscle fibers exceeds the isometric force at optimum
            fiber length. Journal of biomechanics, 36(9), 1309-1316.

            
        */
        class Millard2016TorqueMuscle {

            public:
                /**
                Default constructor, which for the moment does nothing. 
                Calling any of the models functions after the default
                construction will result in a runtime error.
                */
                Millard2016TorqueMuscle();

                /**
                    This constructor builds an Millard2016TorqueMucle
                    given the specific coefficients from the paper. This 
                    constructor should only be used when

                    - You have coefficients for the Anderson torque model 
                    - And these coefficients are not in the built-in data sets

                    <b> Note: directions </b> 
                    This constructs a single joint-torque muscle: 
                    it can only generate torque in one direction. If you want 
                    to generate a torque in two directions, you need 2 torque 
                    muscles.


          
                    @param c1c2c3c4c5c6
                            A 6-element VectorNd that contains the parameters
                            c1,...,c6 in order. These parameters are described
                            in detail in the Anderson paper listed in the 
                            class description.

                    @param b1k1b2k2
                            A 4-element VectorNd that contains the parameters
                            b1,k1,b2,k2 in order. These parameters are described 
                            in detail in the Anderson paper listed in the 
                            class description.
                    
                    @param subjectHeightInMeters
                            This parameter is used to scale from the normalized
                            curves reported by Anderson et al. 
                            See the class description for details.

                    @param subjectMassInKg
                            This parameter is used to scale from the normalized
                            curves reported by Anderson et al. 
                            See the class description for details.

                    @param jointAngleOffsetRelativeToAnderson2007
                            Offset angle between your model's joints and the
                            reference system defined by Anderson et al. 
                            See the class description for details.

                    @param signOfJointAngleRelativeToAnderson2007
                            The sign convention that converts your model's joint
                            angles to the angles used in Anderson's model.
                            See the class description for details.

                    @param signOfJointTorque 
                            The sign that maps fiberTorque from Anderson's model
                            (which is always positive) to the correctly signed
                            joint torque which is always positive.

                    @param name
                            The name of the muscle. This is needed to do useful
                            things like provide error messages that are human
                            readable.
                    
                    @throws std::invalid_argument when
                        -# c1c2c3c4c5c6 is not 6 elements long
                        -# b1k1b2k2 is not 4 elements long
                        -# subjectHeightInMeters <= 0
                        -# subjectMassInKg <= 0
                        -# abs(signOfJointTorque)-1 > epsilon

                */
                Millard2016TorqueMuscle(                  
                    const RigidBodyDynamics::Math::VectorNd& c1c2c3c4c5c6,
                    const RigidBodyDynamics::Math::VectorNd& b1k1b2k2,
                    double subjectHeightInMeters,
                    double subjectMassInKg,
                    double jointAngleOffsetRelativeToAnderson2007,
                    double signOfJointAngleRelativeToAnderson2007,
                    double signOfJointTorque,
                    const std::string& name
                    );


                    /**

                    This is the most general and easy-to-use constructor for
                    this class as it allows you to easily access the large 
                    table of built-in torque muscle coefficients

                    <b> Note: directions </b> 
                    This constructs a single joint-torque muscle: 
                    it can only generate torque in one direction. If you want 
                    to generate a torque in two directions, you need 2 torque 
                    muscles.

                    <b> Note: signs and offsets </b> 

                    All of the angles in these models are defined anatomically.
                    You will need to set a series of variables to correctly
                    map from your model's joint coordinates and sign conventions
                    to that of the models:
                    jointAngleOffsetRelativeToDoxygenFigures,
                    signOfJointAngleRelativeToDoxygenFigures,
                    signOfJointTorqueToDoxygenFigures. Also note that due to 
                    the anatomical angle definitions some left and right handed
                    joints will require different signs. This will be true 
                    for internal/external rotation at the shoulder,
                    horizontal adduction/abduction at the shoulder, ulnar/radial
                    deviation at the wrist, pronation/supination of the wrist,
                    and others as the list of directions grows.
                  

                    @param dataSet
                           The desired source of joint torque
                           coefficients. Use the DataSet structure to choose
                           the desired data set (e.g. DataSet::Anderson2007,
                           or DataSet::Gymnast)

                    @param subjectInfo
                            A struct that contains metadata about the subject
                            which is used to scale the maximum torque of the 
                            torque muscle.

                    @param jointTorque
                           Select the joint and torque direction of interest.
                           Use the struct for each data set to choose a 
                           joint-torque-direction that is in the set (e.g.
                           Anderson2007::HipExtension, or 
                           Gymnast::ShoulderHorizontalAdduction)
            
                    @param jointAngleOffsetRelativeToDoxygenFigures
                            Offset angle between your model's joints and the
                            reference figures in class description. 
 
                    @param signOfJointAngleRelativeToDoxygenFigures
                            The sign convention that converts your model's joint
                            angles to the angles used in the reference figures.

                    @param signOfJointTorqueToDoxygenFigures
                            The sign that maps fiberTorque from Anderson's model
                            (which is always positive) to the correctly signed
                            joint torque for your model.

                    @param name
                            The name of the muscle. This is needed to do useful
                            things like provide error messages that are human
                            readable.
                    
                    @throws abort() when
                        -# The combination of dataSet, gender, joint, and jointDirection does not correspond to a valid entry
                        -# subjectHeightInMeters <= 0
                        -# subjectMassInKg <= 0
                        -# abs(signOfJointTorque)-1 > epsilon

                */
                Millard2016TorqueMuscle(
                    DataSet::item   dataSet,
                    const SubjectInformation &subjectInfo,
                    int jointTorque,
                    double  jointAngleOffsetRelativeToDoxygenFigures,
                    double  signOfJointAngleRelativeToDoxygenFigures,
                    double  signOfJointTorqueToDoxygenFigures,
                    const   std::string& name
                    );


                /**
                Calculates the joint torque developed by the 
                muscle. 

                @param jointAngle (radians)
                
                @param jointAngularVelocity (radians/sec)

                @param activation: the percentage of the muscle that is
                        turned on [0-1]. This function allows activations to be 
                        outside [0,1], because this is useful during the 
                        intermediate solutions of an optimization run. However,
                        you must ensure after the fact that your activations
                        fall within a bound of [0,1].

                @returns torque developed by the musce in (Nm).                                   
                */
                double calcJointTorque(                        
                        double jointAngle,
                        double jointAngularVelocity,
                        double activation) const;


                /**
                Calculates a large number of internal quantities of the
                torque muscle ranging from the values of the muscle's 
                components, the stiffness of the muscle, and its power output.

                @param jointAngle (radians)

                @param jointAngularVelocity (radians/sec)

                @param activation: the percentage of the muscle that is
                        turned on [0-1]. This function allows activations to be 
                        outside [0,1], because this is useful during the 
                        intermediate solutions of an optimization run. However,
                        you must ensure after the fact that your activations
                        fall within a bound of [0,1].

                @param torqueMuscleInfoStruct: A torque muscle struct
                */
                void calcTorqueMuscleInfo(
                        double jointAngle,
                        double jointAngularVelocity,
                        double activation,
                        TorqueMuscleInfo& torqueMuscleInfoStruct) const;

                /**
                    @return the maximum-active-isometric torque that this muscle 
                            can produce in Nm.
                */
                double  getMaximumActiveIsometricTorque() const; 

                /**
                    @return the passive-torque-scale that is applied to the
                            passive-torque-curve.
                */
                double  getPassiveTorqueScale() const;

                /**
                    Sets the scaling of the passive-joint-torques. By default this scale
                    is one.

                    @param passiveTorqueScale
                            The scale applied to the passive-joint-torque curve (unitless)
                */
                void  setPassiveTorqueScale(double passiveTorqueScale);


                /**
                    Sets the strength of the muscle to match a desired value.

                    @param maxIsometricTorque
                            The desired maximum-active-isometric torque of the muscle (Nm)
                            
                */
                void    setMaximumActiveIsometricTorque(
                            double maxIsometricTorque);    

                /**
                    @return the parameters c1,...,c6 that desribe the active-torque-angle
                            and torque-velocity curves of this
                            torque muscle model. See the Anderson et al. paper metioned
                            in the class description for detail.
                */
                const RigidBodyDynamics::Math::VectorNd& getParametersC1C2C3C4C5C6();

                /**
                    @return the parameters b1,k1,b2,k2 that desribe the passive-torque-angle
                            curves of this model. See the Anderson et al. paper metioned
                            in the class description for detail.
                */
                const RigidBodyDynamics::Math::VectorNd& getParametersB1K1B2K2();

                /**
                    @return the SmoothSegmentedFunction the has been fitted to Anderson et al.'s
                            passive torque angle curve.
                */
                const RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction& getActiveTorqueAngleCurve() const;
                
                /**
                    @return the SmoothSegmentedFunction the has been fitted to Anderson et al.'s
                            active torque angle curve.
                */
                const RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction& getPassiveTorqueAngleCurve() const;
                
                /**
                    @return the SmoothSegmentedFunction the has been fitted to Anderson et al.'s
                            torque velocity curve.
                */
                const RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction& getTorqueAngularVelocityCurve() const;

                /**
                  Prints 2 csv files:
                    -# 'fileName' + '_variableLengthfixedVelocity': All of the fields
                       in TorqueMuscleInfo are recorded to file as the jointAngle varies
                       but the jointAngularVelocity is zero.                        
                    -#'fileName' + '_fixedLengthVariableVelocity': All of the fields
                       in TorqueMuscleInfo are recorded to file as the jointAngle is fixed
                       but the jointAngularVelocity varies.

                        Each column has a header, so that you can tell what each piece of
                        data means.

                       @param path: the path to the destination folder. Don't put an '\' on
                                    the end.
                       @param fileNameWithoutExtension: the name of the file, but without 
                                an extension.                                    
                       @param numberOfSamplePoints: the number of sample points to use in 
                                the files.                                
                */
                void printJointTorqueProfileToFile(
                        const std::string& path,
                        const std::string& fileNameWithoutExtension,
                        int numberOfSamplePoints);

                std::string getName();
                void setName(std::string& name);

            private:
                bool muscleCurvesAreDirty;
                void updateTorqueMuscleCurves();
                TorqueMuscleInfo tmInfo;

                RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction taCurve;
                RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction tpCurve;
                RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction tvCurve;

                RigidBodyDynamics::Math::VectorNd c1c2c3c4c5c6;
                RigidBodyDynamics::Math::VectorNd b1k1b2k2;
                
                double strengthScaleFactor;
                double maxActiveIsometricTorque;
                double passiveTorqueScale;

                double signOfJointAngle;
                double signOfConcentricAnglularVelocity;
                double signOfJointTorque;
                double angleOffset;

                std::string muscleName;

                double calcFiberAngle(double jointAngle) const;
                double calcFiberAngularVelocity(double jointAngularVelocity) const;



                //const static RigidBodyDynamics::Math::MatrixNd& getAnderson2007ParameterMatrix();
                static double const Anderson2007Table3Mean[36][14];
                static double const Anderson2007Table3Std[36][14];
                static double const GymnastWholeBody[20][14];

    };



}
}
}

#endif
