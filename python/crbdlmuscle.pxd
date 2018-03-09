#cython: boundscheck=False

from libcpp.string cimport string
from libcpp cimport bool
from libcpp.vector cimport vector

cimport cpython.ref as cpy_ref


from crbdl cimport VectorNd
from crbdl cimport Vector3d
from crbdl cimport MatrixNd


cdef extern from "rbdl/addons/geometry/SmoothSegmentedFunction.h" namespace "RigidBodyDynamics::Addons::Geometry":
      cdef cppclass SmoothSegmentedFunction:
        SmoothSegmentedFunction()
        SmoothSegmentedFunction(
               const MatrixNd& mX, 
               const MatrixNd& mY, 
               double x0, double x1,
               double y0, double y1,
               double dydx0, double dydx1,
               const string& name)
        void updSmoothSegmentedFunction(
             const MatrixNd& mX, 
             const MatrixNd& mY, 
             double x0, double x1,
             double y0, double y1,
             double dydx0, double dydx1,
             const string& name)
        void shift(double xShift, double yShift)
        void scale(double xScale, double yScale)
        double calcValue(double x) const
        double calcDerivative(double x, int order) const
        string getName() const
        void setName(const string &name)
        VectorNd getCurveDomain() const
        void printCurveToCSVFile(const string& path,
          const string& fileNameWithoutExtension,
            double domainMin,
            double domainMax) const
        MatrixNd calcSampledCurve(int maxOrder,
                double domainMin,
                double domainMax) const
        void getXControlPoints(MatrixNd& mat) const
        void getYControlPoints(MatrixNd& mat) const


cdef extern from "rbdl/addons/muscle/Millard2016TorqueMuscle.h" namespace "RigidBodyDynamics::Addons::Muscle":
      ctypedef enum DataSet_item "RigidBodyDynamics::Addons::Muscle::DataSet::item":
        DataSet_Anderson2007 "RigidBodyDynamics::Addons::Muscle::DataSet::Anderson2007"
        DataSet_Gymnast "RigidBodyDynamics::Addons::Muscle::DataSet::Gymnast"
        DataSet_Last "RigidBodyDynamics::Addons::Muscle::DataSet::Last"
      
      
      ctypedef enum GenderSet_item "RigidBodyDynamics::Addons::Muscle::GenderSet::item":
        GenderSet_Male "RigidBodyDynamics::Addons::Muscle::GenderSet::Male"
        GenderSet_Female "RigidBodyDynamics::Addons::Muscle::GenderSet::Female"
        GenderSet_Last "RigidBodyDynamics::Addons::Muscle::GenderSet::Last"
        
      ctypedef enum AgeGroupSet_item "RigidBodyDynamics::Addons::Muscle::AgeGroupSet::item":
        AgeGroupSet_Young18To25 "RigidBodyDynamics::Addons::Muscle::AgeGroupSet::Young18To25"
        AgeGroupSet_Middle55To65 "RigidBodyDynamics::Addons::Muscle::AgeGroupSet::Middle55To65"
        AgeGroupSet_SeniorOver65 "RigidBodyDynamics::Addons::Muscle::AgeGroupSet::SeniorOver65"
        AgeGroupSet_Last "RigidBodyDynamics::Addons::Muscle::AgeGroupSet::Last"


      cdef cppclass SubjectInformation:
          GenderSet_item gender;
          AgeGroupSet_item ageGroup;
          double heightInMeters
          double massInKg
   
      cdef cppclass TorqueMuscleSummary:
          double activation
          double fiberPassiveTorqueAngleMultiplier
          double fiberActiveTorqueAngleMultiplier
          double fiberActiveTorqueAngularVelocityMultiplier
          double fiberNormalizedDampingTorque
          double fiberTorque
          double jointTorque
          
          TorqueMuscleSummary()

      
      cdef cppclass TorqueMuscleInfo:
        double jointAngle
        double jointAngularVelocity
        double fiberAngle
        double fiberAngularVelocity
        double fiberPassiveTorqueAngleMultiplier
        double fiberActiveTorqueAngleMultiplier
        double fiberActiveTorqueAngularVelocityMultiplier
        double activation
        double fiberActiveTorque
        double fiberPassiveTorque
        double fiberDampingTorque
        double fiberNormDampingTorque
        double fiberTorque
        double jointTorque
        double fiberStiffness
        double jointStiffness
        double fiberActivePower
        double fiberPassivePower
        double fiberPower
        double jointPower
        double DjointTorqueDactivation
        double DjointTorqueDjointAngle
        double DjointTorqueDjointAngularVelocity
        
        TorqueMuscleInfo()
 

      
      cdef cppclass Millard2016TorqueMuscle:
        Millard2016TorqueMuscle()
        Millard2016TorqueMuscle(
                    DataSet_item   dataSet,
                    const SubjectInformation &subjectInfo,
                    int jointTorque,
                    double  jointAngleOffsetRelativeToDoxygenFigures,
                    double  signOfJointAngleRelativeToDoxygenFigures,
                    double  signOfJointTorqueToDoxygenFigures,
                    const   string& name
                    )
        double calcJointTorque(
                        double jointAngle,
                        double jointAngularVelocity,
                        double activation) const
        void calcActivation(double jointAngle,
                                    double jointAngularVelocity,
                                    double jointTorque,
                                    TorqueMuscleSummary& tms) const
        void calcActivation(double jointAngle,
                                   double jointAngularVelocity,
                                   double jointTorque,
                                   TorqueMuscleSummary &tms,
                                   double taLambdaIn,
                                   double tvLambdaIn,
                                   double tpLambdaIn,
                                   double maxActiveIsometricTorqueIn) const
        double calcMaximumActiveIsometricTorqueScalingFactor(
                          double jointAngle,
                          double jointAngularVelocity,
                          double activation,
                          double jointTorque) const
        void calcTorqueMuscleInfo(
                        double jointAngle,
                        double jointAngularVelocity,
                        double activation,
                        TorqueMuscleInfo& torqueMuscleInfoStruct) const
        double getJointTorqueSign() const
        double getJointAngleSign() const
        double getJointAngleOffset() const
        double getMaximumActiveIsometricTorque() const
        double getJointAngleAtMaximumActiveIsometricTorque() const
        double getJointAngleAtOneNormalizedPassiveIsometricTorque() const
        double getMaximumConcentricJointAngularVelocity() const
        double getPassiveTorqueScale() const
        double getPassiveCurveAngleOffset() const
        double getNormalizedDampingCoefficient() const
        void setNormalizedDampingCoefficient(double beta)
        void setPassiveTorqueScale(double passiveTorqueScale)
        void setPassiveCurveAngleOffset(
                  double passiveCurveAngleOffsetVal)
        void fitPassiveTorqueScale(double jointAngle,
                                           double passiveTorque)
        void fitPassiveCurveAngleOffset(double jointAngle,
                                           double passiveTorque)
        void setMaximumActiveIsometricTorque(double maxIsometricTorque)
        void setMaximumConcentricJointAngularVelocity(double maxAngularVelocity)
        const SmoothSegmentedFunction& getActiveTorqueAngleCurve() const
        const SmoothSegmentedFunction& getPassiveTorqueAngleCurve() const
        const SmoothSegmentedFunction& getTorqueAngularVelocityCurve() const
        void printJointTorqueProfileToFile(
                        const string& path,
                        const string& fileNameWithoutExtension,
                        int numberOfSamplePoints)
        string getName();
        void setName(string& name)
        

        
