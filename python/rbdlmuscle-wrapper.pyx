#cython: boundscheck=False, embedsignature=True

import numpy as np
cimport numpy as np
from enum import IntEnum
from enum import Enum
from libc.stdint cimport uintptr_t
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool


#cimport cpython.ref as cpy_ref

from crbdl cimport VectorNd  
from crbdl cimport Vector3d 
from crbdl cimport MatrixNd 

cimport crbdlmuscle


from rbdl cimport NumpyToVector3d as NumpyToVector3d
from rbdl cimport Vector3dToNumpy as Vector3dToNumpy
from rbdl cimport NumpyToVectorNd as NumpyToVectorNd
from rbdl cimport VectorNdToNumpy as VectorNdToNumpy
from rbdl cimport NumpyToMatrixNd as NumpyToMatrixNd
from rbdl cimport MatrixNdToNumpy as MatrixNdToNumpy





##############################
#
# SmoothSegmentedFunction.h
#
##############################




cdef class SmoothSegmentedFunction:
  
    cdef crbdlmuscle.SmoothSegmentedFunction *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __cinit__(self,
          np.ndarray[double, ndim=2, mode="c"] mX = None, 
          np.ndarray[double, ndim=2, mode="c"] mY = None,
          double x0 = 0, double x1 = 0, double y0 = 0, double y1 =0,
          double dydx0 = 0, 
          double dydx1 = 0, 
          string name = "", 
          uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            if mY is None:
                self.thisptr = new crbdlmuscle.SmoothSegmentedFunction()
            else:
                self.thisptr = new crbdlmuscle.SmoothSegmentedFunction(
                        NumpyToMatrixNd(mX), NumpyToMatrixNd(mY), 
                        x0, x1, y0, y1, dydx0, dydx1, name)
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.SmoothSegmentedFunction*>address

    def updSmoothSegmentedFunction( self,
             np.ndarray[double, ndim=2, mode="c"] mX, 
             np.ndarray[double, ndim=2, mode="c"] mY, 
             double x0, double x1,
             double y0, double y1,
             double dydx0, double dydx1,
             string name):
        self.thisptr.updSmoothSegmentedFunction(  
             NumpyToMatrixNd(mX), 
             NumpyToMatrixNd(mY), 
             x0, x1, y0, y1, dydx0, dydx1, name)
             
    def shift(self, double xShift, double yShift):
        self.thisptr.shift(xShift, yShift)
        
    def scale(self, double xScale, double yScale):
        self.thisptr.scale(xScale, yScale)
    
    def calcValue(self, double x):
        return self.thisptr.calcValue(x)
    
    def calcDerivative(self, double x, int order):
        return self.thisptr.calcDerivative(x, order)
    
    def getName(self):
        return self.thisptr.getName()
        
    def setName(self, string name):
        self.thisptr.setName(name)
    
    def getCurveDomain(self):
        return VectorNdToNumpy( self.thisptr.getCurveDomain())
    
    def printCurveToCSVFile(self, string path,
          string fileNameWithoutExtension,
            double domainMin,
            double domainMax):
          self.thisptr.printCurveToCSVFile(path,
                       fileNameWithoutExtension,
                       domainMin,
                       domainMax)
                      
    def calcSampledCurve(self, int maxOrder,
                double domainMin,
                double domainMax):
        return MatrixNdToNumpy( self.thisptr.calcSampledCurve(maxOrder,
                domainMin,
                domainMax) )
                
    def getXControlPoints(self, np.ndarray[double, ndim=2, mode="c"] mat):
        cdef MatrixNd cmat = MatrixNd()
        self.thisptr.getXControlPoints( cmat )
        
        np.resize(mat,(cmat.rows(),cmat.cols()))
        for i in range(cmat.rows()):
          for j in range (cmat.cols()):
            mat[i,j] = cmat.coeff(i,j)
        
    
    def getYControlPoints(self, np.ndarray[double, ndim=2, mode="c"] mat):
        cdef MatrixNd cmat = MatrixNd()
        self.thisptr.getYControlPoints( cmat )
        
        np.resize(mat,(cmat.rows(),cmat.cols()))
        for i in range(cmat.rows()):
          for j in range (cmat.cols()):
            mat[i,j] = cmat.coeff(i,j)




##############################
#
# Millard2016TorqueMuscle.h
#
##############################


class DataSet(IntEnum):
    Anderson2007 = 0
    Gymnast = 1
    Last = 2


class GenderSet(IntEnum):
    Male = 0
    Female = 1
    Last = 2
    
    
class AgeGroupSet(IntEnum):
    Young18To25 = 0
    Middle55To65 = 1
    SeniorOver65 = 2
    Last = 3
    
class JointTorqueSet(IntEnum):
    HipExtension                  = 0
    HipFlexion                    = 1
    KneeExtension                 = 2
    KneeFlexion                   = 3
    AnkleExtension                = 4
    AnkleFlexion                  = 5
    ElbowExtension                = 6
    ElbowFlexion                  = 7
    ShoulderExtension             = 8
    ShoulderFlexion               = 9
    WristExtension                = 10
    WristFlexion                  = 11
    ShoulderHorizontalAdduction   = 12
    ShoulderHorizontalAbduction   = 13
    ShoulderInternalRotation      = 14
    ShoulderExternalRotation      = 15
    WristUlnarDeviation           = 16
    WristRadialDeviation          = 17
    WristPronation                = 18
    WristSupination               = 19
    LumbarExtension               = 20
    LumbarFlexion                 = 21
    Last                          = 22
       
            

cdef class SubjectInformation:
  
    cdef crbdlmuscle.SubjectInformation *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
                        
    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdlmuscle.SubjectInformation()
            
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.SubjectInformation*>address
    
    property gender:
      def __get__(self):
        return GenderSet(self.thisptr.gender)
      def __set__ (self, value):
         if value == GenderSet.Male:
            self.thisptr.gender = crbdlmuscle.GenderSet_Male
         elif value == GenderSet.Female:
            self.thisptr.gender = crbdlmuscle.GenderSet_Female
         else:
             raise TypeError('gender must be an instance of GenderSet Enum or 0/1')
            
                
    property ageGroup:
      def __get__(self):
        return AgeGroupSet(self.thisptr.ageGroup)
      def __set__ (self, value):
         if value == AgeGroupSet.Young18To25:
             self.thisptr.ageGroup = crbdlmuscle.AgeGroupSet_Young18To25
         elif value == AgeGroupSet.Middle55To65:
             self.thisptr.ageGroup = crbdlmuscle.AgeGroupSet_Middle55To65
         elif value == AgeGroupSet.SeniorOver65:
             self.thisptr.ageGroup = crbdlmuscle.AgeGroupSet_SeniorOver65
         else:
             raise TypeError('gender must be an instance of GenderSet Enum or 0/1/2')
    
    property heightInMeters:
      def __get__ (self):
        return self.thisptr.heightInMeters
      def __set__ (self, value):
        self.thisptr.heightInMeters = value
    
    property massInKg:
      def __get__ (self):
        return self.thisptr.massInKg
      def __set__ (self, value):
        self.thisptr.massInKg = value




cdef class TorqueMuscleSummary:
  
    cdef crbdlmuscle.TorqueMuscleSummary *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
            
    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdlmuscle.TorqueMuscleSummary()
            
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.TorqueMuscleSummary*>address
    
    property activation:
        def __get__ (self):
            return self.thisptr.activation
        def __set__ (self, value):
            self.thisptr.activation = value
    
    property fiberPassiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberPassiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberPassiveTorqueAngleMultiplier = value
    
    property fiberActiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberActiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberActiveTorqueAngleMultiplier = value
    
    property fiberActiveTorqueAngularVelocityMultiplier:
        def __get__ (self):
            return self.thisptr.fiberActiveTorqueAngularVelocityMultiplier
        def __set__ (self, value):
            self.thisptr.fiberActiveTorqueAngularVelocityMultiplier = value
    
    property fiberNormalizedDampingTorque:
        def __get__ (self):
            return self.thisptr.fiberNormalizedDampingTorque
        def __set__ (self, value):
            self.thisptr.fiberNormalizedDampingTorque = value
    
    property fiberTorque:
        def __get__ (self):
            return self.thisptr.fiberTorque
        def __set__ (self, value):
            self.thisptr.fiberTorque = value
    
    property jointTorque:
        def __get__ (self):
            return self.thisptr.jointTorque
        def __set__ (self, value):
            self.thisptr.jointTorque = value



    

cdef class TorqueMuscleInfo:
    cdef crbdlmuscle.TorqueMuscleInfo *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
    
    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdlmuscle.TorqueMuscleInfo()
            
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.TorqueMuscleInfo*>address
    
    
    property jointAngle:
        def __get__ (self):
            return self.thisptr.jointAngle
        def __set__ (self, value):
            self.thisptr.jointAngle = value
    
    property jointAngularVelocity:
        def __get__ (self):
            return self.thisptr.jointAngularVelocity
        def __set__ (self, value):
            self.thisptr.jointAngularVelocity = value
    
    property fiberAngle:
        def __get__ (self):
            return self.thisptr.fiberAngle
        def __set__ (self, value):
            self.thisptr.fiberAngle = value
            
    property fiberAngularVelocity:
        def __get__ (self):
            return self.thisptr.fiberAngularVelocity
        def __set__ (self, value):
            self.thisptr.fiberAngularVelocity = value
            
    property fiberPassiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberPassiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberPassiveTorqueAngleMultiplier = value
            
    property fiberActiveTorqueAngleMultiplier:
        def __get__ (self):
            return self.thisptr.fiberActiveTorqueAngleMultiplier
        def __set__ (self, value):
            self.thisptr.fiberActiveTorqueAngleMultiplier = value
            
    property fiberActiveTorqueAngularVelocityMultiplier:
        def __get__ (self):
            return self.thisptr.fiberActiveTorqueAngularVelocityMultiplier
        def __set__ (self, value):
            self.thisptr.fiberActiveTorqueAngularVelocityMultiplier = value
            
    property activation:
        def __get__ (self):
            return self.thisptr.activation
        def __set__ (self, value):
            self.thisptr.activation = value
            
            
    property fiberActiveTorque:
        def __get__ (self):
            return self.thisptr.fiberActiveTorque
        def __set__ (self, value):
            self.thisptr.fiberActiveTorque = value
    
    property fiberPassiveTorque:
        def __get__ (self):
            return self.thisptr.fiberPassiveTorque
        def __set__ (self, value):
            self.thisptr.fiberPassiveTorque = value
    
    property fiberDampingTorque:
        def __get__ (self):
            return self.thisptr.fiberDampingTorque
        def __set__ (self, value):
            self.thisptr.fiberDampingTorque = value
    
    property fiberNormDampingTorque:
        def __get__ (self):
            return self.thisptr.fiberNormDampingTorque
        def __set__ (self, value):
            self.thisptr.fiberNormDampingTorque = value
            
    property fiberTorque:
        def __get__ (self):
            return self.thisptr.fiberTorque
        def __set__ (self, value):
            self.thisptr.fiberTorque = value
            
    property jointTorque:
        def __get__ (self):
            return self.thisptr.jointTorque
        def __set__ (self, value):
            self.thisptr.jointTorque = value
            
    property fiberStiffness:
        def __get__ (self):
            return self.thisptr.fiberStiffness
        def __set__ (self, value):
            self.thisptr.fiberStiffness = value
            
    property jointStiffness:
        def __get__ (self):
            return self.thisptr.jointStiffness
        def __set__ (self, value):
            self.thisptr.jointStiffness = value
            
            
    property fiberActivePower:
        def __get__ (self):
            return self.thisptr.fiberActivePower
        def __set__ (self, value):
            self.thisptr.fiberActivePower = value
            
    property fiberPassivePower:
        def __get__ (self):
            return self.thisptr.fiberPassivePower
        def __set__ (self, value):
            self.thisptr.fiberPassivePower = value
            
    property fiberPower:
        def __get__ (self):
            return self.thisptr.fiberPower
        def __set__ (self, value):
            self.thisptr.fiberPower = value
    
    property jointPower:
        def __get__ (self):
            return self.thisptr.jointPower
        def __set__ (self, value):
            self.thisptr.jointPower = value
            
    property DjointTorqueDactivation:
        def __get__ (self):
            return self.thisptr.DjointTorqueDactivation
        def __set__ (self, value):
            self.thisptr.DjointTorqueDactivation = value
    
    property DjointTorqueDjointAngle:
        def __get__ (self):
            return self.thisptr.DjointTorqueDjointAngle
        def __set__ (self, value):
            self.thisptr.DjointTorqueDjointAngle = value
            
    property DjointTorqueDjointAngularVelocity:
        def __get__ (self):
            return self.thisptr.DjointTorqueDjointAngularVelocity
        def __set__ (self, value):
            self.thisptr.DjointTorqueDjointAngularVelocity = value

                
   

cdef class Millard2016TorqueMuscle:
    cdef crbdlmuscle.Millard2016TorqueMuscle *thisptr
    cdef free_on_dealloc
    
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
    

    def __cinit__(self, dataSet = None, 
          SubjectInformation subjectInfo = None, 
          int jointTorque = 1, 
          double jointAngleOffsetRelativeToDoxygenFigures = 1., 
          double signOfJointAngleRelativeToDoxygenFigures = 1., 
          double signOfJointTorqueToDoxygenFigures = 1., 
          string name = "empty", 
          uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            if dataSet is None:
                self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle()
            else:
                # Type checking
                #if not isinstance(dataSet, DataSet):
                #     raise TypeError('dataSet must be an instance of DataSet Enum')
                if subjectInfo is None:
                    print "Warning: dataset, but no subject info specified, using default constructor"
                    self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle()
                else:
                    if dataSet == DataSet.Anderson2007:
                      
                        self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle(
                        crbdlmuscle.DataSet_Anderson2007, subjectInfo.thisptr[0], jointTorque, 
                        jointAngleOffsetRelativeToDoxygenFigures, 
                        signOfJointAngleRelativeToDoxygenFigures, 
                        signOfJointTorqueToDoxygenFigures, name)
                        
                    elif dataSet == DataSet.Gymnast:
                      
                        self.thisptr = new crbdlmuscle.Millard2016TorqueMuscle(
                        crbdlmuscle.DataSet_Gymnast, subjectInfo.thisptr[0], jointTorque, 
                        jointAngleOffsetRelativeToDoxygenFigures, 
                        signOfJointAngleRelativeToDoxygenFigures, 
                        signOfJointTorqueToDoxygenFigures, name)
                    else:
                        raise TypeError('dataSet must be an instance of DataSet Enum or 0/1')
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdlmuscle.Millard2016TorqueMuscle*>address
    
    def calcJointTorque( self, double jointAngle, double jointAngularVelocity, 
                         double activation):
        return self.thisptr.calcJointTorque( jointAngle, jointAngularVelocity, activation)
        
    def calcActivation(self, double jointAngle,
                                    double jointAngularVelocity,
                                    double jointTorque,
                                    TorqueMuscleSummary tms):
        self.thisptr.calcActivation( jointAngle,
                                     jointAngularVelocity,
                                     jointTorque,
                                     tms.thisptr[0])
                                     
                                     
                                                                      

    def calcMaimumActiveIsometricTorqueScalingFactor(self, double jointAngle,
                          double jointAngularVelocity,
                          double activation,
                          double jointTorque):
        return self.thisptr.calcMaximumActiveIsometricTorqueScalingFactor(
                          jointAngle,
                          jointAngularVelocity,
                          activation,
                          jointTorque)
                          
    def calcTorqueMuscleInfo(self, double jointAngle, 
                          double jointAngularVelocity,
                          double activation,
                          TorqueMuscleInfo torqueMuscleInfoStruct):
        self.thisptr.calcTorqueMuscleInfo(
                        jointAngle,
                        jointAngularVelocity,
                        activation,
                        torqueMuscleInfoStruct.thisptr[0])
                      
    def getJointTorqueSign(self):
        return self.thisptr.getJointTorqueSign() 
        
    def getJointAngleSign(self):
        return self.thisptr.getJointAngleSign() 
        
    def getJointAngleOffset(self):
        return self.thisptr.getJointAngleOffset()
    
    def getMaximumActiveIsometricTorque(self):
        return self.thisptr.getMaximumActiveIsometricTorque()
        
    def getJointAngleAtMaximumActiveIsometricTorque(self):
        return self.thisptr.getJointAngleAtMaximumActiveIsometricTorque()
    
    def getJointAngleAtOneNormalizedPassiveIsometricTorque(self):
        return self.thisptr.getJointAngleAtOneNormalizedPassiveIsometricTorque()
    
    def getMaximumConcentricJointAngularVelocity(self):
        return self.thisptr.getMaximumConcentricJointAngularVelocity() 
    
    def getPassiveTorqueScale(self):
        return self.thisptr.getPassiveTorqueScale() 
        
    def getPassiveCurveAngleOffset(self):    
        return self.thisptr.getPassiveCurveAngleOffset() 
        
    def getNormalizedDampingCoefficient(self):
        return self.thisptr.getNormalizedDampingCoefficient() 
        
    def setNormalizedDampingCoefficient(self, double beta):    
        self.thisptr.setNormalizedDampingCoefficient(beta)
        
        
    def setPassiveTorqueScale(self, double passiveTorqueScale):    
        self.thisptr.setPassiveTorqueScale(passiveTorqueScale)
        
    def setPassiveCurveAngleOffset( self,
                  double passiveCurveAngleOffsetVal):
        self.thisptr.setPassiveCurveAngleOffset(
                  passiveCurveAngleOffsetVal)
                  
    def fitPassiveTorqueScale(self, double jointAngle,
                                           double passiveTorque):          
        self.thisptr.fitPassiveTorqueScale(jointAngle,
                                           passiveTorque)
                                           
    def fitPassiveCurveAngleOffset(self, double jointAngle,
                                           double passiveTorque):                                       
        self.thisptr.fitPassiveCurveAngleOffset(jointAngle,
                                           passiveTorque)
                                           
    def setMaximumActiveIsometricTorque(self, double maxIsometricTorque):                                       
        self.thisptr.setMaximumActiveIsometricTorque(maxIsometricTorque)
        
    def setMaximumConcentricJointAngularVelocity(self, double maxAngularVelocity):    
        self.thisptr.setMaximumConcentricJointAngularVelocity(maxAngularVelocity)

    def getActiveTorqueAngleCurve(self):
      cdef crbdlmuscle.SmoothSegmentedFunction ssf = crbdlmuscle.SmoothSegmentedFunction()
      cdef VectorNd x_ = VectorNd()
      cdef MatrixNd Mx = MatrixNd()
      cdef MatrixNd My = MatrixNd()
      cdef string name_
      sff = self.thisptr.getActiveTorqueAngleCurve()
      
      x_ = sff.getCurveDomain()
      sff.getXControlPoints(Mx)
      sff.getYControlPoints(My)
      
      return SmoothSegmentedFunction( MatrixNdToNumpy(Mx), 
                                      MatrixNdToNumpy(My),
                                      x_[0], x_[1], 
                                      sff.calcValue(x_[0]),
                                      sff.calcValue(x_[1]),
                                      sff.calcDerivative(x_[0], 1),
                                      sff.calcDerivative(x_[1], 1),
                                      sff.getName())
                                      
    def getPassiveTorqueAngleCurve(self):
      cdef crbdlmuscle.SmoothSegmentedFunction ssf = crbdlmuscle.SmoothSegmentedFunction()
      cdef VectorNd x_ = VectorNd()
      cdef MatrixNd Mx = MatrixNd()
      cdef MatrixNd My = MatrixNd()
      cdef string name_
      sff = self.thisptr.getPassiveTorqueAngleCurve()
      
      x_ = sff.getCurveDomain()
      sff.getXControlPoints(Mx)
      sff.getYControlPoints(My)
      
      return SmoothSegmentedFunction( MatrixNdToNumpy(Mx), 
                                      MatrixNdToNumpy(My),
                                      x_[0], x_[1], 
                                      sff.calcValue(x_[0]),
                                      sff.calcValue(x_[1]),
                                      sff.calcDerivative(x_[0], 1),
                                      sff.calcDerivative(x_[1], 1),
                                      sff.getName())
                                
    def getTorqueAngularVelocityCurve(self):
      cdef crbdlmuscle.SmoothSegmentedFunction ssf = crbdlmuscle.SmoothSegmentedFunction()
      cdef VectorNd x_ = VectorNd()
      cdef MatrixNd Mx = MatrixNd()
      cdef MatrixNd My = MatrixNd()

      sff = self.thisptr.getTorqueAngularVelocityCurve()
      
      x_ = sff.getCurveDomain()
      sff.getXControlPoints(Mx)
      sff.getYControlPoints(My)
      
      return SmoothSegmentedFunction( MatrixNdToNumpy(Mx), 
                                      MatrixNdToNumpy(My),
                                      x_[0], x_[1], 
                                      sff.calcValue(x_[0]),
                                      sff.calcValue(x_[1]),
                                      sff.calcDerivative(x_[0], 1),
                                      sff.calcDerivative(x_[1], 1),
                                      sff.getName())
      

    def printJointTorqueProfileToFile(self, const string path,
                        const string fileNameWithoutExtension,
                        int numberOfSamplePoints):
                          
        self.thisptr.printJointTorqueProfileToFile(
                        path,
                        fileNameWithoutExtension,
                        numberOfSamplePoints)
                        
    def getName(self):
        return self.thisptr.getName()
        
    def setName(self,  string name):   
        self.thisptr.setName(name)
       
                                





       
    
    
    
