# WARNING! 
#
# This file was automatically created from rbdl-wrapper.pyx using wrappergen.py.
# Do not modify this file directly. Edit original source instead!!

#cython: boundscheck=False, embedsignature=True

import numpy as np
cimport numpy as np
from libc.stdint cimport uintptr_t
from libcpp.string cimport string

cimport crbdl

##############################
#
# Linear Algebra Types
#
##############################

cdef class Vector3d:
    cdef crbdl.Vector3d *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0, pyvalues=None):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.Vector3d()

            if pyvalues != None:
                for i in range (3):
                    self.thisptr.data()[i] = pyvalues[i]
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.Vector3d*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "Vector3d [{:3.4f}, {:3.4f}, {:3.4f}]".format (
                self.thisptr.data()[0], self.thisptr.data()[1], self.thisptr.data()[2])

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [self.thisptr.data()[i] for i in xrange(*key.indices(len(self)))]
        else:
            return self.thisptr.data()[key]

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                self.thisptr.data()[i] = value[src_index]
                src_index = src_index + 1
        else:
            self.thisptr.data()[key] = value

    def __len__ (self):
        return 3

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return Vector3d (address)

    @classmethod
    def fromPythonArray (cls, python_values):
        return Vector3d (0, python_values)

cdef class Matrix3d:
    cdef crbdl.Matrix3d *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0, pyvalues=None):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.Matrix3d()

            if pyvalues != None:
                for i in range (3):
                    for j in range (3):
                        (&(self.thisptr.coeff(i,j)))[0] = pyvalues[i,j]
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.Matrix3d*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "Matrix3d [{:3.4f}, {:3.4f}, {:3.4f}]".format (
                self.thisptr.data()[0], self.thisptr.data()[1], self.thisptr.data()[2])

    def __getitem__(self, key):
        return self.thisptr.data()[key]

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                self.thisptr.data()[i] = value[src_index]
                src_index = src_index + 1
        else:
            self.thisptr.data()[key] = value

    def __len__ (self):
        return 3

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return Matrix3d (address)

    @classmethod
    def fromPythonArray (cls, python_values):
        return Matrix3d (0, python_values)


cdef class VectorNd:
    cdef crbdl.VectorNd *thisptr
    cdef free_on_dealloc

    def __cinit__(self, ndim, uintptr_t address=0, pyvalues=None):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.VectorNd(ndim)

            if pyvalues != None:
                for i in range (ndim):
                    self.thisptr.data()[i] = pyvalues[i]
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.VectorNd*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [self.thisptr.data()[i] for i in xrange(*key.indices(len(self)))]
        else:
            return self.thisptr.data()[key]

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                self.thisptr.data()[i] = value[src_index]
                src_index = src_index + 1
        else:
            self.thisptr.data()[key] = value

    def __len__ (self):
        return self.thisptr.rows()

    def toNumpy (self):
        result = np.ndarray (self.thisptr.rows())
        for i in range (0, self.thisptr.rows()):
            result[i] = self.thisptr[0][i]
        return result

    # Constructors
    @classmethod
    def fromPythonArray (cls, python_values):
        return VectorNd (len(python_values), 0, python_values)

    @classmethod
    def fromPointer(cls, uintptr_t address):
        cdef crbdl.VectorNd* vector_ptr = <crbdl.VectorNd*> address
        return VectorNd (vector_ptr.rows(), <uintptr_t> address)

cdef class Quaternion:
    cdef crbdl.Quaternion *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0, pyvalues=None, pymatvalues=None):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.Quaternion()

            if pyvalues != None:
                for i in range (4):
                    self.thisptr.data()[i] = pyvalues[i]
            elif pymatvalues != None:
                mat = Matrix3d()
                for i in range (3):
                    for j in range (3):
                        (&(mat.thisptr.coeff(i,j)))[0] = pymatvalues[i,j]
                self.thisptr[0] = crbdl.fromMatrix (mat.thisptr[0])
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.Quaternion*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "Quaternion [{:3.4f}, {:3.4f}, {:3.4f}, {:3.4}]".format (
                self.thisptr.data()[0], self.thisptr.data()[1],
                self.thisptr.data()[2], self.thisptr.data()[3])

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [self.thisptr.data()[i] for i in xrange(*key.indices(len(self)))]
        else:
            return self.thisptr.data()[key]

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                self.thisptr.data()[i] = value[src_index]
                src_index = src_index + 1
        else:
            self.thisptr.data()[key] = value

    def __len__ (self):
        return 4

    def toMatrix(self):
        cdef crbdl.Matrix3d mat
        mat = self.thisptr.toMatrix()
        result = np.array ([3,3])
        for i in range (3):
            for j in range (3):
                result[i,j] = mat.coeff(i,j)

        return result

    def toNumpy(self):
        result = np.ndarray (self.thisptr.rows())
        for i in range (0, self.thisptr.rows()):
            result[i] = self.thisptr[0][i]
        return result

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return Quaternion (address)

    @classmethod
    def fromPythonArray (cls, python_values):
        return Quaternion (0, python_values)

    @classmethod
    def fromPythonMatrix (cls, python_matrix_values):
        return Quaternion (0, None, python_matrix_values)

cdef class SpatialVector:
    cdef crbdl.SpatialVector *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.SpatialVector()
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.SpatialVector*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "SpatialVector [{:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}]".format (
                self.thisptr.data()[0], self.thisptr.data()[1], self.thisptr.data()[2],
                self.thisptr.data()[3], self.thisptr.data()[4], self.thisptr.data()[5])

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [self.thisptr.data()[i] for i in xrange(*key.indices(len(self)))]
        else:
            return self.thisptr.data()[key]

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                self.thisptr.data()[i] = value[src_index]
                src_index = src_index + 1
        else:
            self.thisptr.data()[key] = value

    def __len__ (self):
        return 6

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return SpatialVector (address)

cdef class SpatialMatrix:
    cdef crbdl.SpatialMatrix *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.SpatialMatrix()
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.SpatialMatrix*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "SpatialMatrix [{:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}, {:3.4f}]".format (
                self.thisptr.data()[0], self.thisptr.data()[1], self.thisptr.data()[2],
                self.thisptr.data()[3], self.thisptr.data()[4], self.thisptr.data()[5])

    def __getitem__(self, key):
        return self.thisptr.data()[key]

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                self.thisptr.data()[i] = value[src_index]
                src_index = src_index + 1
        else:
            self.thisptr.data()[key] = value

    def __len__ (self):
        return 6

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return SpatialMatrix (address)

##############################
#
# Conversion Numpy <-> Eigen
#
##############################

# Vector3d
cdef crbdl.Vector3d NumpyToVector3d (np.ndarray[double, ndim=1, mode="c"] x):
    cdef crbdl.Vector3d cx = crbdl.Vector3d()
    for i in range (3):
        cx[i] = x[i]

    return cx

cdef np.ndarray Vector3dToNumpy (crbdl.Vector3d cx):
    result = np.ndarray ((cx.rows()))
    for i in range (cx.rows()):
        result[i] = cx[i]

    return result

# VectorNd
cdef crbdl.VectorNd NumpyToVectorNd (np.ndarray[double, ndim=1, mode="c"] x):
    cdef crbdl.VectorNd cx = crbdl.VectorNd(x.shape[0])
    for i in range (x.shape[0]):
        cx[i] = x[i]

    return cx

cdef np.ndarray VectorNdToNumpy (crbdl.VectorNd cx):
    result = np.ndarray ((cx.rows()))
    for i in range (cx.rows()):
        result[i] = cx[i]

    return result

# MatrixNd
cdef crbdl.MatrixNd NumpyToMatrixNd (np.ndarray[double, ndim=2, mode="c"] M):
    cdef crbdl.MatrixNd cM = crbdl.MatrixNd(M.shape[0], M.shape[1])
    for i in range (M.shape[0]):
        for j in range (M.shape[1]):
            (&(cM.coeff(i,j)))[0] = M[i,j]

    return cM 

cdef np.ndarray MatrixNdToNumpy (crbdl.MatrixNd cM):
    result = np.ndarray ([cM.rows(), cM.cols()])
    for i in range (cM.rows()):
        for j in range (cM.cols()):
            result[i,j] = cM.coeff(i,j)

    return result

# SpatialVector
cdef np.ndarray SpatialVectorToNumpy (crbdl.SpatialVector cx):
    result = np.ndarray ((cx.rows()))
    for i in range (cx.rows()):
        result[i] = cx[i]

    return result

cdef crbdl.Quaternion NumpyToQuaternion (np.ndarray[double, ndim=1, mode="c"] x):
    cdef crbdl.Quaternion cx = crbdl.Quaternion()
    for i in range (3):
        cx[i] = x[i]

    return cx

cdef np.ndarray QuaternionToNumpy (crbdl.Quaternion cx):
    result = np.ndarray ((cx.rows()))
    for i in range (cx.rows()):
        result[i] = cx[i]

    return result

##############################
#
# Spatial Algebra Types
#
##############################

cdef class SpatialTransform:
    cdef crbdl.SpatialTransform *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0):
         if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.SpatialTransform()
         else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.SpatialTransform*>address
 
    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "SpatialTransform E = [ [{:3.4f}, {:3.4f}, {:3.4f}], [{:3.4f}, {:3.4f}, {:3.4f}], [{:3.4f}, {:3.4f}, {:3.4f}] ], r = [{:3.4f}, {:3.4f}, {:3.4f}]".format (
                self.thisptr.E.coeff(0,0), self.thisptr.E.coeff(0,1), self.thisptr.E.coeff(0,2), 
                self.thisptr.E.coeff(1,0), self.thisptr.E.coeff(1,1), self.thisptr.E.coeff(1,2), 
                self.thisptr.E.coeff(2,0), self.thisptr.E.coeff(2,1), self.thisptr.E.coeff(2,2), 
                self.thisptr.r[0], self.thisptr.r[1], self.thisptr.r[2])

    property E:
        """ Rotational part of the SpatialTransform. """
        def __get__ (self):
            result = np.ndarray ((3,3))
            for i in range (3):
                for j in range (3):
                    result[i,j] = self.thisptr.E.coeff(i,j)

            return result

        def __set__ (self, value):
            for i in range (3):
                for j in range (3):
                    (&(self.thisptr.E.coeff(i,j)))[0] = value[i,j]

    property r:
        """ Translational part of the SpatialTransform. """
        def __get__ (self):
            result = np.ndarray ((3))
            for i in range (3):
                result[i] = self.thisptr.r[i]

            return result

        def __set__ (self, value):
            for i in range (3):
                (&(self.thisptr.r[i]))[0] = value[i]

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return SpatialTransform (address)

cdef class SpatialRigidBodyInertia:
    cdef crbdl.SpatialRigidBodyInertia *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.SpatialRigidBodyInertia()
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.SpatialRigidBodyInertia*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "rbdl.SpatialRigidBodyInertia (0x{:0x})".format(<uintptr_t><void *> self.thisptr)

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return SpatialRigidBodyInertia (address)

    property m:
        def __get__ (self):
            return self.thisptr.m

        def __set__ (self, value):
            self.thisptr.m = value

    property h:
        """ Translational part of the SpatialRigidBodyInertia. """
        def __get__ (self):
            result = np.ndarray ((3))
            for i in range (3):
                result[i] = self.thisptr.h[i]

            return result

        def __set__ (self, value):
            for i in range (3):
                (&(self.thisptr.h[i]))[0] = value[i]

    property Ixx:
        def __get__ (self):
            return self.thisptr.Ixx

        def __set__ (self, value):
            self.thisptr.Ixx = value

    property Iyx:
        def __get__ (self):
            return self.thisptr.Iyx

        def __set__ (self, value):
            self.thisptr.Iyx = value

    property Iyy:
        def __get__ (self):
            return self.thisptr.Iyy

        def __set__ (self, value):
            self.thisptr.Iyy = value

    property Izx:
        def __get__ (self):
            return self.thisptr.Izx

        def __set__ (self, value):
            self.thisptr.Izx = value

    property Izy:
        def __get__ (self):
            return self.thisptr.Izy

        def __set__ (self, value):
            self.thisptr.Izy = value

    property Izz:
        def __get__ (self):
            return self.thisptr.Izz

        def __set__ (self, value):
            self.thisptr.Izz = value

##############################
#
# Rigid Multibody Types
#
##############################

cdef class Body:
    cdef crbdl.Body *thisptr
    cdef free_on_dealloc

    def __cinit__(self, **kwargs):
        cdef double c_mass
        cdef crbdl.Vector3d c_com
        cdef crbdl.Matrix3d c_inertia
        cdef uintptr_t address=0

        if "address" in kwargs.keys():
            address=kwargs["address"]
        mass = None
        if "mass" in kwargs.keys():
            mass=kwargs["mass"]
        com = None
        if "com" in kwargs.keys():
            com=kwargs["com"]
        inertia = None
        if "inertia" in kwargs.keys():
            inertia=kwargs["inertia"]

        if address == 0:
            self.free_on_dealloc = True
            if (mass != None) and (com != None) and (inertia != None):
                c_mass = mass

                for i in range (3):
                    c_com[i] = com[i]

                for i in range (3):
                    for j in range (3):
                        (&(c_inertia.coeff(i,j)))[0] = inertia[i,j]

                self.thisptr = new crbdl.Body(c_mass, c_com, c_inertia)
            else:
                self.thisptr = new crbdl.Body()
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.Body*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "rbdl.Body (0x{:0x})".format(<uintptr_t><void *> self.thisptr)

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return Body (address=address)

    @classmethod
    def fromMassComInertia(cls, double mass, 
            np.ndarray[double, ndim=1] com,
            np.ndarray[double, ndim=2] inertia):

        return Body (address=0, mass=mass, com=com, inertia=inertia)

    # Properties
    property mMass:
        def __get__ (self):
            return self.thisptr.mMass

        def __set__ (self, value):
            self.thisptr.mMass = value

    property mCenterOfMass:
        def __get__ (self):
            result = np.ndarray ((3))
            for i in range (3):
                result[i] = self.thisptr.mCenterOfMass[i]

            return result

        def __set__ (self, value):
            for i in range (3):
                (&(self.thisptr.mCenterOfMass[i]))[0] = value[i]

    property mInertia:
        def __get__ (self):
            result = np.ndarray ((3,3))
            for i in range (3):
                for j in range (3):
                    result[i,j] = self.thisptr.mInertia.coeff(i,j)

            return result

        def __set__ (self, value):
            for i in range (3):
                for j in range (3):
                    (&(self.thisptr.mInertia.coeff(i,j)))[0] = value[i,j]

    property mIsVirtual:
        def __get__ (self):
            return self.thisptr.mIsVirtual

        def __set__ (self, value):
            self.thisptr.mIsVirtual = value

cdef class FixedBody:
    cdef crbdl.FixedBody *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.FixedBody()
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.FixedBody*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "rbdl.FixedBody (0x{:0x})".format(<uintptr_t><void *> self.thisptr)

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return FixedBody (address)

    # Properties
    property mMass:
        def __get__ (self):
            return self.thisptr.mMass

        def __set__ (self, value):
            self.thisptr.mMass = value

    property mCenterOfMass:
        def __get__ (self):
            result = np.ndarray ((3))
            for i in range (3):
                result[i] = self.thisptr.mCenterOfMass[i]

            return result

        def __set__ (self, value):
            for i in range (3):
                (&(self.thisptr.mCenterOfMass[i]))[0] = value[i]

    property mInertia:
        def __get__ (self):
            result = np.ndarray ((3,3))
            for i in range (3):
                for j in range (3):
                    result[i,j] = self.thisptr.mInertia.coeff(i,j)

            return result

        def __set__ (self, value):
            for i in range (3):
                for j in range (3):
                    (&(self.thisptr.mInertia.coeff(i,j)))[0] = value[i,j]

cdef enum JointType:
    JointTypeUndefined = 0
    JointTypeRevolute
    JointTypePrismatic
    JointTypeRevoluteX
    JointTypeRevoluteY
    JointTypeRevoluteZ
    JointTypeSpherical
    JointTypeEulerZYX
    JointTypeEulerXYZ
    JointTypeEulerYXZ
    JointTypeTranslationXYZ
    JointTypeFloatingBase
    JointTypeFixed
    JointType1DoF
    JointType2DoF
    JointType3DoF
    JointType4DoF
    JointType5DoF
    JointType6DoF
    JointTypeCustom

cdef class Joint:
    cdef crbdl.Joint *thisptr
    cdef free_on_dealloc

    joint_type_map = {
            JointTypeUndefined: "JointTypeUndefined",
            JointTypeRevolute: "JointTypeRevolute",
            JointTypePrismatic: "JointTypePrismatic",
            JointTypeRevoluteX: "JointTypeRevoluteX",
            JointTypeRevoluteY: "JointTypeRevoluteY",
            JointTypeRevoluteZ: "JointTypeRevoluteZ",
            JointTypeSpherical: "JointTypeSpherical",
            JointTypeEulerZYX: "JointTypeEulerZYX",
            JointTypeEulerXYZ: "JointTypeEulerXYZ",
            JointTypeEulerYXZ: "JointTypeEulerYXZ",
            JointTypeTranslationXYZ: "JointTypeTranslationXYZ",
            JointTypeFloatingBase: "JointTypeFloatingBase",
            JointTypeFixed: "JointTypeFixed",
            JointType1DoF: "JointType1DoF",
            JointType2DoF: "JointType2DoF",
            JointType3DoF: "JointType3DoF",
            JointType4DoF: "JointType4DoF",
            JointType5DoF: "JointType5DoF",
            JointType6DoF: "JointType6DoF",
            JointTypeCustom: "JointTypeCustom",
            }

    def _joint_type_from_str (self, joint_type_str):
        if joint_type_str not in self.joint_type_map.values():
            raise ValueError("Invalid JointType '" + str(joint_type_str) + "'!")
        else:
            for joint_type, joint_str in self.joint_type_map.iteritems():
                if joint_str == joint_type_str:
                    return joint_type

    def __cinit__(self, uintptr_t address=0, joint_type=-1):
        if address == 0:
            self.free_on_dealloc = True
            if joint_type == -1:
                self.thisptr = new crbdl.Joint()
            else:
                self.thisptr = new crbdl.Joint(self._joint_type_from_str(joint_type))
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.Joint*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        joint_type_str = "JointTypeUndefined"

        if self.thisptr.mJointType in self.joint_type_map.keys():
            joint_type_str = self.joint_type_map[self.thisptr.mJointType]

        return "rbdl.Joint (0x{:0x}), JointType: {:s}".format(<uintptr_t><void *> self.thisptr, joint_type_str)

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return Joint (address)

    @classmethod
    def fromJointType(cls, joint_type):
        return Joint (0, joint_type)

    @classmethod
    def fromJointAxes(cls, axes):
        assert (len(axes) > 0)
        assert (len(axes[0]) == 6)
        axes_count = len(axes)
        joint_type = JointType1DoF + axes_count - 1

        result = Joint (0, cls.joint_type_map[joint_type])

        for i in range (axes_count):
            result.setJointAxis(i, axes[i])

        return result

    property mDoFCount:
        def __get__ (self):
            return self.thisptr.mDoFCount

        def __set__ (self, value):
            self.thisptr.mDoFCount = value

    property mJointType:
        def __get__ (self):
            return self.joint_type_map[self.thisptr.mJointType]

    property q_index:
        def __get__ (self):
            return self.thisptr.q_index

    def getJointAxis (self, index):
        assert index >= 0 and index < self.thisptr.mDoFCount, "Invalid joint axis index!"
        return SpatialVectorToNumpy (self.thisptr.mJointAxes[index])

    def setJointAxis (self, index, value):
        assert index >= 0 and index < self.thisptr.mDoFCount, "Invalid joint axis index!"
        for i in range (6):
            (&(self.thisptr.mJointAxes[index][i]))[0] = value[i]
            self.thisptr.mJointAxes[index][i]

cdef class Model

cdef class _Model_v_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.v[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.v[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.v[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.v[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.v.size()

cdef class _Model_a_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.a[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.a[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.a[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.a[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.a.size()

cdef class _Model_mJoints_Joint_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [Joint.fromPointer (<uintptr_t> &(self.parent.mJoints[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return Joint.fromPointer (<uintptr_t> &(self.parent.mJoints[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], Joint), "Invalid type! Expected Joint, but got " + str(type(value[src_index])) + "."
                self.parent.mJoints[i] = (<Joint> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, Joint), "Invalid type! Expected Joint, but got " + str(type(value)) + "."
            self.parent.mJoints[key] = (<Joint> value).thisptr[0]

    def __len__(self):
        return self.parent.mJoints.size()

cdef class _Model_S_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.S[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.S[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.S[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.S[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.S.size()

cdef class _Model_X_J_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_J[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_J[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value[src_index])) + "."
                self.parent.X_J[i] = (<SpatialTransform> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value)) + "."
            self.parent.X_J[key] = (<SpatialTransform> value).thisptr[0]

    def __len__(self):
        return self.parent.X_J.size()

cdef class _Model_v_J_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.v_J[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.v_J[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.v_J[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.v_J[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.v_J.size()

cdef class _Model_c_J_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.c_J[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.c_J[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.c_J[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.c_J[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.c_J.size()

cdef class _Model_X_T_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_T[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_T[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value[src_index])) + "."
                self.parent.X_T[i] = (<SpatialTransform> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value)) + "."
            self.parent.X_T[key] = (<SpatialTransform> value).thisptr[0]

    def __len__(self):
        return self.parent.X_T.size()

cdef class _Model_c_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.c[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.c[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.c[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.c[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.c.size()

cdef class _Model_IA_SpatialMatrix_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialMatrix.fromPointer (<uintptr_t> &(self.parent.IA[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialMatrix.fromPointer (<uintptr_t> &(self.parent.IA[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialMatrix), "Invalid type! Expected SpatialMatrix, but got " + str(type(value[src_index])) + "."
                self.parent.IA[i] = (<SpatialMatrix> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialMatrix), "Invalid type! Expected SpatialMatrix, but got " + str(type(value)) + "."
            self.parent.IA[key] = (<SpatialMatrix> value).thisptr[0]

    def __len__(self):
        return self.parent.IA.size()

cdef class _Model_pA_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.pA[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.pA[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.pA[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.pA[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.pA.size()

cdef class _Model_U_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.U[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.U[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.U[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.U[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.U.size()

cdef class _Model_f_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.f[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.f[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.f[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.f[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.f.size()

cdef class _Model_I_SpatialRigidBodyInertia_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialRigidBodyInertia.fromPointer (<uintptr_t> &(self.parent.I[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialRigidBodyInertia.fromPointer (<uintptr_t> &(self.parent.I[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialRigidBodyInertia), "Invalid type! Expected SpatialRigidBodyInertia, but got " + str(type(value[src_index])) + "."
                self.parent.I[i] = (<SpatialRigidBodyInertia> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialRigidBodyInertia), "Invalid type! Expected SpatialRigidBodyInertia, but got " + str(type(value)) + "."
            self.parent.I[key] = (<SpatialRigidBodyInertia> value).thisptr[0]

    def __len__(self):
        return self.parent.I.size()

cdef class _Model_Ic_SpatialRigidBodyInertia_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialRigidBodyInertia.fromPointer (<uintptr_t> &(self.parent.Ic[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialRigidBodyInertia.fromPointer (<uintptr_t> &(self.parent.Ic[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialRigidBodyInertia), "Invalid type! Expected SpatialRigidBodyInertia, but got " + str(type(value[src_index])) + "."
                self.parent.Ic[i] = (<SpatialRigidBodyInertia> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialRigidBodyInertia), "Invalid type! Expected SpatialRigidBodyInertia, but got " + str(type(value)) + "."
            self.parent.Ic[key] = (<SpatialRigidBodyInertia> value).thisptr[0]

    def __len__(self):
        return self.parent.Ic.size()

cdef class _Model_hc_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialVector.fromPointer (<uintptr_t> &(self.parent.hc[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialVector.fromPointer (<uintptr_t> &(self.parent.hc[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value[src_index])) + "."
                self.parent.hc[i] = (<SpatialVector> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialVector), "Invalid type! Expected SpatialVector, but got " + str(type(value)) + "."
            self.parent.hc[key] = (<SpatialVector> value).thisptr[0]

    def __len__(self):
        return self.parent.hc.size()

cdef class _Model_X_lambda_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_lambda[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_lambda[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value[src_index])) + "."
                self.parent.X_lambda[i] = (<SpatialTransform> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value)) + "."
            self.parent.X_lambda[key] = (<SpatialTransform> value).thisptr[0]

    def __len__(self):
        return self.parent.X_lambda.size()

cdef class _Model_X_base_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_base[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_base[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value[src_index])) + "."
                self.parent.X_base[i] = (<SpatialTransform> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, SpatialTransform), "Invalid type! Expected SpatialTransform, but got " + str(type(value)) + "."
            self.parent.X_base[key] = (<SpatialTransform> value).thisptr[0]

    def __len__(self):
        return self.parent.X_base.size()

cdef class _Model_mFixedBodies_FixedBody_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [FixedBody.fromPointer (<uintptr_t> &(self.parent.mFixedBodies[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return FixedBody.fromPointer (<uintptr_t> &(self.parent.mFixedBodies[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], FixedBody), "Invalid type! Expected FixedBody, but got " + str(type(value[src_index])) + "."
                self.parent.mFixedBodies[i] = (<FixedBody> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, FixedBody), "Invalid type! Expected FixedBody, but got " + str(type(value)) + "."
            self.parent.mFixedBodies[key] = (<FixedBody> value).thisptr[0]

    def __len__(self):
        return self.parent.mFixedBodies.size()

cdef class _Model_mBodies_Body_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [Body.fromPointer (<uintptr_t> &(self.parent.mBodies[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return Body.fromPointer (<uintptr_t> &(self.parent.mBodies[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], Body), "Invalid type! Expected Body, but got " + str(type(value[src_index])) + "."
                self.parent.mBodies[i] = (<Body> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, Body), "Invalid type! Expected Body, but got " + str(type(value)) + "."
            self.parent.mBodies[key] = (<Body> value).thisptr[0]

    def __len__(self):
        return self.parent.mBodies.size()


cdef class Model:
    cdef crbdl.Model *thisptr
    cdef _Model_v_SpatialVector_VectorWrapper v
    cdef _Model_a_SpatialVector_VectorWrapper a
    cdef _Model_mJoints_Joint_VectorWrapper mJoints
    cdef _Model_S_SpatialVector_VectorWrapper S
    cdef _Model_X_J_SpatialTransform_VectorWrapper X_J
    cdef _Model_v_J_SpatialVector_VectorWrapper v_J
    cdef _Model_c_J_SpatialVector_VectorWrapper c_J
    cdef _Model_X_T_SpatialTransform_VectorWrapper X_T
    cdef _Model_c_SpatialVector_VectorWrapper c
    cdef _Model_IA_SpatialMatrix_VectorWrapper IA
    cdef _Model_pA_SpatialVector_VectorWrapper pA
    cdef _Model_U_SpatialVector_VectorWrapper U
    cdef _Model_f_SpatialVector_VectorWrapper f
    cdef _Model_I_SpatialRigidBodyInertia_VectorWrapper I
    cdef _Model_Ic_SpatialRigidBodyInertia_VectorWrapper Ic
    cdef _Model_hc_SpatialVector_VectorWrapper hc
    cdef _Model_X_lambda_SpatialTransform_VectorWrapper X_lambda
    cdef _Model_X_base_SpatialTransform_VectorWrapper X_base
    cdef _Model_mFixedBodies_FixedBody_VectorWrapper mFixedBodies
    cdef _Model_mBodies_Body_VectorWrapper mBodies

    def __cinit__(self):
        self.thisptr = new crbdl.Model()
        self.v = _Model_v_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.a = _Model_a_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.mJoints = _Model_mJoints_Joint_VectorWrapper (<uintptr_t> self.thisptr)
        self.S = _Model_S_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.X_J = _Model_X_J_SpatialTransform_VectorWrapper (<uintptr_t> self.thisptr)
        self.v_J = _Model_v_J_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.c_J = _Model_c_J_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.X_T = _Model_X_T_SpatialTransform_VectorWrapper (<uintptr_t> self.thisptr)
        self.c = _Model_c_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.IA = _Model_IA_SpatialMatrix_VectorWrapper (<uintptr_t> self.thisptr)
        self.pA = _Model_pA_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.U = _Model_U_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.f = _Model_f_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.I = _Model_I_SpatialRigidBodyInertia_VectorWrapper (<uintptr_t> self.thisptr)
        self.Ic = _Model_Ic_SpatialRigidBodyInertia_VectorWrapper (<uintptr_t> self.thisptr)
        self.hc = _Model_hc_SpatialVector_VectorWrapper (<uintptr_t> self.thisptr)
        self.X_lambda = _Model_X_lambda_SpatialTransform_VectorWrapper (<uintptr_t> self.thisptr)
        self.X_base = _Model_X_base_SpatialTransform_VectorWrapper (<uintptr_t> self.thisptr)
        self.mFixedBodies = _Model_mFixedBodies_FixedBody_VectorWrapper (<uintptr_t> self.thisptr)
        self.mBodies = _Model_mBodies_Body_VectorWrapper (<uintptr_t> self.thisptr)

    def __dealloc__(self):
        del self.thisptr

    def __repr__(self):
        return "rbdl.Model (0x{:0x})".format(<uintptr_t><void *> self.thisptr)

    def AddBody (self, 
            parent_id,
            SpatialTransform joint_frame not None,
            Joint joint not None,
            Body body not None,
            string body_name = ""):
        return self.thisptr.AddBody (
                parent_id,
                joint_frame.thisptr[0], 
                joint.thisptr[0],
                body.thisptr[0],
                body_name
                )

    def AppendBody (self, 
            SpatialTransform joint_frame not None,
            Joint joint not None,
            Body body not None,
            string body_name = ""):
        return self.thisptr.AppendBody (
                joint_frame.thisptr[0], 
                joint.thisptr[0],
                body.thisptr[0],
                body_name
                )

    def SetQuaternion (self,
            int body_id,
            np.ndarray[double, ndim=1, mode="c"] quat,
            np.ndarray[double, ndim=1, mode="c"] q):
        quat_wrap = Quaternion.fromPythonArray (quat)
        q_wrap = VectorNd.fromPythonArray (q)
        self.thisptr.SetQuaternion (body_id,
            (<Quaternion>quat_wrap).thisptr[0],
            (<VectorNd>q_wrap).thisptr[0])
        for i in range(len(q)):
            q[i] = q_wrap[i]

    def GetQuaternion (self,
            int body_id,
            np.ndarray[double, ndim=1, mode="c"] q):
        return QuaternionToNumpy (self.thisptr.GetQuaternion(body_id, NumpyToVectorNd (q)))

    def GetBody (self, index):
        return Body (<uintptr_t> &(self.thisptr.mBodies[index]))

    property dof_count:
        def __get__ (self):
            return self.thisptr.dof_count

    property q_size:
        def __get__ (self):
            return self.thisptr.q_size

    property qdot_size:
        def __get__ (self):
            return self.thisptr.qdot_size

    property previously_added_body_id:
        def __get__ (self):
            return self.thisptr.previously_added_body_id

    property gravity:
        def __get__ (self):
            return np.ndarray ([
                self.thisptr.gravity[0],
                self.thisptr.gravity[1],
                self.thisptr.gravity[2],
                ]
                )
        def __set__ (self, values):
            for i in range (0,3):
                self.thisptr.gravity[i] = values[i]

    property v:
        def __get__ (self):
            return self.v

    property a:
        def __get__ (self):
            return self.a


    property mJoints:
        def __get__ (self):
            return self.mJoints

    property S:
        def __get__ (self):
            return self.S

    property X_J:
        def __get__ (self):
            return self.X_J

    property v_J:
        def __get__ (self):
            return self.v_J

    property c_J:
        def __get__ (self):
            return self.c_J


    property mJointUpdateOrder:
        def __get__ (self):
            return self.thisptr.mJointUpdateOrder

    property X_T:
        def __get__ (self):
            return self.X_T


    property mFixedJointCount:
        def __get__ (self):
            return self.thisptr.mFixedJointCount

    # TODO
    # multdof3_S
    # multdof3_U
    # multdof3_Dinv
    # multdof3_u

    property multdof3_w_index:
        def __get__ (self):
            return self.thisptr.multdof3_w_index

    property c:
        def __get__ (self):
            return self.c

    property IA:
        def __get__ (self):
            return self.IA

    property pA:
        def __get__ (self):
            return self.pA

    property U:
        def __get__ (self):
            return self.U


    # TODO
    # d
    # u

    property f:
        def __get__ (self):
            return self.f

    property I:
        def __get__ (self):
            return self.I

    property Ic:
        def __get__ (self):
            return self.Ic

    property hc:
        def __get__ (self):
            return self.hc


    property X_lambda:
        def __get__ (self):
            return self.X_lambda

    property X_base:
        def __get__ (self):
            return self.X_base


    property mFixedBodies:
        def __get__ (self):
            return self.mFixedBodies


    property fixed_body_discriminator:
        def __get__ (self):
            return self.thisptr.fixed_body_discriminator

    property mBodies:
        def __get__ (self):
            return self.mBodies


##############################
#
# Constraint Types
#
##############################

cdef class ConstraintSet

cdef class _ConstraintSet_point_Vector3d_VectorWrapper:
    cdef crbdl.ConstraintSet *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.ConstraintSet *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [Vector3d.fromPointer (<uintptr_t> &(self.parent.point[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return Vector3d.fromPointer (<uintptr_t> &(self.parent.point[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], Vector3d), "Invalid type! Expected Vector3d, but got " + str(type(value[src_index])) + "."
                self.parent.point[i] = (<Vector3d> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, Vector3d), "Invalid type! Expected Vector3d, but got " + str(type(value)) + "."
            self.parent.point[key] = (<Vector3d> value).thisptr[0]

    def __len__(self):
        return self.parent.point.size()

cdef class _ConstraintSet_normal_Vector3d_VectorWrapper:
    cdef crbdl.ConstraintSet *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.ConstraintSet *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [Vector3d.fromPointer (<uintptr_t> &(self.parent.normal[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return Vector3d.fromPointer (<uintptr_t> &(self.parent.normal[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], Vector3d), "Invalid type! Expected Vector3d, but got " + str(type(value[src_index])) + "."
                self.parent.normal[i] = (<Vector3d> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, Vector3d), "Invalid type! Expected Vector3d, but got " + str(type(value)) + "."
            self.parent.normal[key] = (<Vector3d> value).thisptr[0]

    def __len__(self):
        return self.parent.normal.size()


cdef class ConstraintSet:
    cdef crbdl.ConstraintSet *thisptr
    cdef _ConstraintSet_point_Vector3d_VectorWrapper point
    cdef _ConstraintSet_normal_Vector3d_VectorWrapper normal

    def __cinit__(self):
        self.thisptr = new crbdl.ConstraintSet()
        self.point = _ConstraintSet_point_Vector3d_VectorWrapper (<uintptr_t> self.thisptr)
        self.normal = _ConstraintSet_normal_Vector3d_VectorWrapper (<uintptr_t> self.thisptr)

    def __dealloc__(self):
        del self.thisptr

    def __repr__(self):
        return "rbdl.ConstraintSet (0x{:0x})".format(<uintptr_t><void *> self.thisptr)

    def AddConstraint (self, 
            body_id not None,
            body_point not None,
            world_normal not None,
            constraint_name = None,
            normal_acceleration = 0.):
        cdef crbdl.Vector3d c_body_point
        cdef crbdl.Vector3d c_world_normal

        for i in range (3):
            c_body_point[i] = body_point[i]
            c_world_normal[i] = world_normal[i]

        return self.thisptr.AddConstraint (
                body_id,
                c_body_point,
                c_world_normal,
                constraint_name,
                normal_acceleration
                )

    def Bind (self, model):
        return self.thisptr.Bind ((<Model>model).thisptr[0])

    def size (self):
        return self.thisptr.size()

    def clear (self):
        self.thisptr.clear()

    property bound:
        def __get__ (self):
            return self.thisptr.bound

#    %VectorWrapperAddProperty (TYPE=string, MEMBER=name, PARENT=ConstraintSet)%

    property point:
        def __get__ (self):
            return self.point

    property normal:
        def __get__ (self):
            return self.normal


#    property acceleration:
#        def __get__(self):
#            return VectorNd.fromPointer (<uintptr_t> &(self.thisptr.acceleration)).toNumpy()
#        def __set__(self, values):
#            vec = VectorNd.fromPythonArray (values)
#            self.thisptr.acceleration = <crbdl.VectorNd> (vec.thisptr[0])

##############################
#
# Kinematics.h
#
##############################

def CalcBodyToBaseCoordinates (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        update_kinematics=True):
    return Vector3dToNumpy (crbdl.CalcBodyToBaseCoordinates (
            model.thisptr[0],
            NumpyToVectorNd (q),
            body_id,
            NumpyToVector3d (body_point_position),
            update_kinematics 
            ))

def CalcBaseToBodyCoordinates (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        update_kinematics=True):
    return Vector3dToNumpy (crbdl.CalcBaseToBodyCoordinates (
            model.thisptr[0],
            NumpyToVectorNd (q),
            body_id,
            NumpyToVector3d (body_point_position),
            update_kinematics 
            ))

def CalcPointVelocity (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        update_kinematics=True):
    return Vector3dToNumpy (crbdl.CalcPointVelocity (
            model.thisptr[0],
            NumpyToVectorNd (q),
            NumpyToVectorNd (qdot),
            body_id,
            NumpyToVector3d (body_point_position),
            update_kinematics 
            ))

def CalcPointAcceleration (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        np.ndarray[double, ndim=1, mode="c"] qddot,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        update_kinematics=True):
    return Vector3dToNumpy (crbdl.CalcPointAcceleration (
            model.thisptr[0],
            NumpyToVectorNd (q),
            NumpyToVectorNd (qdot),
            NumpyToVectorNd (qddot),
            body_id,
            NumpyToVector3d (body_point_position),
            update_kinematics 
            ))

def CalcPointVelocity6D (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        update_kinematics=True):
    return SpatialVectorToNumpy (crbdl.CalcPointVelocity6D (
            model.thisptr[0],
            NumpyToVectorNd (q),
            NumpyToVectorNd (qdot),
            body_id,
            NumpyToVector3d (body_point_position),
            update_kinematics 
            ))

def CalcPointAcceleration6D (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        np.ndarray[double, ndim=1, mode="c"] qddot,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        update_kinematics=True):
    return SpatialVectorToNumpy (crbdl.CalcPointAcceleration6D (
            model.thisptr[0],
            NumpyToVectorNd (q),
            NumpyToVectorNd (qdot),
            NumpyToVectorNd (qddot),
            body_id,
            NumpyToVector3d (body_point_position),
            update_kinematics 
            ))

def CalcPointJacobian (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        np.ndarray[double, ndim=2, mode="c"] G,
        update_kinematics=True):
    crbdl.CalcPointJacobianPtr (
            model.thisptr[0],
            <double*>q.data,
            body_id,
            NumpyToVector3d (body_point_position),
            <double*>G.data,
            update_kinematics 
            )

def CalcPointJacobian6D (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        np.ndarray[double, ndim=2, mode="c"] G,
        update_kinematics=True):
    crbdl.CalcPointJacobian6DPtr (
            model.thisptr[0],
            <double*>q.data,
            body_id,
            NumpyToVector3d (body_point_position),
            <double*>G.data,
            update_kinematics 
            )

def CalcBodySpatialJacobian(Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        np.ndarray[double, ndim=2, mode="c"] G,
        update_kinematics=True):
    crbdl.CalcBodySpatialJacobianPtr(
            model.thisptr[0],
            <double*>q.data,
            body_id,
            <double*>G.data,
            update_kinematics 
            )

##############################
#
# rbdl_utils.h
#
##############################

def CalcCenterOfMass (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        np.ndarray[double, ndim=1, mode="c"] com,
        np.ndarray[double, ndim=1, mode="c"] com_velocity=None,
        np.ndarray[double, ndim=1, mode="c"] angular_momentum=None,
        update_kinematics=True):
    cdef double cmass
    cdef crbdl.Vector3d c_com = crbdl.Vector3d()
    cdef crbdl.Vector3d* c_com_vel_ptr # = crbdl.Vector3d()
    cdef crbdl.Vector3d* c_ang_momentum_ptr # = crbdl.Vector3d()

    c_com_vel_ptr = <crbdl.Vector3d*> NULL
    c_ang_momentum_ptr = <crbdl.Vector3d*> NULL

    if com_velocity != None:
        c_com_vel_ptr = new crbdl.Vector3d()

    if angular_momentum != None:
        c_ang_momentum_ptr = new crbdl.Vector3d()

    cmass = 0.0
    crbdl.CalcCenterOfMass (
            model.thisptr[0],
            NumpyToVectorNd (q),
            NumpyToVectorNd (qdot),
            cmass,
            c_com,
            c_com_vel_ptr,
            c_ang_momentum_ptr,
            update_kinematics)

    com[0] = c_com[0]
    com[1] = c_com[1]
    com[2] = c_com[2]

    if com_velocity != None:
        com_velocity[0] = c_com_vel_ptr.data()[0]
        com_velocity[1] = c_com_vel_ptr.data()[1]
        com_velocity[2] = c_com_vel_ptr.data()[2]
        del c_com_vel_ptr

    if angular_momentum != None:
        angular_momentum[0] = c_ang_momentum_ptr.data()[0]
        angular_momentum[1] = c_ang_momentum_ptr.data()[1]
        angular_momentum[2] = c_ang_momentum_ptr.data()[2]
        del c_ang_momentum_ptr

    return cmass 

##############################
#
# Dynamics.h
#
##############################

def InverseDynamics (Model model, 
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        np.ndarray[double, ndim=1, mode="c"] qddot,
        np.ndarray[double, ndim=1, mode="c"] tau):
    crbdl.InverseDynamicsPtr (model.thisptr[0], 
            <double*>q.data, 
            <double*>qdot.data, 
            <double*>qddot.data, 
            <double*>tau.data,
            NULL
            )

def NonlinearEffects (Model model, 
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        np.ndarray[double, ndim=1, mode="c"] tau):
    crbdl.NonlinearEffectsPtr (model.thisptr[0], 
            <double*>q.data, 
            <double*>qdot.data, 
            <double*>tau.data
            )

def CompositeRigidBodyAlgorithm (Model model, 
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=2, mode="c"] H,
        update_kinematics=True):
    crbdl.CompositeRigidBodyAlgorithmPtr (model.thisptr[0],
            <double*>q.data,
            <double*>H.data,
            update_kinematics);

def ForwardDynamics (Model model, 
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        np.ndarray[double, ndim=1, mode="c"] tau,
        np.ndarray[double, ndim=1, mode="c"] qddot):
    crbdl.ForwardDynamicsPtr (model.thisptr[0], 
            <double*>q.data, 
            <double*>qdot.data, 
            <double*>tau.data,
            <double*>qddot.data, 
            NULL
            )

def loadModel (
        filename,
        **kwargs
        ):
    verbose = False
    if "verbose" in kwargs.keys():
        verbose=kwargs["verbose"]

    floating_base = False
    if "floating_base" in kwargs.keys():
        floating_base=kwargs["floating_base"]

    result = Model()
    if crbdl.rbdl_loadmodel (filename, result.thisptr, floating_base, verbose):
        return result

    print ("Error loading model {}!".format (filename))
    return None

