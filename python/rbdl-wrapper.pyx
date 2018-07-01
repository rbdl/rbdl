#cython: boundscheck=False, embedsignature=True

import numpy as np
cimport numpy as np
from enum import IntEnum
from enum import Enum
from libc.stdint cimport uintptr_t
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map as cpp_map
from libcpp cimport bool


cimport cpython.ref as cpy_ref


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

            if pyvalues is not None:
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

            if pyvalues is not None:
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

            if pyvalues is not None:
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
        
        
cdef class MatrixNd:
    cdef crbdl.MatrixNd *thisptr
    cdef free_on_dealloc

    def __cinit__(self, msize = None, uintptr_t address=0, pyvalues = None):
        if address == 0:
            self.free_on_dealloc = True
            
            if pyvalues is not None:
                xdim, ydim = pyvalues.shape()
                for i in range (xdim):
                    for j in range (ydim):
                        (&(self.thisptr.coeff(i,j)))[0] = pyvalues[i,j]
            else:
              if msize != None:
                xdim, ydim = msize
                self.thisptr = new crbdl.MatrixNd(xdim, ydim)
              else:
                raise NameError('empty initialization not possible')
     
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.MatrixNd*>address

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
            x, y = key
            self.thisptr.data()[x*y] = value

    def __len__ (self):
        return self.thisptr.rows() #*self.thisptr.cols()

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return MatrixNd (address)

    @classmethod
    def fromPythonArray (cls, python_values):
        return MatrixNd (0, python_values)



cdef class Quaternion:
    cdef crbdl.Quaternion *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0, pyvalues=None, pymatvalues=None):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.Quaternion()

            if pyvalues is not None:
                for i in range (4):
                    self.thisptr.data()[i] = pyvalues[i]
            elif pymatvalues is not None:
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

    def __cinit__(self, uintptr_t address=0, pyvalues=None):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.SpatialVector()

            if pyvalues is not None:
                for i in range (6):
                    self.thisptr.data()[i] = pyvalues[i]
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

    @classmethod
    def fromPythonArray (cls, python_values):
        return SpatialVector (0, python_values)

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
    
# Matrix3d
cdef crbdl.Matrix3d NumpyToMatrix3d (np.ndarray[double, ndim=2, mode="c"] M):
    cdef crbdl.Matrix3d cM = crbdl.Matrix3d()
    for i in range (3):
        for j in range (3):
            (&(cM.coeff(i,j)))[0] = M[i,j]

    return cM

cdef np.ndarray Matrix3dToNumpy (crbdl.Matrix3d cM):
    result = np.ndarray ([cM.rows(), cM.cols()])
    for i in range (cM.rows()):
        for j in range (cM.cols()):
            result[i,j] = cM.coeff(i,j)

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
cdef crbdl.SpatialVector NumpyToSpatialVector (np.ndarray[double, ndim=1, mode="c"] cx):
    cdef crbdl.SpatialVector result = crbdl.SpatialVector()
    for i in range (6):
        result[i] = cx[i]

    return result

cdef np.ndarray SpatialVectorToNumpy (crbdl.SpatialVector cx):
    result = np.ndarray ((cx.rows()))
    for i in range (cx.rows()):
        result[i] = cx[i]

    return result



#Quaternion
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
            if (mass is not None) and (com is not None) and (inertia is not None):
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

#ctypedef enum JointType:
#    JointTypeUndefined = 0
#    JointTypeRevolute
#    JointTypePrismatic
#    JointTypeRevoluteX
#    JointTypeRevoluteY
#    JointTypeRevoluteZ
#    JointTypeSpherical
#    JointTypeEulerZYX
#    JointTypeEulerXYZ
#    JointTypeEulerYXZ
#    JointTypeTranslationXYZ
#    JointTypeFloatingBase
#    JointTypeFixed
#    JointTypeHelical
#    JointType1DoF
#    JointType2DoF
#    JointType3DoF
#    JointType4DoF
#    JointType5DoF
#    JointType6DoF
#    JointTypeCustom


class PJointType(Enum):
    PJointTypeUndefined = 0
    PJointTypeRevolute = 1
    PJointTypePrismatic = 2
    PJointTypeRevoluteX = 3
    PJointTypeRevoluteY = 4
    PJointTypeRevoluteZ = 5
    PJointTypeSpherical = 6
    PJointTypeEulerZYX = 7
    PJointTypeEulerXYZ = 8
    PJointTypeEulerYXZ = 9
    PJointTypeTranslationXYZ = 10
    PJointTypeFloatingBase = 11
    PJointTypeFixed = 12
    PJointTypeHelical = 13
    PJointType1DoF = 14
    PJointType2DoF = 15
    PJointType3DoF = 16
    PJointType4DoF = 17
    PJointType5DoF = 18
    PJointType6DoF = 19
    PJointTypeCustom = 20


cdef class Joint:
    cdef crbdl.Joint *thisptr
    cdef free_on_dealloc

#    joint_type_map = {
#            JointTypeUndefined: "JointTypeUndefined",
#            JointTypeRevolute: "JointTypeRevolute",
#            JointTypePrismatic: "JointTypePrismatic",
#            JointTypeRevoluteX: "JointTypeRevoluteX",
#            JointTypeRevoluteY: "JointTypeRevoluteY",
#            JointTypeRevoluteZ: "JointTypeRevoluteZ",
#            JointTypeSpherical: "JointTypeSpherical",
#            JointTypeEulerZYX: "JointTypeEulerZYX",
#            JointTypeEulerXYZ: "JointTypeEulerXYZ",
#            JointTypeEulerYXZ: "JointTypeEulerYXZ",
#            JointTypeTranslationXYZ: "JointTypeTranslationXYZ",
#            JointTypeFloatingBase: "JointTypeFloatingBase",
#            JointTypeFixed: "JointTypeFixed",
#            JointTypeHelical: "JointTypeHelical", #1 DoF joint with both rotational and translational motion
#            JointType1DoF: "JointType1DoF",
#            JointType2DoF: "JointType2DoF",
#            JointType3DoF: "JointType3DoF",
#            JointType4DoF: "JointType4DoF",
#            JointType5DoF: "JointType5DoF",
#            JointType6DoF: "JointType6DoF",
#            JointTypeCustom: "JointTypeCustom",
#            }


    joint_type_map = {
            PJointType.PJointTypeUndefined:       crbdl.JointTypeUndefined,
            PJointType.PJointTypeRevolute:        crbdl.JointTypeRevolute,
            PJointType.PJointTypePrismatic:       crbdl.JointTypePrismatic,
            PJointType.PJointTypeRevoluteX:       crbdl.JointTypeRevoluteX,
            PJointType.PJointTypeRevoluteY:       crbdl.JointTypeRevoluteY,
            PJointType.PJointTypeRevoluteZ:       crbdl.JointTypeRevoluteZ,
            PJointType.PJointTypeSpherical:       crbdl.JointTypeSpherical,
            PJointType.PJointTypeEulerZYX:        crbdl.JointTypeEulerZYX,
            PJointType.PJointTypeEulerXYZ:        crbdl.JointTypeEulerXYZ,
            PJointType.PJointTypeEulerYXZ:        crbdl.JointTypeEulerYXZ,
            PJointType.PJointTypeTranslationXYZ:  crbdl.JointTypeTranslationXYZ,
            PJointType.PJointTypeFloatingBase:    crbdl.JointTypeFloatingBase,
            PJointType.PJointTypeFixed:           crbdl.JointTypeFixed,
            PJointType.PJointTypeHelical:         crbdl.JointTypeHelical, #1 DoF joint with both rotational and translational motion
            PJointType.PJointType1DoF:            crbdl.JointType1DoF,
            PJointType.PJointType2DoF:            crbdl.JointType2DoF,
            PJointType.PJointType3DoF:            crbdl.JointType3DoF,
            PJointType.PJointType4DoF:            crbdl.JointType4DoF,
            PJointType.PJointType5DoF:            crbdl.JointType5DoF,
            PJointType.PJointType6DoF:            crbdl.JointType6DoF,
            PJointType.PJointTypeCustom:          crbdl.JointTypeCustom,
            }


    def _joint_type_from_str (self, joint_type_str):
        if joint_type_str not in self.joint_type_map.values():
            raise ValueError("Invalid JointType '" + str(joint_type_str) + "'!")
        else:
            for joint_type, joint_str in self.joint_type_map.iteritems():
                if joint_str == joint_type_str:
                    return joint_type

    def __cinit__(self, uintptr_t address=0, joint_type=-1, dof = -1, axes = None):
        if address == 0:
            self.free_on_dealloc = True
            
            if (axes is not None):

                if len(axes) == 1:
                    self.thisptr = new crbdl.Joint (NumpyToSpatialVector(axes[0]))
                elif len(axes) == 2:
                    self.thisptr = new crbdl.Joint (NumpyToSpatialVector(axes[0]), 
                                                    NumpyToSpatialVector(axes[1]))
                elif len(axes) == 3:
                    self.thisptr = new crbdl.Joint (NumpyToSpatialVector(axes[0]),
                                                    NumpyToSpatialVector(axes[1]),
                                                    NumpyToSpatialVector(axes[2]))
                elif len(axes) == 4:
                    self.thisptr = new crbdl.Joint (NumpyToSpatialVector(axes[0]),
                                                    NumpyToSpatialVector(axes[1]),
                                                    NumpyToSpatialVector(axes[2]),
                                                    NumpyToSpatialVector(axes[3]))
                elif len(axes) == 5:
                    self.thisptr = new crbdl.Joint (NumpyToSpatialVector(axes[0]),
                                                    NumpyToSpatialVector(axes[1]),
                                                    NumpyToSpatialVector(axes[2]),
                                                    NumpyToSpatialVector(axes[3]),
                                                    NumpyToSpatialVector(axes[4]))
                else:
                    self.thisptr = new crbdl.Joint (NumpyToSpatialVector(axes[0]),
                                                    NumpyToSpatialVector(axes[1]),
                                                    NumpyToSpatialVector(axes[2]),
                                                    NumpyToSpatialVector(axes[3]),
                                                    NumpyToSpatialVector(axes[4]),
                                                    NumpyToSpatialVector(axes[5]))
            else: 
              if joint_type == -1:
                self.thisptr = new crbdl.Joint()
              else:
                
               # joint_type = self._joint_type_from_str(joint_type)
                
                
                if dof != -1:
                  self.thisptr = new crbdl.Joint(<crbdl.JointType> self.joint_type_map[joint_type], <int> dof)
                else:
                  self.thisptr = new crbdl.Joint(<crbdl.JointType> self.joint_type_map[joint_type])
                  
                  
               # print self._joint_type_from_str(joint_type) 
               # self.thisptr = new crbdl.Joint(self._joint_type_from_str(joint_type))
               # joint_type = self._joint_type_from_str(joint_type)
               # self.thisptr = new crbdl.Joint(<JointType> joint_type)
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
    def forCustomJoint(cls, joint_type, int dof):
        return Joint (0, joint_type, dof)

    @classmethod
    def fromJointAxes(cls, np.ndarray[double, ndim=2, mode="c"] axes):
        assert (len(axes) > 0)
        assert (len(axes) < 7)
        assert (len(axes[0]) == 6)
        axes_count = len(axes)
    #    joint_type = JointType1DoF + axes_count - 1
    
        result = Joint (0, -1, -1, axes)
   
   #     result = Joint (0, cls.joint_type_map[joint_type])

   #     for i in range (axes_count):
   #         result.setJointAxis(i, axes[i])

        return result

    property mDoFCount:
        def __get__ (self):
            return self.thisptr.mDoFCount

        def __set__ (self, value):
            self.thisptr.mDoFCount = value

    property mJointType:
        def __get__ (self):
            return self.joint_type_map[self.thisptr.mJointType]

# was NOT WRITABLE originally - had to be changed because of CustomJoint
    property q_index:
        def __get__ (self):
            return self.thisptr.q_index
        
        def __set__ (self, value):
            self.thisptr.q_index = value
    
    property custom_joint_index:
        def __get__ (self):
            return self.thisptr.custom_joint_index

    def getJointAxis (self, index):
        assert index >= 0 and index < self.thisptr.mDoFCount, "Invalid joint axis index!"
        return SpatialVectorToNumpy (self.thisptr.mJointAxes[index])

    def setJointAxis (self, index, value):
        assert index >= 0 and index < self.thisptr.mDoFCount, "Invalid joint axis index!"
        for i in range (6):
            (&(self.thisptr.mJointAxes[index][i]))[0] = value[i]
            self.thisptr.mJointAxes[index][i]


cdef class CustomJoint:
  
    cdef crbdl.ICustomJoint* thisptr
    cdef free_on_dealloc
    
    
    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.ICustomJoint(<cpy_ref.PyObject*>self)
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.ICustomJoint*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr
    
#    def jcalc (self, Model model,
#                joint_id,
#                np.ndarray[double, ndim = 1, mode="c"] q,
#                np.ndarray[double, ndim = 1, mode="c"] qdot
#              ):
#        self.thisptr.jcalc (model.thisptr[0],
#                joint_id,
#                NumpyToVectorNd(q),
#                NumpyToVectorNd(qdot)
#                )
                
#    def jcalc_X_lambda_S (self, Model model,
#                          joint_id,
#                          np.ndarray[double, ndim = 1, mode="c"] q
#                          ):
#        self.thisptr.jcalc_X_lambda_S (model.thisptr[0],
#                                joint_id,
#                                NumpyToVectorNd(q),
#                          )
     
    
    property mDoFCount:
        def __get__ (self):
            return self.thisptr.mDoFCount

        def __set__ (self, value):
            self.thisptr.mDoFCount = value
            
    property XJ:
        def __get__ (self):
            ST = SpatialTransform()
            #resultE = np.ndarray ((self.thisptr.XJ.E.rows(),self.thisptr.XJ.E.cols()))
            ST.r = (Vector3d.fromPointer (<uintptr_t> &(self.thisptr.XJ.r))).toNumpy()
            for i in range (self.thisptr.XJ.E.rows()):
                for j in range (self.thistr.XJ.E.cols()):
                    ST.E[i,j] = self.thisptr.XJ.E.coeff(i,j)

            return ST

        def __set__ (self, value):
            for i in range (value.r.shape[0]):
                    (&(self.thisptr.XJ.r[i]))[0] = value.r[i]   
            for i in range (value.E.shape[0]):
                for j in range (value.E.shape[1]):
                    (&(self.thisptr.XJ.E.coeff(i,j)))[0] = value.E[i,j]
    
    property S:
        def __get__ (self):
            result = np.ndarray ((self.thisptr.S.rows(),self.thisptr.S.cols()))
            for i in range (self.thisptr.S.rows()):
                for j in range (self.thisptr.S.cols()):
                    result[i,j] = self.thisptr.S.coeff(i,j)

            return result

        def __set__ (self, value):
            self.thisptr.S.resize(value.shape[0],value.shape[1])
            for i in range (value.shape[0]):
                for j in range (value.shape[1]):
                    (&(self.thisptr.S.coeff(i,j)))[0] = value[i,j]
        
#        def __getitem__(self, key):
#          if isinstance( key, slice ) :
#              raise Error('slices not supported yet')
#          else:
#              x, y = key
#              return self.thisptr.S.coeff(x,y)
            

#        def __setitem__(self, key, value):
            
#            if isinstance( key, slice ) :
#                raise Error('slices not supported yet')
#            else:
#                x, y = key
#                (&(self.thisptr.S.coeff(x,y)))[0] = value

    
    property U:
        def __get__ (self):
            result = np.ndarray ((self.thisptr.U.rows(),self.thisptr.U.cols()))
            for i in range (self.thisptr.U.rows()):
                for j in range (self.thisptr.U.cols()):
                    result[i,j] = self.thisptr.U.coeff(i,j)

            return result

        def __set__ (self, value):
            self.thisptr.U.resize(value.shape[0], value.shape[1])
            for i in range (value.shape[0]):
                for j in range (value.shape[1]):
                    (&(self.thisptr.U.coeff(i,j)))[0] = value[i,j]     
    
    property Dinv:
        def __get__ (self):
            result = np.ndarray ((self.thisptr.Dinv.rows(),self.thisptr.Dinv.cols()))
            for i in range (self.thisptr.Dinv.rows()):
                for j in range (self.thisptr.Dinv.cols()):
                    result[i,j] = self.thisptr.Dinv.coeff(i,j)

            return result

        def __set__ (self, value):
            self.thisptr.Dinv.resize(value.shape[0], value.shape[1])
            for i in range (value.shape[0]):
                for j in range (value.shape[1]):
                    (&(self.thisptr.Dinv.coeff(i,j)))[0] = value[i,j]     
    
    property u:
        def __get__ (self):
            return (VectorNd.fromPointer (<uintptr_t> &(self.thisptr.u))).toNumpy()

        def __set__ (self, value):
           self.thisptr.u.resize(value.shape[0])
           #vec = VectorNd.fromPythonArray (value)
           #self.thisptr.e = <crbdl.VectorNd> (vec.thisptr[0]) 
           for i in range (value.shape[0]):
                    (&(self.thisptr.u[i]))[0] = value[i]   
    
    property d_u:
        def __get__ (self):
            return (VectorNd.fromPointer (<uintptr_t> &(self.thisptr.d_u))).toNumpy()

        def __set__ (self, value):
           self.thisptr.d_u.resize(value.shape[0])
           #vec = VectorNd.fromPythonArray (value)
           #self.thisptr.e = <crbdl.VectorNd> (vec.thisptr[0]) 
           for i in range (value.shape[0]):
                    (&(self.thisptr.d_u[i]))[0] = value[i]   
                                      
            


cdef public api int cy_call_jcalc(object self, crbdl.Model *model, 
                                        unsigned int joint_id,
                                        crbdl.VectorNd q,
                                        crbdl.VectorNd qdot, 
                                        int *error):
      
      print "has jcalc", (hasattr(self, "jcalc"))
      if (hasattr(self, "jcalc")):
        error[0] = 0
    #    qddot = np.zeros(model.dof_count)
    #    print "c "
    #    for i in range(3):
    #      print model.X_J[joint_id].r[i]
#        print "py1 "
#        error[0] = 0
        c_model = Model()
        del c_model.thisptr
        c_model.thisptr = model
#      #  UpdateKinematics (c_model, VectorNdToNumpy(q), VectorNdToNumpy(qdot), qddot)
#        print "after UppdateKinematics "
        print c_model.gravity
        print c_model.dof_count
        print c_model.mFixedJointCount
        dd = c_model.X_J
        print dd[joint_id]
        
#        print "py2 " , c_model.X_J[joint_id], "ccccc "
#        print c_model.mJoints[joint_id].q_index
#        print c_model.thisptr.mJoints[joint_id].q_index
#        for i in range(3):
#          print c_model.thisptr.X_J[joint_id].r[i]

#        print pymodel.gravity
        
        print "lambda_py", c_model.thisptr._lambda[0]
        print "lambda_c", model._lambda[0]
        
        self.jcalc(c_model, joint_id, VectorNdToNumpy (q), VectorNdToNumpy (qdot))
        print "after jcalc"
        print "lambda_py", c_model.thisptr._lambda[0]
        print "lambda_c", model._lambda[0]
        
        c_model.thisptr = NULL
        del c_model
        
        
      else:
        error[0] = 1
     
cdef public api int cy_call_jcalc_X_lambda_S(object self, 
                                        crbdl.Model *model, 
                                        unsigned int joint_id,
                                        crbdl.VectorNd q, 
                                        int *error):
                                          
      print "has jcalc", (hasattr(self, "pyjcalc_X_lambda_S"))                                  
      if (hasattr(self, "pyjcalc_X_lambda_S")):
        error[0] = 0
        c_model = Model()
        c_model.thisptr = model
#        qdot = np.zeros(model.dof_count)
#        qddot = np.zeros(model.dof_count)
 #       crbdl.UpdateKinematics (c_model.thisptr[0], q, NumpyToVectorNd(qdot), NumpyToVectorNd(qddot))
        getattr(self, "jcalc_X_lambda_S")(c_model, joint_id, VectorNdToNumpy (q));
      else:
        error[0] = 1 
        
        

cdef class Model

%VectorWrapperClassDefinitions(PARENT=Model)%

cdef class Model:
    cdef crbdl.Model *thisptr
    %VectorWrapperMemberDefinitions (PARENT=Model)%

    def __cinit__(self):
        self.thisptr = new crbdl.Model()
        %VectorWrapperCInitCode (PARENT=Model)%

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
    
    def AddBodyCustomJoint (self,
                parent_id,
                SpatialTransform joint_frame not None,
                CustomJoint custom_joint not None,
                Body body not None,
                string body_name = ""):
        
        print custom_joint.thisptr.mDoFCount
        print custom_joint.S
                
        return self.thisptr.AddBodyCustomJoint (
                parent_id,
                joint_frame.thisptr[0],
                custom_joint.thisptr,
                body.thisptr[0],
                body_name
                )
            
#        unsigned int Model::AddBodyCustomJoint (
#    const unsigned int parent_id,
#    const Math::SpatialTransform &joint_frame,
#    CustomJoint *custom_joint,
#    const Body &body,
#    std::string body_name) {
#  Joint proxy_joint (JointTypeCustom, custom_joint->mDoFCount);
#  proxy_joint.custom_joint_index = mCustomJoints.size();
#  //proxy_joint.mDoFCount = custom_joint->mDoFCount; //MM added. Otherwise
#  //model.q_size = 0, which is not good.

#  mCustomJoints.push_back (custom_joint);

#  unsigned int body_id = AddBody (parent_id, 
#      joint_frame, 
#      proxy_joint, 
#      body,
#      body_name);

#  return body_id;
#}

    def SetQuaternion (self,
            unsigned int body_id,
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
            unsigned int body_id,
            np.ndarray[double, ndim=1, mode="c"] q):
        return QuaternionToNumpy (self.thisptr.GetQuaternion(body_id, NumpyToVectorNd (q)))

    def GetBody (self, index):
        return Body (address=<uintptr_t> &(self.thisptr.mBodies[index]))

    def GetParentBodyId (self, index):
        return self.thisptr.GetParentBodyId(index)

    def GetBodyId (self, name):
        return self.thisptr.GetBodyId(name)

    def GetBodyName (self, index):
        return self.thisptr.GetBodyName(index)

    def IsBodyId (self, index):
        return self.thisptr.IsBodyId(index)

    def IsFixedBodyId (self, index):
        return self.thisptr.IsFixedBodyId(index)

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
            return np.array ([
                self.thisptr.gravity[0],
                self.thisptr.gravity[1],
                self.thisptr.gravity[2]
                ]
                )
        def __set__ (self, values):
            for i in range (0,3):
                self.thisptr.gravity[i] = values[i]

#    %VectorWrapperAddProperty (TYPE=int, MEMBER=_lambda, PARENT=Model)%
#    property _lambda:
#        def __get__ (self):
#            vector_size = self.thisptr._lambda.size()
#            print "vector_size",  vector_size
#            ST = np.empty([vector_size], dtype=int)
#            for k in range(vector_size):
#              print "wrappergen ", self.thisptr._lambda[k]
#              ST[k] = self.thisptr._lambda[k]
#            return ST

    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=v, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=a, PARENT=Model)%

    %VectorWrapperAddProperty (TYPE=Joint, MEMBER=mJoints, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=S, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialTransform, MEMBER=X_J, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=v_J, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=c_J, PARENT=Model)%

    property mJointUpdateOrder:
        def __get__ (self):
            return self.thisptr.mJointUpdateOrder

    %VectorWrapperAddProperty (TYPE=SpatialTransform, MEMBER=X_T, PARENT=Model)%

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

    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=c, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialMatrix, MEMBER=IA, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=pA, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=U, PARENT=Model)%


    # TODO
    # d
    # u

    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=f, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialRigidBodyInertia, MEMBER=I, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialRigidBodyInertia, MEMBER=Ic, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialVector, MEMBER=hc, PARENT=Model)%

    %VectorWrapperAddProperty (TYPE=SpatialTransform, MEMBER=X_lambda, PARENT=Model)%
    %VectorWrapperAddProperty (TYPE=SpatialTransform, MEMBER=X_base, PARENT=Model)%

    %VectorWrapperAddProperty (TYPE=FixedBody, MEMBER=mFixedBodies, PARENT=Model)%

    property fixed_body_discriminator:
        def __get__ (self):
            return self.thisptr.fixed_body_discriminator

    %VectorWrapperAddProperty (TYPE=Body, MEMBER=mBodies, PARENT=Model)%
    
    property mBodyNameMap:
      def __get__(self):
        return self.thisptr.mBodyNameMap
    
##############################
#
# Constraint Types
#
##############################

cdef class ConstraintSet

%VectorWrapperClassDefinitions(PARENT=ConstraintSet)%

cdef class ConstraintSet:
    cdef crbdl.ConstraintSet *thisptr
    %VectorWrapperMemberDefinitions (PARENT=ConstraintSet)%

    def __cinit__(self):
        self.thisptr = new crbdl.ConstraintSet()
        %VectorWrapperCInitCode (PARENT=ConstraintSet)%

    def __dealloc__(self):
        del self.thisptr

    def __repr__(self):
        return "rbdl.ConstraintSet (0x{:0x})".format(<uintptr_t><void *> self.thisptr)

    def AddContactConstraint (self,
            body_id not None,
            body_point not None,
            world_normal not None,
            constraint_name = None,
            normal_acceleration = 0.):
        cdef crbdl.Vector3d c_body_point
        cdef crbdl.Vector3d c_world_normal
        cdef char* constraint_name_ptr

        for i in range (3):
            c_body_point[i] = body_point[i]
            c_world_normal[i] = world_normal[i]

        if constraint_name == None:
            constraint_name_ptr = NULL
        else:
            constraint_name_ptr = constraint_name

        return self.thisptr.AddContactConstraint (
                body_id,
                c_body_point,
                c_world_normal,
                constraint_name_ptr,
                normal_acceleration
                )

    def AddLoopConstraint_BaumEn (self,
            id_predecessor not None,
            id_successor not None,
            SpatialTransform X_predecessor not None,
            SpatialTransform X_successor not None,
            SpatialVector axis not None,
            double T_stab_inv,
            constraint_name = None):
        cdef char* constraint_name_ptr

        if constraint_name == None:
            constraint_name_ptr = NULL
        else:
            constraint_name_ptr = constraint_name

        return self.thisptr.AddLoopConstraint (
            id_predecessor,
            id_successor,
            X_predecessor.thisptr[0],
            X_successor.thisptr[0],
            axis.thisptr[0],
            T_stab_inv,
            constraint_name_ptr)
            
            
    
    def AddLoopConstraint (self,
            id_predecessor not None,
            id_successor not None,
            SpatialTransform X_predecessor not None,
            SpatialTransform X_successor not None,
            SpatialVector axis not None,
            baumgarte_enabled,
            double T_stab_inv,
            constraint_name = None):
        cdef char* constraint_name_ptr

        if constraint_name == None:
            constraint_name_ptr = NULL
        else:
            constraint_name_ptr = constraint_name

        return self.thisptr.AddLoopConstraint (
            id_predecessor,
            id_successor,
            X_predecessor.thisptr[0],
            X_successor.thisptr[0],
            axis.thisptr[0],
            baumgarte_enabled,
            T_stab_inv,
            constraint_name_ptr)

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

    %VectorWrapperAddProperty (TYPE=Vector3d, MEMBER=point, PARENT=ConstraintSet)%
    %VectorWrapperAddProperty (TYPE=Vector3d, MEMBER=normal, PARENT=ConstraintSet)%

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

def UpdateKinematics (Model model,
            np.ndarray[double, ndim=1, mode="c"] q not None,
            np.ndarray[double, ndim=1, mode="c"] qdot not None,
            np.ndarray[double, ndim=1, mode="c"] qddot not None):

        crbdl.UpdateKinematics (model.thisptr[0],
            NumpyToVectorNd(q),
            NumpyToVectorNd(qdot),
            NumpyToVectorNd(qddot))
            

def UpdateKinematicsCustom (Model model,
            np.ndarray[double, ndim=1, mode="c"] q,
            np.ndarray[double, ndim=1, mode="c"] qdot = None,
            np.ndarray[double, ndim=1, mode="c"] qddot = None):

    if qddot is None:
      
      crbdl.UpdateKinematicsCustomPtr (model.thisptr[0],
            <double*>q.data,
            <double*>qdot.data,
            NULL)
            
      if qdot is None:
      
        crbdl.UpdateKinematicsCustomPtr (model.thisptr[0],
            <double*>q.data,
            NULL,
            NULL)
    else:
      
        crbdl.UpdateKinematicsCustomPtr (model.thisptr[0],
            <double*>q.data,
            <double*>qdot.data,
            <double*>qddot.data)


def CalcBodyToBaseCoordinates (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        unsigned int body_id,
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
        unsigned int body_id,
        np.ndarray[double, ndim=1, mode="c"] body_point_position,
        update_kinematics=True):
    return Vector3dToNumpy (crbdl.CalcBaseToBodyCoordinates (
            model.thisptr[0],
            NumpyToVectorNd (q),
            body_id,
            NumpyToVector3d (body_point_position),
            update_kinematics
            ))
            
def CalcBodyWorldOrientation (Model model,
           np.ndarray[double, ndim=1, mode="c"] q,
           unsigned int body_id,
           update_kinematics = True):    
    
    return Matrix3dToNumpy( crbdl.CalcBodyWorldOrientation (
          model.thisptr[0],
          NumpyToVectorNd (q),
          body_id,
          update_kinematics
          ))

def CalcPointVelocity (Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        unsigned int body_id,
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
        unsigned int body_id,
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
        unsigned int body_id,
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
        unsigned int body_id,
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
        unsigned int body_id,
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
        unsigned int body_id,
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
        unsigned int body_id,
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
            
def InverseKinematics (Model model, 
        np.ndarray[double, ndim=1, mode="c"] qinit, 
        np.ndarray[np.int64_t, ndim=1] body_ids, 
        np.ndarray[double, ndim=2, mode="c"] body_point_position, 
        np.ndarray[double, ndim=2, mode="c"] target_pos_position, 
        np.ndarray[double, ndim=1, mode="c"] qres, 
        double step_tol = 1.0e-12, 
        double lambda_ = 0.01, 
        int max_iter = 50):
          
  cdef vector[unsigned int] body_id
  cdef vector[crbdl.Vector3d] body_point
  cdef vector[crbdl.Vector3d] target_pos	
  cdef crbdl.Vector3d buf
  
	
  for i in range( body_ids.shape[0]):
    body_id.push_back(<unsigned int> body_ids[i])		
  for i in range( body_point_position.shape[0]):
    buf = NumpyToVector3d(body_point_position[i])
    body_point.push_back(buf)		
  for i in range( target_pos_position.shape[0]):
    buf = NumpyToVector3d(target_pos_position[i])
    target_pos.push_back(buf)						
  
  return crbdl.InverseKinematicsPtr (model.thisptr[0],
						<double*> qinit.data,
						body_id,
						body_point,
						target_pos,
						<double*> qres.data,
						<double> step_tol,
						<double> lambda_,
						<unsigned int> max_iter
						)
						

class ConstraintType(IntEnum):
    ConstraintTypePosition = 0
    ConstraintTypeOrientation = 1
    ConstraintTypeFull = 2
    ConstraintTypePositionXY = 3
    ConstraintTypePositionZ = 4
    ConstraintTypePositionCoMXY = 5


     

cdef class InverseKinematicsConstraintSet

%VectorWrapperClassDefinitions(PARENT=InverseKinematicsConstraintSet)%

cdef class InverseKinematicsConstraintSet:
    cdef crbdl.InverseKinematicsConstraintSet *thisptr
    %VectorWrapperMemberDefinitions (PARENT=InverseKinematicsConstraintSet)%

    def __cinit__(self):
        self.thisptr = new crbdl.InverseKinematicsConstraintSet()
        %VectorWrapperCInitCode (PARENT=InverseKinematicsConstraintSet)%

    def __dealloc__(self):
        del self.thisptr

    def __repr__(self):
        return "rbdl.InverseKinematicsConstraintSet (0x{:0x})".format(<uintptr_t><void *> self.thisptr)



    def AddPointConstraint (self,
            body_id not None,
            body_point not None,
            target_pos not None,
            weight = 1.):     

        return self.thisptr.AddPointConstraint (
                body_id,
                NumpyToVector3d(body_point),
                NumpyToVector3d(target_pos),
				weight
                )
    
    def AddPointConstraintXY (self,
            body_id not None,
            body_point not None,
            target_pos not None,
            weight = 1.):     

        return self.thisptr.AddPointConstraintXY (
                body_id,
                NumpyToVector3d(body_point),
                NumpyToVector3d(target_pos),
                weight
                )
                
    def AddPointConstraintZ (self,
            body_id not None,
            body_point not None,
            target_pos not None,
            weight = 1.):     

        return self.thisptr.AddPointConstraintZ (
                body_id,
                NumpyToVector3d(body_point),
                NumpyToVector3d(target_pos),
                weight
                )
    
    def AddPointConstraintCoMXY (self,
            body_id not None,
            target_pos not None,
            weight = 1.):     

        return self.thisptr.AddPointConstraintCoMXY (
                body_id,
                NumpyToVector3d(target_pos),
                weight
                )


    def AddOrientationConstraint (self,
            body_id not None,
            target_orientation not None,
            weight = 1.):

        return self.thisptr.AddOrientationConstraint (
            body_id,
            NumpyToMatrix3d(target_orientation),
            weight
            )

            
    def AddFullConstraint (self,
            body_id not None,
            body_point not None,
            target_pos not None,
            target_orientation not None,
            weight = 1.):

        return self.thisptr.AddFullConstraint (
            body_id,
            NumpyToVector3d(body_point),
            NumpyToVector3d(target_pos),
            NumpyToMatrix3d(target_orientation),
			weight
			)

    def ClearConstraints (self):
        return self.thisptr.ClearConstraints()

  
    property J:
        def __get__ (self):
            result = np.ndarray ((self.thisptr.J.rows(),self.thisptr.J.cols()))
            for i in range (self.thisptr.J.rows()):
                for j in range (self.thisptr.J.cols()):
                    result[i,j] = self.thisptr.J.coeff(i,j)

            return result

        def __set__ (self, value):
            self.thisptr.J.resize(value.shape[0], value.shape[1])
            for i in range (value.shape[0]):
                for j in range (value.shape[1]):
                    (&(self.thisptr.J.coeff(i,j)))[0] = value[i,j]
    
    property G:
        def __get__ (self):
            result = np.ndarray ((self.thisptr.G.rows(),self.thisptr.G.cols()))
            for i in range (self.thisptr.G.rows()):
                for j in range (self.thisptr.G.cols()):
                    result[i,j] = self.thisptr.G.coeff(i,j)

            return result

        def __set__ (self, value):
            self.thisptr.G.resize(value.shape[0], value.shape[1])
            for i in range (value.shape[0]):
                for j in range (value.shape[1]):
                    (&(self.thisptr.G.coeff(i,j)))[0] = value[i,j]      
    
    property e:
        def __get__ (self):
            return (VectorNd.fromPointer (<uintptr_t> &(self.thisptr.e))).toNumpy()

        def __set__ (self, value):
           self.thisptr.e.resize(value.shape[0])
           for i in range (value.shape[0]):
                    (&(self.thisptr.e[i]))[0] = value[i]      
                
    property num_constraints:
        def __get__ (self):
            return self.thisptr.num_constraints
            
          
    property dlambda:
        def __get__ (self):
            return self.thisptr.dlambda
        
        def __set__ (self, value):
            self.thisptr.dlambda = value
            
    property num_steps:
        def __get__ (self):
            return self.thisptr.num_steps
        
        def __set__ (self, value):
            self.thisptr.num_steps = value
            
    property max_steps:
        def __get__ (self):
            return self.thisptr.max_steps
        
        def __set__ (self, value):
            self.thisptr.max_steps = value
            
    property step_tol:
        def __get__ (self):
            return self.thisptr.step_tol
        
        def __set__ (self, value):
            self.thisptr.step_tol = value
            
    property constraint_tol:
        def __get__ (self):
            return self.thisptr.constraint_tol
        
        def __set__ (self, value):
            self.thisptr.constraint_tol = value
            
    property error_norm:
        def __get__ (self):
            return self.thisptr.error_norm
        
        def __set__ (self, value):
            self.thisptr.error_norm = value

    property body_ids:
        def __get__ (self):
          return self.thisptr.body_ids
    
        def __set__ (self, value):
            self.thisptr.body_ids = value 
            
    property constraint_weight:
        def __get__ (self):
          return self.thisptr.constraint_weight
    
        def __set__ (self, value):
            self.thisptr.constraint_weight = value 
       
    
    property constraint_row_index:
        def __get__ (self):
          return self.thisptr.constraint_row_index
    
        def __set__ (self, value):
            self.thisptr.constraint_row_index = value 
            
    property constraint_type:
        def __get__ (self):
          type_list = []  
          for i in range(self.thisptr.constraint_type.size()):
            type_list.append(ConstraintType(self.thisptr.constraint_type[i]))
          
          return type_list
     
     
    %VectorWrapperAddProperty (TYPE=Vector3d, MEMBER=body_points, PARENT=InverseKinematicsConstraintSet)%
    %VectorWrapperAddProperty (TYPE=Vector3d, MEMBER=target_positions, PARENT=InverseKinematicsConstraintSet)%
    %VectorWrapperAddProperty (TYPE=Matrix3d, MEMBER=target_orientations, PARENT=InverseKinematicsConstraintSet)%



def InverseKinematicsCS (Model model, 
    np.ndarray[double, ndim=1, mode="c"] qinit, 
    InverseKinematicsConstraintSet CS,
    np.ndarray[double, ndim=1, mode="c"] qres):
          					
  
    return crbdl.InverseKinematicsCSPtr (model.thisptr[0],
        <double*> qinit.data,
				CS.thisptr[0],
				<double*> qres.data,
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

    if com_velocity is not None:
        c_com_vel_ptr = new crbdl.Vector3d()

    if angular_momentum is not None:
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

    if com_velocity is not None:
        com_velocity[0] = c_com_vel_ptr.data()[0]
        com_velocity[1] = c_com_vel_ptr.data()[1]
        com_velocity[2] = c_com_vel_ptr.data()[2]
        del c_com_vel_ptr

    if angular_momentum is not None:
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
        np.ndarray[double, ndim=1, mode="c"] tau,
        np.ndarray[double, ndim=2, mode="c"] f_external = None):
      
    cdef vector[crbdl.SpatialVector] *f_ext = new vector[crbdl.SpatialVector]()
    
    if f_external is None:
        
        crbdl.InverseDynamicsPtr (model.thisptr[0],
            <double*>q.data,
            <double*>qdot.data,
            <double*>qddot.data,
            <double*>tau.data,
            NULL
            )
        
    else:
         
        for ele in f_external: 
            f_ext.push_back(NumpyToSpatialVector(ele))
            
        crbdl.InverseDynamicsPtr (model.thisptr[0],
            <double*>q.data,
            <double*>qdot.data,
            <double*>qddot.data,
            <double*>tau.data,
            f_ext
            )
            
    del f_ext
            

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
        np.ndarray[double, ndim=1, mode="c"] qddot,
        np.ndarray[double, ndim=2, mode="c"] f_external = None):
      
    cdef vector[crbdl.SpatialVector] *f_ext = new vector[crbdl.SpatialVector]()
 
    if f_external is None:
        
        crbdl.ForwardDynamicsPtr (model.thisptr[0],
            <double*>q.data,
            <double*>qdot.data,
            <double*>tau.data,
            <double*>qddot.data,
            NULL
            )
    
    else:
      
        for ele in f_external: 
            f_ext.push_back(NumpyToSpatialVector(ele))
            
        crbdl.ForwardDynamicsPtr (model.thisptr[0],
            <double*>q.data,
            <double*>qdot.data,
            <double*>tau.data,
            <double*>qddot.data,
            f_ext
            )
    
    del f_ext
      
def ForwardDynamicsConstraintsDirect (
        Model model,
        np.ndarray[double, ndim=1, mode="c"] q,
        np.ndarray[double, ndim=1, mode="c"] qdot,
        np.ndarray[double, ndim=1, mode="c"] tau,
        ConstraintSet CS,
        np.ndarray[double, ndim=1, mode="c"] qddot,
        np.ndarray[double, ndim=2, mode="c"] f_external = None):
      
    cdef vector[crbdl.SpatialVector] *f_ext = new vector[crbdl.SpatialVector]()
    
    if f_external is None:
        
        crbdl.ForwardDynamicsConstraintsDirectPtr (
        model.thisptr[0],
        <double*>q.data,
        <double*>qdot.data,
        <double*>tau.data,
        CS.thisptr[0],
        <double*>qddot.data,
        NULL
        )
    
    else:
      
        for ele in f_external: 
            f_ext.push_back(NumpyToSpatialVector(ele))
        
        crbdl.ForwardDynamicsConstraintsDirectPtr (
            model.thisptr[0],
            <double*>q.data,
            <double*>qdot.data,
            <double*>tau.data,
            CS.thisptr[0],
            <double*>qddot.data,
            f_ext
            )
            
    del f_ext
    
            
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

