import numpy as np
cimport numpy as np
from libc.stdint cimport uintptr_t
from libcpp.string cimport string

cimport crbdl

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
        return self.thisptr.data()[key]

    def __setitem__(self, key, value):
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
        self.thisptr.data()[key] = value

    def __len__ (self):
        return 6

    # Constructors
    @classmethod
    def fromPointer(cls, uintptr_t address):
        return SpatialMatrix (address)

cdef class SpatialTransform:
    cdef crbdl.SpatialTransform *thisptr

    def __cinit__(self):
        self.thisptr = new crbdl.SpatialTransform()

    def __dealloc__(self):
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

cdef class Body:
    cdef crbdl.Body *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0, mass=None, com=None, inertia=None):
        cdef double c_mass
        cdef crbdl.Vector3d c_com
        cdef crbdl.Matrix3d c_inertia

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
        return Body (address)

    @classmethod
    def fromMassComInertia(cls, double mass, 
            np.ndarray[double, ndim=1] com,
            np.ndarray[double, ndim=2] inertia):

        return Body (0, mass, com, inertia)

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
    JointTypeFixed
    JointType1DoF
    JointType2DoF
    JointType3DoF
    JointType4DoF
    JointType5DoF
    JointType6DoF

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
            JointTypeFixed: "JointTypeFixed",
            JointType1DoF: "JointType1DoF",
            JointType2DoF: "JointType2DoF",
            JointType3DoF: "JointType3DoF",
            JointType4DoF: "JointType4DoF",
            JointType5DoF: "JointType5DoF",
            JointType6DoF: "JointType6DoF",
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
        result = SpatialVector()
        for i in range (6):
            result[i] = self.thisptr.mJointAxes[index][i]
        return result

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
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.v[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.v[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.v.size()

cdef class _Model_a_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.a[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.a[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.a.size()

cdef class _Model_mJoints_Joint_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return Joint.fromPointer (<uintptr_t> &(self.parent.mJoints[key]))

    def __setitem__(self, key, Joint value not None):
        self.parent.mJoints[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.mJoints.size()

cdef class _Model_S_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.S[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.S[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.S.size()

cdef class _Model_X_J_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_J[key]))

    def __setitem__(self, key, SpatialTransform value not None):
        self.parent.X_J[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.X_J.size()

cdef class _Model_v_J_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.v_J[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.v_J[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.v_J.size()

cdef class _Model_c_J_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.c_J[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.c_J[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.c_J.size()

cdef class _Model_X_T_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_T[key]))

    def __setitem__(self, key, SpatialTransform value not None):
        self.parent.X_T[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.X_T.size()

cdef class _Model_c_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.c[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.c[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.c.size()

cdef class _Model_IA_SpatialMatrix_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialMatrix.fromPointer (<uintptr_t> &(self.parent.IA[key]))

    def __setitem__(self, key, SpatialMatrix value not None):
        self.parent.IA[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.IA.size()

cdef class _Model_pA_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.pA[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.pA[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.pA.size()

cdef class _Model_U_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.U[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.U[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.U.size()

cdef class _Model_f_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.f[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.f[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.f.size()

cdef class _Model_I_SpatialRigidBodyInertia_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialRigidBodyInertia.fromPointer (<uintptr_t> &(self.parent.I[key]))

    def __setitem__(self, key, SpatialRigidBodyInertia value not None):
        self.parent.I[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.I.size()

cdef class _Model_Ic_SpatialRigidBodyInertia_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialRigidBodyInertia.fromPointer (<uintptr_t> &(self.parent.Ic[key]))

    def __setitem__(self, key, SpatialRigidBodyInertia value not None):
        self.parent.Ic[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.Ic.size()

cdef class _Model_hc_SpatialVector_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialVector.fromPointer (<uintptr_t> &(self.parent.hc[key]))

    def __setitem__(self, key, SpatialVector value not None):
        self.parent.hc[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.hc.size()

cdef class _Model_X_lambda_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_lambda[key]))

    def __setitem__(self, key, SpatialTransform value not None):
        self.parent.X_lambda[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.X_lambda.size()

cdef class _Model_X_base_SpatialTransform_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return SpatialTransform.fromPointer (<uintptr_t> &(self.parent.X_base[key]))

    def __setitem__(self, key, SpatialTransform value not None):
        self.parent.X_base[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.X_base.size()

cdef class _Model_mFixedBodies_FixedBody_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return FixedBody.fromPointer (<uintptr_t> &(self.parent.mFixedBodies[key]))

    def __setitem__(self, key, FixedBody value not None):
        self.parent.mFixedBodies[key] = value.thisptr[0]

    def __len__(self):
        return self.parent.mFixedBodies.size()

cdef class _Model_mBodies_Body_VectorWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return Body.fromPointer (<uintptr_t> &(self.parent.mBodies[key]))

    def __setitem__(self, key, Body value not None):
        self.parent.mBodies[key] = value.thisptr[0]

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



cdef class VectorNd:
    cdef crbdl.VectorNd *thisptr

    def __cinit__ (self, ndim, uintptr_t data_ptr=0, pyvalues=None):
        self.thisptr = new crbdl.VectorNd(ndim)

        if pyvalues != None:
            for i in range (ndim):
                self.thisptr.data()[i] = pyvalues[i]

    def __dealloc__(self):
        del self.thisptr

    def __getitem__(self, key):
        return self.thisptr.data()[key]

    def __setitem__(self, key, value):
        self.thisptr.data()[key] = value

    # Constructors
    @classmethod
    def fromPythonArray (cls, python_values):
        return VectorNd (len(python_values), 0, python_values)

def CompositeRigidBodyAlgorithm (Model model, np.ndarray[double,
    ndim=1, mode="c"] q, np.ndarray[double, ndim=2, mode="c"] H, update_kinematics=True):
    cdef crbdl.VectorNd q_in = crbdl.VectorFromPtr (model.qdot_size, &q[0])
    cdef crbdl.MatrixNd H_in = crbdl.MatrixFromPtr (H.shape[0], H.shape[1],
            &H[0,0], False)

    crbdl.CompositeRigidBodyAlgorithm (model.thisptr[0], q_in, H_in, update_kinematics);

    for i in range(H.shape[0]):
        for j in range(H.shape[1]):
            H[i,j] = H_in.coeff(i,j)

