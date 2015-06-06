import numpy as np
cimport numpy as np
from libc.stdint cimport uintptr_t
from libcpp.string cimport string

cimport crbdl

cdef class SpatialVector:
    cdef crbdl.SpatialVector *thisptr

    def __cinit__(self):
        self.thisptr = new crbdl.SpatialVector()

    def __dealloc__(self):
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

cdef class Body:
    cdef crbdl.Body *thisptr
    cdef free_on_dealloc

    def __cinit__(self, uintptr_t address=0):
        if address == 0:
            self.free_on_dealloc = True
            self.thisptr = new crbdl.Body()
        else:
            self.free_on_dealloc = False
            self.thisptr = <crbdl.Body*>address

    def __dealloc__(self):
        if self.free_on_dealloc:
            del self.thisptr

    def __repr__(self):
        return "rbdl.Body (0x{:0x})".format(<uintptr_t><void *> self.thisptr)

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

    def __cinit__(self, joint_type=-1):
        if joint_type == -1:
            self.thisptr = new crbdl.Joint()
        else:
            self.thisptr = new crbdl.Joint(self._joint_type_from_str(joint_type))

    def __dealloc__(self):
        del self.thisptr

    def __repr__(self):
        joint_type_str = "JointTypeUndefined"

        if self.thisptr.mJointType in self.joint_type_map.keys():
            joint_type_str = self.joint_type_map[self.thisptr.mJointType]

        return "rbdl.Joint (0x{:0x}), JointType: {:s}".format(<uintptr_t><void *> self.thisptr, joint_type_str)

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

cdef class BodyArrayWrapper:
    cdef crbdl.Model *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.Model *> ptr

    def __getitem__(self, key):
        return Body (<uintptr_t> &(self.parent.mBodies[key]))

    def __setitem__(self, key, Body value not None):
        self.parent.mBodies[key] = value.thisptr[0]

cdef class Model:
    cdef crbdl.Model *thisptr
    cdef BodyArrayWrapper mBodies

    def __cinit__(self):
        self.thisptr = new crbdl.Model()
        self.mBodies = BodyArrayWrapper (<uintptr_t> self.thisptr)

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

    property mBodies:
        def __get__ (self):
            return self.mBodies
