from libcpp.string cimport string
from libcpp cimport bool
from libcpp.vector cimport vector

cdef extern from "<rbdl/rbdl_math.h>" namespace "RigidBodyDynamics::Math":
    cdef cppclass VectorNd:
        VectorNd ()
        VectorNd (int dim)
        int rows()
        int cols()
        void resize (int)
        double& operator[](int)
        double* data()

    cdef cppclass Vector3d:
        Vector3d ()
        int rows()
        int cols()
        double& operator[](int)
        double* data()

    cdef cppclass SpatialVector:
        SpatialVector ()
        int rows()
        int cols()
        double& operator[](int)
        double* data()

    cdef cppclass Matrix3d:
        Matrix3d ()
        int rows()
        int cols()
        double& coeff "operator()"(int,int)
        double* data()

    cdef cppclass MatrixNd:
        MatrixNd ()
        MatrixNd (rows, cols)
        int rows()
        int cols()
        void resize (int,int)
        double& coeff "operator()"(int,int)
        double* data()
        void setZero()

    cdef cppclass SpatialMatrix:
        SpatialMatrix ()
        int rows()
        int cols()
        double& coeff "operator()"(int,int)
        double* data()

    cdef cppclass Matrix63:
        Matrix63 ()
        int rows()
        int cols()
        double& coeff "operator()"(int,int)
        double* data()

cdef extern from "<rbdl/rbdl_mathutils.h>" namespace "RigidBodyDynamics::Math":
    cdef VectorNd VectorFromPtr (unsigned int n, double *ptr)
    cdef MatrixNd MatrixFromPtr (unsigned int rows, unsigned int cols, double *ptr, bool row_major)

cdef extern from "<rbdl/SpatialAlgebraOperators.h>" namespace "RigidBodyDynamics::Math":
    cdef cppclass SpatialTransform:
        SpatialTransform()
        SpatialMatrix toMatrix()
        SpatialTransform inverse()
        SpatialTransform operator*(const SpatialTransform&)
        Matrix3d E
        Vector3d r

    cdef cppclass SpatialRigidBodyInertia:
        SpatialRigidBodyInertia()
        SpatialMatrix toMatrix()

        double m
        Vector3d h
        double Ixx, Iyx, Iyy, Izx, Izy, Izz

cdef extern from "<rbdl/Body.h>" namespace "RigidBodyDynamics":
    cdef cppclass Body:
        Body()
        Body(const double mass, const Vector3d &com, const Matrix3d &inertia)
        double mMass
        Vector3d mCenterOfMass
        Matrix3d mInertia
        bool mIsVirtual

    cdef cppclass FixedBody:
        FixedBody()
        double mMass
        Vector3d mCenterOfMass
        Matrix3d mInertia
        unsigned int mMovableParent
        SpatialTransform mParentTransform
        SpatialTransform mBaseTransform
        bool mIsVirtual


cdef extern from "<rbdl/Joint.h>" namespace "RigidBodyDynamics":
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

cdef extern from "<rbdl/Joint.h>" namespace "RigidBodyDynamics":
    cdef cppclass Joint:
        Joint()
        Joint(JointType joint_type)
        SpatialVector* mJointAxes
        JointType mJointType
        unsigned int mDoFCount
        unsigned int q_index

cdef extern from "<rbdl/Model.h>" namespace "RigidBodyDynamics":
    cdef cppclass Model:
        Model()
        unsigned int AddBody (const unsigned int parent_id,
                const SpatialTransform &joint_frame,
                const Joint &joint,
                const Body &body,
                string body_name
                )
#        vector[unsigned int] lambda
#        vector[unsigned int] lambda_q
#        vector[vector[unsigned int]] mu

        unsigned int dof_count
        unsigned int q_size
        unsigned int qdot_size
        unsigned int previously_added_body_id

        Vector3d gravity
        vector[SpatialVector] v
        vector[SpatialVector] a

        vector[Joint] mJoints
        vector[SpatialVector] S
        vector[SpatialTransform] X_J
        vector[SpatialVector] v_J 
        vector[SpatialVector] c_J 

        vector[unsigned int] mJointUpdateOrder

        vector[SpatialTransform] X_T

        vector[unsigned int] mFixedJointCount

        vector[Matrix63] multdof3_S
        vector[Matrix63] multdof3_U
        vector[Matrix63] multdof3_Dinv
        vector[Matrix63] multdof3_u
        vector[unsigned int] multdof3_w_index

        vector[SpatialVector] c
        vector[SpatialMatrix] IA
        vector[SpatialVector] pA 
        vector[SpatialVector] U 
        VectorNd d
        VectorNd u
        vector[SpatialVector] f 
        vector[SpatialRigidBodyInertia] I
        vector[SpatialRigidBodyInertia] Ic
        vector[SpatialVector] hc 

        vector[SpatialTransform] X_lambda
        vector[SpatialTransform] X_base

        vector[FixedBody] mFixedBodies
        unsigned int fixed_body_discriminator

        vector[Body] mBodies

cdef extern from "<rbdl/Kinematics.h>" namespace "RigidBodyDynamics":
    cdef void UpdateKinematics (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd &qddot)

cdef extern from "<rbdl/Dynamics.h>" namespace "RigidBodyDynamics":
    cdef void CompositeRigidBodyAlgorithm (Model& model,
            const VectorNd &q,
            MatrixNd &H,
            bool update_kinematics)

