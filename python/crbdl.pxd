#cython: boundscheck=False

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

    cdef cppclass Quaternion:
        Quaternion ()
        int rows()
        int cols()
        double& operator[](int)
        double* data()
        Matrix3d toMatrix()
#        Quaternion fromMatrix (Matrix3d &mat)

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
        MatrixNd (int rows, int cols)
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

cdef extern from "<rbdl/Quaternion.h>" namespace "RigidBodyDynamics::Math::Quaternion":
    Quaternion fromMatrix(const Matrix3d &mat)

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
        JointTypeFloatingBase
        JointTypeFixed
        JointType1DoF
        JointType2DoF
        JointType3DoF
        JointType4DoF
        JointType5DoF
        JointType6DoF
        JointTypeCustom

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
        unsigned int AppendBody (const SpatialTransform &joint_frame,
                const Joint &joint,
                const Body &body,
                string body_name
                )
        unsigned int GetParentBodyId(
                unsigned int body_id)
        unsigned int GetBodyId(
                const char *body_name)
        string GetBodyName (
                unsigned int body_id)
        bool IsBodyId (
                unsigned int body_id)
        bool IsFixedBodyId (
                unsigned int body_id)
        Quaternion GetQuaternion (
                unsigned int body_id,
                const VectorNd &q)
        void SetQuaternion (
                unsigned int body_id,
                const Quaternion &quat,
                VectorNd &q)

        vector[unsigned int] _lambda
        vector[unsigned int] lambda_q
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

    cdef Vector3d CalcBodyToBaseCoordinates (Model& model,
            const VectorNd &q,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef Vector3d CalcBaseToBodyCoordinates (Model& model,
            const VectorNd &q,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef Matrix3d CalcBodyWorldOrientation (Model& model,
            const VectorNd &q,
            const unsigned int body_id,
            bool update_kinematics)

    cdef Vector3d CalcPointVelocity (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef Vector3d CalcPointAcceleration (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd &qddot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef SpatialVector CalcPointVelocity6D (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef SpatialVector CalcPointAcceleration6D (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd &qddot,
            const unsigned int body_id,
            const Vector3d &body_point_coordinates,
            bool update_kinematics)

    cdef Vector3d CalcAngularVelocityfromMatrix (
        const Matrix3d &RotMat
    )

cdef extern from "<rbdl/rbdl_utils.h>" namespace "RigidBodyDynamics::Utils":
    cdef void CalcCenterOfMass (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd *qdot,
            double &mass,
            Vector3d &com,
            Vector3d *com_velocity,
            Vector3d *com_acceleration,
            Vector3d *angular_momentum,
            Vector3d *change_of_angular_momentum,
            bool update_kinematics)

    cdef void CalcZeroMomentPoint (Model& model,
            const VectorNd &q,
            const VectorNd &qdot,
            const VectorNd &qddot,
            Vector3d* zmp,
            const Vector3d &normal,
            const Vector3d &point,
            bool update_kinematics)

cdef extern from "<rbdl/Constraints.h>" namespace "RigidBodyDynamics":
    cdef cppclass ConstraintSet:
        ConstraintSet()
        unsigned int AddContactConstraint (
                unsigned int body_id,
                const Vector3d &body_point,
                const Vector3d &world_normal,
                const char* constraint_name,
                double normal_acceleration)

        unsigned int AddLoopConstraint (
                unsigned int id_predecessor, 
                unsigned int id_successor,
                const SpatialTransform &X_predecessor,
                const SpatialTransform &X_successor,
                const SpatialVector &axis,
                bool enable_stabilization,
                double stabilization_param,
                const char *constraint_name)

        ConstraintSet Copy()
        # void SetSolver (Math::LinearSolver solver)
        bool Bind (const Model &model)

        size_t size()
        void clear()
        # Math::LinearSolver
        bool bound

        vector[string] name
        vector[unsigned int] body
        vector[Vector3d] point
        vector[Vector3d] normal

        VectorNd acceleration
        VectorNd force
        VectorNd impulse
        VectorNd v_plus

        MatrixNd H
        VectorNd C
        VectorNd gamma
        VectorNd G

        MatrixNd A
        VectorNd b
        VectorNd x

        MatrixNd GT_qr_Q
        MatrixNd Y
        MatrixNd Z
        VectorNd qddot_y
        VectorNd qddot_z

        MatrixNd K
        VectorNd a
        VectorNd QDDot_t
        VectorNd QDDot_0

        vector[SpatialVector] f_t
        vector[SpatialVector] f_ext_constraints
        vector[Vector3d] point_accel_0

        vector[SpatialVector] d_pA
        vector[SpatialVector] d_a
        VectorNd d_u

        vector[SpatialMatrix] d_IA
        vector[SpatialVector] d_U

        VectorNd d_d
        vector[Vector3d] d_multdof3_u

cdef extern from "rbdl_ptr_functions.h" namespace "RigidBodyDynamics":
    cdef void CalcPointJacobianPtr (Model& model,
            const double *q_ptr,
            unsigned int body_id,
            const Vector3d &point_position,
            double *G,
            bool update_kinematics)

    cdef void CalcPointJacobian6DPtr (Model &model,
            const double *q_ptr,
            unsigned int body_id,
            const Vector3d &point_position,
            double *G,
            bool update_kinematics)

    cdef void CalcBodySpatialJacobianPtr (
            Model &model,
            const double *q_ptr,
            unsigned int body_id,
            double *G,
            bool update_kinematics)

    cdef void InverseDynamicsPtr (
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            const double* qddot_ptr,
            double* tau_ptr,
            vector[SpatialVector] *f_ext
            )

    cdef void NonlinearEffectsPtr (
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            double* tau_ptr
            )

    cdef void CompositeRigidBodyAlgorithmPtr (Model& model,
            const double *q,
            double *H,
            bool update_kinematics)

    cdef void ForwardDynamicsPtr (
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            double* tau_ptr,
            const double* qddot_ptr,
            vector[SpatialVector] *f_ext
            )

    cdef void ForwardDynamicsConstraintsDirectPtr (
            Model &model,
            const double* q_ptr,
            const double* qdot_ptr,
            const double* tau_ptr,
            ConstraintSet &CS,
            double* qddot_ptr
            )

cdef extern from "rbdl_loadmodel.cc":
    cdef bool rbdl_loadmodel (
            const char* filename,
            Model* model,
            bool floating_base,
            bool verbose)
