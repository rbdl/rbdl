from libcpp.string cimport string
from libcpp cimport bool

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
        int rows()
        int cols()
        void resize (int,int)
        double& coeff "operator()"(int,int)
        double* data()

    cdef cppclass SpatialMatrix:
        SpatialMatrix ()
        int rows()
        int cols()
        double& coeff "operator()"(int,int)
        double* data()

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

cdef extern from "<rbdl/Body.h>" namespace "RigidBodyDynamics":
    cdef cppclass Body:
        Body()

cdef extern from "<rbdl/Model.h>" namespace "RigidBodyDynamics":
    cdef cppclass Model:
        Model()

