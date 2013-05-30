/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#include <rbdl_config.h>

#ifdef RBDL_USE_SIMPLE_MATH
  #include "SimpleMath/SimpleMath.h"
	#include <vector>

	typedef SimpleMath::Fixed::Matrix<double, 3,1> Vector3_t;
	typedef SimpleMath::Fixed::Matrix<double, 3,3> Matrix3_t;

	typedef SimpleMath::Fixed::Matrix<double, 6,1> SpatialVector_t;
	typedef SimpleMath::Fixed::Matrix<double, 6,6> SpatialMatrix_t;

	typedef SimpleMath::Dynamic::Matrix<double> MatrixN_t;
	typedef SimpleMath::Dynamic::Matrix<double> VectorN_t;

#else
	#include "Eigen/Dense"
	#include "Eigen/StdVector"

	class Vector3_t : public Eigen::Vector3d
	{
	public:
		typedef Eigen::Vector3d Base;

		template<typename OtherDerived>
		Vector3_t(const Eigen::MatrixBase<OtherDerived>& other)
		: Eigen::Vector3d(other)
		{}

		template<typename OtherDerived>
		Vector3_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
		{
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE Vector3_t()
		{}

		EIGEN_STRONG_INLINE Vector3_t(
			const double& v0, const double& v1, const double& v2
		)
		{
			Base::_check_template_params();
			EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Matrix, 3)

			(*this) << v0, v1, v2;
		}

		void set(const double& v0, const double& v1, const double& v2)
		{
			Base::_check_template_params();
			EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Matrix, 3)

			(*this) << v0, v1, v2;
		}
	};

	class Matrix3_t : public Eigen::Matrix3d
	{
	public:
		typedef Eigen::Matrix3d Base;

		template<typename OtherDerived>
		Matrix3_t(const Eigen::MatrixBase<OtherDerived>& other)
		 : Eigen::Matrix3d(other)
		{}

		template<typename OtherDerived>
		Matrix3_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
		{
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE Matrix3_t()
		{}

		EIGEN_STRONG_INLINE Matrix3_t(
			const double& m00, const double& m01, const double& m02,
			const double& m10, const double& m11, const double& m12,
			const double& m20, const double& m21, const double& m22
		)
		{
			Base::_check_template_params();
			EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix, 3, 3)

			(*this)
				<< m00, m01, m02,
				   m10, m11, m12,
				   m20, m21, m22
			;
		}
	};

	typedef Eigen::VectorXd VectorN_t;
	typedef Eigen::MatrixXd MatrixN_t;

	class SpatialVector_t : public Eigen::Matrix<double, 6, 1>
	{
	public:
		typedef Eigen::Matrix<double, 6, 1> Base;

		template<typename OtherDerived>
		SpatialVector_t(const Eigen::MatrixBase<OtherDerived>& other)
		: Eigen::Matrix<double, 6, 1>(other)
		{}

		template<typename OtherDerived>
		SpatialVector_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
		{
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE SpatialVector_t()
		{}

		EIGEN_STRONG_INLINE SpatialVector_t(
			const double& v0, const double& v1, const double& v2,
			const double& v3, const double& v4, const double& v5
		)
		{
			Base::_check_template_params();
			EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Matrix, 6)

			(*this) << v0, v1, v2, v3, v4, v5;
		}

		void set(
			const double& v0, const double& v1, const double& v2,
			const double& v3, const double& v4, const double& v5
		)
		{
			Base::_check_template_params();
			EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Matrix, 6)

			(*this) << v0, v1, v2, v3, v4, v5;
		}
	};

	class SpatialMatrix_t : public Eigen::Matrix<double, 6, 6>
	{
	public:
		typedef Eigen::Matrix<double, 6, 6> Base;

		template<typename OtherDerived>
		SpatialMatrix_t(const Eigen::MatrixBase<OtherDerived>& other)
		: Eigen::Matrix<double, 6, 6>(other)
		{}

		template<typename OtherDerived>
		SpatialMatrix_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
		{
			this->Base::operator=(other);
			return *this;
		}

		EIGEN_STRONG_INLINE SpatialMatrix_t()
		{}

		EIGEN_STRONG_INLINE SpatialMatrix_t(
			const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m04, const Scalar& m05,
			const Scalar& m10, const Scalar& m11, const Scalar& m12, const Scalar& m13, const Scalar& m14, const Scalar& m15,
			const Scalar& m20, const Scalar& m21, const Scalar& m22, const Scalar& m23, const Scalar& m24, const Scalar& m25,
			const Scalar& m30, const Scalar& m31, const Scalar& m32, const Scalar& m33, const Scalar& m34, const Scalar& m35,
			const Scalar& m40, const Scalar& m41, const Scalar& m42, const Scalar& m43, const Scalar& m44, const Scalar& m45,
			const Scalar& m50, const Scalar& m51, const Scalar& m52, const Scalar& m53, const Scalar& m54, const Scalar& m55
		)
		{
			Base::_check_template_params();
			EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix, 6, 6)

			(*this)
				<< m00, m01, m02, m03, m04, m05
				 , m10, m11, m12, m13, m14, m15
				 , m20, m21, m22, m23, m24, m25
				 , m30, m31, m32, m33, m34, m35
				 , m40, m41, m42, m43, m44, m45
				 , m50, m51, m52, m53, m54, m55
			;
		}

		void set(
			const Scalar& m00, const Scalar& m01, const Scalar& m02, const Scalar& m03, const Scalar& m04, const Scalar& m05,
			const Scalar& m10, const Scalar& m11, const Scalar& m12, const Scalar& m13, const Scalar& m14, const Scalar& m15,
			const Scalar& m20, const Scalar& m21, const Scalar& m22, const Scalar& m23, const Scalar& m24, const Scalar& m25,
			const Scalar& m30, const Scalar& m31, const Scalar& m32, const Scalar& m33, const Scalar& m34, const Scalar& m35,
			const Scalar& m40, const Scalar& m41, const Scalar& m42, const Scalar& m43, const Scalar& m44, const Scalar& m45,
			const Scalar& m50, const Scalar& m51, const Scalar& m52, const Scalar& m53, const Scalar& m54, const Scalar& m55
		)
		{
			Base::_check_template_params();
			EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix, 6, 6)

			(*this)
				<< m00, m01, m02, m03, m04, m05
				 , m10, m11, m12, m13, m14, m15
				 , m20, m21, m22, m23, m24, m25
				 , m30, m31, m32, m33, m34, m35
				 , m40, m41, m42, m43, m44, m45
				 , m50, m51, m52, m53, m54, m55
			;
		}
	};
#endif

namespace RigidBodyDynamics {

/** \brief Math types such as vectors and matrices and utility functions. */
namespace Math {
	typedef Vector3_t Vector3d;
	typedef Matrix3_t Matrix3d;
	typedef SpatialVector_t SpatialVector;
	typedef SpatialMatrix_t SpatialMatrix;
	typedef VectorN_t VectorNd;
	typedef MatrixN_t MatrixNd;
} /* Math */

} /* RigidBodyDynamics */

#include "SpatialAlgebraOperators.h"

// If we use Eigen3 we have to create specializations of the STL
// std::vector such that the alignment is done properly.
#ifndef RBDL_USE_SIMPLE_MATH
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialVector)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialMatrix)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialTransform)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialRigidBodyInertia)
#endif

#endif /* _MATHWRAPPER_H */
