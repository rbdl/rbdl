/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MATRIXADDONS_H
#define _MATRIXADDONS_H

	  /** \brief Constructs an initialized 3x3 matrix with given coefficients */
    EIGEN_STRONG_INLINE Matrix(
        const Scalar& m00, const Scalar& m01, const Scalar& m02,
        const Scalar& m10, const Scalar& m11, const Scalar& m12,
        const Scalar& m20, const Scalar& m21, const Scalar& m22
        )
    {
      Base::_check_template_params();
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix, 3, 3)
      m_storage.data()[0] = m00;
      m_storage.data()[1] = m01;
      m_storage.data()[2] = m02;

      m_storage.data()[3] = m10;
      m_storage.data()[4] = m11;
      m_storage.data()[5] = m12;

      m_storage.data()[6] = m20;
      m_storage.data()[7] = m21;
      m_storage.data()[8] = m22;
    }

    /** \brief Constructs an initialized 6D vector with given coefficients */
    EIGEN_STRONG_INLINE Matrix(const Scalar& v0, const Scalar& v1, const Scalar& v2, const Scalar& v3, const Scalar& v4, const Scalar& v5)
    {
      Base::_check_template_params();
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Matrix, 6)
      m_storage.data()[0] = v0;
      m_storage.data()[1] = v1;
      m_storage.data()[2] = v2;
      m_storage.data()[3] = v3;
      m_storage.data()[4] = v4;
      m_storage.data()[5] = v5;
    }

    /** \brief Constructs an initialized 6x6 matrix with given coefficients */
    EIGEN_STRONG_INLINE Matrix(
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
      m_storage.data()[ 0] = m00;
      m_storage.data()[ 1] = m01;
      m_storage.data()[ 2] = m02;
      m_storage.data()[ 3] = m03;
      m_storage.data()[ 4] = m04;
      m_storage.data()[ 5] = m05;

      m_storage.data()[ 6] = m10;
      m_storage.data()[ 7] = m11;
      m_storage.data()[ 8] = m12;
      m_storage.data()[ 9] = m13;
      m_storage.data()[10] = m14;
      m_storage.data()[11] = m15;

      m_storage.data()[12] = m20;
      m_storage.data()[13] = m21;
      m_storage.data()[14] = m22;
      m_storage.data()[15] = m23;
      m_storage.data()[16] = m24;
      m_storage.data()[17] = m25;

      m_storage.data()[18] = m30;
      m_storage.data()[19] = m31;
      m_storage.data()[20] = m32;
      m_storage.data()[21] = m33;
      m_storage.data()[22] = m34;
      m_storage.data()[23] = m35;

      m_storage.data()[24] = m40;
      m_storage.data()[25] = m41;
      m_storage.data()[26] = m42;
      m_storage.data()[27] = m43;
      m_storage.data()[28] = m44;
      m_storage.data()[29] = m45;

      m_storage.data()[30] = m50;
      m_storage.data()[31] = m51;
      m_storage.data()[32] = m52;
      m_storage.data()[33] = m53;
      m_storage.data()[34] = m54;
      m_storage.data()[35] = m55;
    }

		void  set(
        const Scalar& v0, const Scalar& v1, const Scalar& v2
         )
    {
      Base::_check_template_params();
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix, 3, 1) 
      m_storage.data()[0] = v0;
      m_storage.data()[1] = v1;
      m_storage.data()[2] = v2;
    }

		void  set(
        const Scalar& m00, const Scalar& m01, const Scalar& m02,
        const Scalar& m10, const Scalar& m11, const Scalar& m12,
        const Scalar& m20, const Scalar& m21, const Scalar& m22
        )
    {
      Base::_check_template_params();
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix, 3, 3)
      m_storage.data()[0] = m00;
      m_storage.data()[1] = m01;
      m_storage.data()[2] = m02;

      m_storage.data()[3] = m10;
      m_storage.data()[4] = m11;
      m_storage.data()[5] = m12;

      m_storage.data()[6] = m20;
      m_storage.data()[7] = m21;
      m_storage.data()[8] = m22;
    }

    void set (const Scalar& v0, const Scalar& v1, const Scalar& v2, const Scalar& v3, const Scalar& v4, const Scalar& v5)
    {
      Base::_check_template_params();
      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Matrix, 6)
      m_storage.data()[0] = v0;
      m_storage.data()[1] = v1;
      m_storage.data()[2] = v2;
      m_storage.data()[3] = v3;
      m_storage.data()[4] = v4;
      m_storage.data()[5] = v5;
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
      m_storage.data()[ 0] = m00;
      m_storage.data()[ 1] = m01;
      m_storage.data()[ 2] = m02;
      m_storage.data()[ 3] = m03;
      m_storage.data()[ 4] = m04;
      m_storage.data()[ 5] = m05;

      m_storage.data()[ 6] = m10;
      m_storage.data()[ 7] = m11;
      m_storage.data()[ 8] = m12;
      m_storage.data()[ 9] = m13;
      m_storage.data()[10] = m14;
      m_storage.data()[11] = m15;

      m_storage.data()[12] = m20;
      m_storage.data()[13] = m21;
      m_storage.data()[14] = m22;
      m_storage.data()[15] = m23;
      m_storage.data()[16] = m24;
      m_storage.data()[17] = m25;

      m_storage.data()[18] = m30;
      m_storage.data()[19] = m31;
      m_storage.data()[20] = m32;
      m_storage.data()[21] = m33;
      m_storage.data()[22] = m34;
      m_storage.data()[23] = m35;

      m_storage.data()[24] = m40;
      m_storage.data()[25] = m41;
      m_storage.data()[26] = m42;
      m_storage.data()[27] = m43;
      m_storage.data()[28] = m44;
      m_storage.data()[29] = m45;

      m_storage.data()[30] = m50;
      m_storage.data()[31] = m51;
      m_storage.data()[32] = m52;
      m_storage.data()[33] = m53;
      m_storage.data()[34] = m54;
      m_storage.data()[35] = m55;
    }
#endif
