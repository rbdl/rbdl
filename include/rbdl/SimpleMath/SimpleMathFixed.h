/**
 * This is a highly inefficient math library. It was conceived by Martin
 * Felis <martin.felis@iwr.uni-heidelberg.de> while he was compiling code
 * that uses a highly efficient math library.
 *
 * It is intended to be used as a fast compiling substitute for the
 * blazingly fast Eigen3 library and tries to mimic its API to a certain
 * extend.
 *
 * Feel free to use it wherever you like. However, no guarantees are given
 * that this code does what it says it would.
 */

#ifndef SIMPLEMATHFIXED_H
#define SIMPLEMATHFIXED_H

#include <sstream>
#include <cstdlib>
#include <cmath>
#include <assert.h>

#include "compileassert.h"
#include "SimpleMathBlock.h"

/** \brief Namespace for a highly inefficient math library
 *
 */
namespace SimpleMath {

template <typename matrix_type>
class HouseholderQR;

template <typename matrix_type>
class ColPivHouseholderQR;

template <typename matrix_type>
class CommaInitializer;

namespace Dynamic {
template <typename val_type> class Matrix;
}

/** \brief Namespace for fixed size elements
 */
namespace Fixed {

// forward declaration
template <typename val_type, unsigned int nrows, unsigned int ncols>
class Matrix;

/** \brief Fixed size matrix class
 */
template <typename val_type, unsigned int nrows, unsigned int ncols>
class Matrix {
	public:
		typedef Matrix<val_type, nrows, ncols> matrix_type;
		typedef val_type value_type;

		unsigned int rows() const {
			return nrows;
		}

		unsigned int cols() const {
			return ncols;
		}

		unsigned int size() const {
			return nrows * ncols;
		}

		Matrix() {};
		Matrix(const Matrix &matrix) {
			unsigned int i;
			for (i = 0; i < nrows * ncols; i++)
				mData[i] = matrix.mData[i];
		}
		Matrix& operator=(const Matrix &matrix) {
			if (this != &matrix) {
				unsigned int i;
				for (i = 0; i < nrows * ncols; i++)
					mData[i] = matrix.mData[i];
			}
			return *this;
		}

		// conversion different val_types

		template <typename other_matrix_type>
		Matrix (const Block<other_matrix_type, val_type> &block) {
			assert (nrows == block.rows());
			assert (ncols == block.cols());

			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					(*this)(i,j) = static_cast<val_type>(block(i,j));
				}
			}
		}
		template <typename other_matrix_type>
		Matrix& operator= (const Block<other_matrix_type, val_type> &block) {
			assert (nrows == block.rows());
			assert (ncols == block.cols());

			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					(*this)(i,j) = static_cast<value_type>(block(i,j));
				}
			}

			return *this;
		}

		template <typename other_type>
		Matrix (const Matrix<other_type, nrows, ncols> &matrix) {
			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					(*this)(i,j) = static_cast<val_type>(matrix(i,j));
				}
			}
		}

		template <typename other_type>
		Matrix& operator=(const Matrix<other_type, nrows, ncols> &matrix) {
			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					(*this)(i,j) = static_cast<val_type>(matrix(i,j));
				}
			}

			return *this;
		}
		
		CommaInitializer<matrix_type> operator<< (const val_type& value) {
			return CommaInitializer<matrix_type> (*this, value);
		}

		// conversion Dynamic->Fixed
		Matrix(const Dynamic::Matrix<val_type> &dynamic_matrix);
		Matrix& operator=(const Dynamic::Matrix<val_type> &dynamic_matrix);

	 	~Matrix() {};

		Matrix (
				const val_type &v00, const val_type &v01, const val_type &v02
				) {
			assert (nrows == 3);
			assert (ncols == 1);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
		}

		void set(
				const val_type &v00, const val_type &v01, const val_type &v02
				) {
			COMPILE_ASSERT (nrows * ncols == 3);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
		}

		Matrix (
				const val_type &v00, const val_type &v01, const val_type &v02,
				const val_type &v10, const val_type &v11, const val_type &v12,
				const val_type &v20, const val_type &v21, const val_type &v22
				) {
			COMPILE_ASSERT (nrows == 3);
			COMPILE_ASSERT (ncols == 3);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;

			mData[1 * 3 + 0] = v10;
			mData[1 * 3 + 1] = v11;
			mData[1 * 3 + 2] = v12;
			
			mData[2 * 3 + 0] = v20;
			mData[2 * 3 + 1] = v21;
			mData[2 * 3 + 2] = v22;
		}

		void set(
				const val_type v00, const val_type v01, const val_type v02,
				const val_type v10, const val_type v11, const val_type v12,
				const val_type v20, const val_type v21, const val_type v22
				) {
			COMPILE_ASSERT (nrows == 3);
			COMPILE_ASSERT (ncols == 3);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;

			mData[1 * 3 + 0] = v10;
			mData[1 * 3 + 1] = v11;
			mData[1 * 3 + 2] = v12;

			mData[2 * 3 + 0] = v20;
			mData[2 * 3 + 1] = v21;
			mData[2 * 3 + 2] = v22;
		}

		Matrix (
				const val_type &v00, const val_type &v01, const val_type &v02, const val_type &v03
				) {
			assert (nrows == 4);
			assert (ncols == 1);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;
		}

		void set(
				const val_type &v00, const val_type &v01, const val_type &v02, const val_type &v03
				) {
			COMPILE_ASSERT (nrows * ncols == 4);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;
		}

		Matrix (
				const val_type &v00, const val_type &v01, const val_type &v02, const val_type &v03,
				const val_type &v10, const val_type &v11, const val_type &v12, const val_type &v13,
				const val_type &v20, const val_type &v21, const val_type &v22, const val_type &v23,
				const val_type &v30, const val_type &v31, const val_type &v32, const val_type &v33
				) {
			COMPILE_ASSERT (nrows == 4);
			COMPILE_ASSERT (ncols == 4);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;

			mData[1 * 4 + 0] = v10;
			mData[1 * 4 + 1] = v11;
			mData[1 * 4 + 2] = v12;
			mData[1 * 4 + 3] = v13;
			
			mData[2 * 4 + 0] = v20;
			mData[2 * 4 + 1] = v21;
			mData[2 * 4 + 2] = v22;
			mData[2 * 4 + 3] = v23;

			mData[3 * 4 + 0] = v30;
			mData[3 * 4 + 1] = v31;
			mData[3 * 4 + 2] = v32;
			mData[3 * 4 + 3] = v33;
		}

		void set(
				const val_type &v00, const val_type &v01, const val_type &v02, const val_type &v03,
				const val_type &v10, const val_type &v11, const val_type &v12, const val_type &v13,
				const val_type &v20, const val_type &v21, const val_type &v22, const val_type &v23,
				const val_type &v30, const val_type &v31, const val_type &v32, const val_type &v33
				) {
			COMPILE_ASSERT (nrows == 4);
			COMPILE_ASSERT (ncols == 4);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;

			mData[1 * 4 + 0] = v10;
			mData[1 * 4 + 1] = v11;
			mData[1 * 4 + 2] = v12;
			mData[1 * 4 + 3] = v13;
			
			mData[2 * 4 + 0] = v20;
			mData[2 * 4 + 1] = v21;
			mData[2 * 4 + 2] = v22;
			mData[2 * 4 + 3] = v23;

			mData[3 * 4 + 0] = v30;
			mData[3 * 4 + 1] = v31;
			mData[3 * 4 + 2] = v32;
			mData[3 * 4 + 3] = v33;
		}

		Matrix (
				const val_type &v00, const val_type &v01, const val_type &v02,
				const val_type &v03, const val_type &v04, const val_type &v05
				) {
			COMPILE_ASSERT (nrows == 6);
			COMPILE_ASSERT (ncols == 1);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;
			mData[4] = v04;
			mData[5] = v05;
		}

		void set(
				const val_type &v00, const val_type &v01, const val_type &v02,
				const val_type &v03, const val_type &v04, const val_type &v05
				) {
			COMPILE_ASSERT (nrows * ncols == 6);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;
			mData[4] = v04;
			mData[5] = v05;
		}

		Matrix (
				const val_type &v00, const val_type &v01, const val_type &v02,
				const val_type &v03, const val_type &v04, const val_type &v05,

				const val_type &v10, const val_type &v11, const val_type &v12,
				const val_type &v13, const val_type &v14, const val_type &v15,

				const val_type &v20, const val_type &v21, const val_type &v22,
				const val_type &v23, const val_type &v24, const val_type &v25,

				const val_type &v30, const val_type &v31, const val_type &v32,
				const val_type &v33, const val_type &v34, const val_type &v35,

				const val_type &v40, const val_type &v41, const val_type &v42,
				const val_type &v43, const val_type &v44, const val_type &v45,

				const val_type &v50, const val_type &v51, const val_type &v52,
				const val_type &v53, const val_type &v54, const val_type &v55
				) {
			COMPILE_ASSERT (nrows == 6);
			COMPILE_ASSERT (ncols == 6);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;
			mData[4] = v04;
			mData[5] = v05;

			mData[6 + 0] = v10;
			mData[6 + 1] = v11;
			mData[6 + 2] = v12;
			mData[6 + 3] = v13;
			mData[6 + 4] = v14;
			mData[6 + 5] = v15;

			mData[12 + 0] = v20;
			mData[12 + 1] = v21;
			mData[12 + 2] = v22;
			mData[12 + 3] = v23;
			mData[12 + 4] = v24;
			mData[12 + 5] = v25;

			mData[18 + 0] = v30;
			mData[18 + 1] = v31;
			mData[18 + 2] = v32;
			mData[18 + 3] = v33;
			mData[18 + 4] = v34;
			mData[18 + 5] = v35;

			mData[24 + 0] = v40;
			mData[24 + 1] = v41;
			mData[24 + 2] = v42;
			mData[24 + 3] = v43;
			mData[24 + 4] = v44;
			mData[24 + 5] = v45;

			mData[30 + 0] = v50;
			mData[30 + 1] = v51;
			mData[30 + 2] = v52;
			mData[30 + 3] = v53;
			mData[30 + 4] = v54;
			mData[30 + 5] = v55;
		};

		void set(
				const val_type v00, const val_type v01, const val_type v02,
				const val_type v03, const val_type v04, const val_type v05,

				const val_type v10, const val_type v11, const val_type v12,
				const val_type v13, const val_type v14, const val_type v15,

				const val_type v20, const val_type v21, const val_type v22,
				const val_type v23, const val_type v24, const val_type v25,

				const val_type v30, const val_type v31, const val_type v32,
				const val_type v33, const val_type v34, const val_type v35,

				const val_type v40, const val_type v41, const val_type v42,
				const val_type v43, const val_type v44, const val_type v45,

				const val_type v50, const val_type v51, const val_type v52,
				const val_type v53, const val_type v54, const val_type v55
				) {
			COMPILE_ASSERT (nrows == 6);
			COMPILE_ASSERT (ncols == 6);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
			mData[3] = v03;
			mData[4] = v04;
			mData[5] = v05;

			mData[6 + 0] = v10;
			mData[6 + 1] = v11;
			mData[6 + 2] = v12;
			mData[6 + 3] = v13;
			mData[6 + 4] = v14;
			mData[6 + 5] = v15;

			mData[12 + 0] = v20;
			mData[12 + 1] = v21;
			mData[12 + 2] = v22;
			mData[12 + 3] = v23;
			mData[12 + 4] = v24;
			mData[12 + 5] = v25;

			mData[18 + 0] = v30;
			mData[18 + 1] = v31;
			mData[18 + 2] = v32;
			mData[18 + 3] = v33;
			mData[18 + 4] = v34;
			mData[18 + 5] = v35;

			mData[24 + 0] = v40;
			mData[24 + 1] = v41;
			mData[24 + 2] = v42;
			mData[24 + 3] = v43;
			mData[24 + 4] = v44;
			mData[24 + 5] = v45;

			mData[30 + 0] = v50;
			mData[30 + 1] = v51;
			mData[30 + 2] = v52;
			mData[30 + 3] = v53;
			mData[30 + 4] = v54;
			mData[30 + 5] = v55;
		}
		
		// comparison
		bool operator==(const Matrix &matrix) const {
			for (unsigned int i = 0; i < nrows * ncols; i++) {
				if (mData[i] != matrix.mData[i])
					return false;
			}
			return true;
		}
		bool operator!=(const Matrix &matrix) const {
			for (unsigned int i = 0; i < nrows * ncols; i++) {
				if (mData[i] != matrix.mData[i])
					return true;
			}
			return false;
		}

		// access operators
		const val_type& operator[](const unsigned int &index) const {
			assert (index	>= 0 && index < nrows * ncols);
			return mData[index];
		};
		val_type& operator[](const unsigned int &index) {
			assert (index	>= 0 && index < nrows * ncols);
			return mData[index];
		}

		const val_type& operator()(const unsigned int &row, const unsigned int &col) const {
			assert (row	>= 0 && row < nrows && col	>= 0 && col < ncols);
			return mData[row*ncols + col];
		};
		val_type& operator()(const unsigned int &row, const unsigned int &col) {
			assert (row	>= 0 && row < nrows && col	>= 0 && col < ncols);
			return mData[row*ncols + col];
		};
		
		void zero() {
			for (unsigned int i = 0; i < ncols * nrows; i++)
				mData[i] = 0.;
		}
		void setZero() {
			zero();
		}

		val_type norm() const {
			return sqrt(this->squaredNorm());
		}

		matrix_type normalize() {
			val_type length = this->norm();

			for (unsigned int i = 0; i < ncols * nrows; i++)
				mData[i] /= length;

			return *this;
		}

		matrix_type normalized() {
			return matrix_type (*this) / this->norm();
		}

		Matrix<val_type, 3, 1> cross(const Matrix<val_type, 3, 1> &other_vector) {
			COMPILE_ASSERT (nrows * ncols == 3);

			Matrix<val_type, 3, 1> result;
			result[0] = mData[1] * other_vector[2] - mData[2] * other_vector[1];
			result[1] = mData[2] * other_vector[0] - mData[0] * other_vector[2];
			result[2] = mData[0] * other_vector[1] - mData[1] * other_vector[0];

			return result;
		}

		static matrix_type Zero() {
			matrix_type result;
			result.setZero();
			return result;
		}

		static matrix_type Zero(int ignore_me) {
			matrix_type result;
			result.setZero();
			return result;
		}

		static matrix_type Zero(int ignore_me, int ignore_me_too) {
			matrix_type result;
			result.setZero();
			return result;
		}

		static matrix_type Identity() {
			matrix_type result;
			result.identity();
			return result;
		}

		static matrix_type Identity(int ignore_me, int ignore_me_too) {
			matrix_type result;
			result.identity();
			return result;
		}

		void identity() {
			COMPILE_ASSERT (nrows == ncols);

			setZero();
			for (unsigned int i = 0; i < ncols; i++)
				mData[i * ncols + i] = 1.;
		}

		void random() {
			for (unsigned int i = 0; i < nrows * ncols; i++) 
				mData[i] = static_cast<val_type> (rand()) / static_cast<val_type> (RAND_MAX);
		}

		val_type squaredNorm() const {
			COMPILE_ASSERT (ncols == 1 || nrows == 1);
			val_type result = 0;

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result += mData[i] * mData[i];

			return result;
		}

		val_type dot(const matrix_type &matrix) const {
			COMPILE_ASSERT (ncols == 1 || nrows == 1);
			val_type result = 0;

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result += mData[i] * matrix[i];

			return result;
		}

		// Blocks using block(i,j,r,c) syntax
		Block<matrix_type, val_type>
			block (unsigned int row_start, unsigned int col_start, unsigned int row_count, unsigned int col_count) {
				return Block<matrix_type, val_type>(*this, row_start, col_start, row_count, col_count);
			}

		const Block<matrix_type, val_type>
			block (unsigned int row_start, unsigned int col_start, unsigned int row_count, unsigned int col_count) const {
				return Block<matrix_type, val_type>(*this, row_start, col_start, row_count, col_count);
			}

		// Blocks using block<r,c>(i,j) syntax
		template <unsigned int block_row_count, unsigned int block_col_count>
		Block<matrix_type, val_type>
			block (unsigned int row_start, unsigned int col_start) {
				return Block<matrix_type, val_type>(*this, row_start, col_start, block_row_count, block_col_count);
			}

		template <unsigned int block_row_count, unsigned int block_col_count>
		const Block<matrix_type, val_type>
			block (unsigned int row_start, unsigned int col_start) const {
				return Block<matrix_type, val_type>(*this, row_start, col_start, block_row_count, block_col_count);
			}

		// Operators with scalars
		void operator*=(const val_type &scalar) {
			for (unsigned int i = 0; i < nrows * ncols; i++)
				mData[i] *= scalar;
		};
		void operator/=(const val_type &scalar) {
			for (unsigned int i = 0; i < nrows * ncols; i++)
				mData[i] /= scalar;
		}
		Matrix operator/(const val_type& scalar) const {
			matrix_type result (*this);

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result[i] /= scalar;

			return result;
		}

		// Operators with other matrices
		Matrix operator+(const Matrix &matrix) const {
			matrix_type result (*this);

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result[i] += matrix[i];

			return result;
		}
		void operator+=(const matrix_type &matrix) {
			for (unsigned int i = 0; i < nrows * ncols; i++)
				mData[i] += matrix.mData[i];
		}
		Matrix operator-(const Matrix &matrix) const {
			matrix_type result (*this);

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result[i] -= matrix[i];

			return result;
		}
		void operator-=(const Matrix &matrix) {
			for (unsigned int i = 0; i < nrows * ncols; i++)
				mData[i] -= matrix.mData[i];
		}

		template <unsigned int other_rows, unsigned int other_cols>
		Matrix<val_type, nrows, other_cols> operator*(const Matrix<val_type, other_rows, other_cols> &matrix) const {
			COMPILE_ASSERT (ncols == other_rows);

			Matrix<val_type, nrows, other_cols> result;
			
			result.setZero();

			unsigned int i,j, k;
			for (i = 0; i < nrows; i++) {
				for (j = 0; j < other_cols; j++) {
					for (k = 0; k < other_rows; k++) {
						result(i,j) += mData[i * ncols + k] * matrix(k,j);
					}
				}
			}
			
			return result;
		}

		// multiplication with dynamic sized matrix
		template <typename other_type>
		Dynamic::Matrix<val_type> operator*(const Dynamic::Matrix<other_type> &other_matrix) {
			assert (ncols == other_matrix.rows());

			Dynamic::Matrix<val_type> result(nrows, other_matrix.cols());
			
			result.setZero();

			unsigned int i,j, k;
			for (i = 0; i < nrows; i++) {
				for (j = 0; j < other_matrix.cols(); j++) {
					for (k = 0; k < other_matrix.rows(); k++) {
						result(i,j) += mData[i * ncols + k] * static_cast<val_type>(other_matrix(k,j));
					}
				}
			}
			
			return result;
		}


		void operator*=(const Matrix &matrix) {
			matrix_type temp (*this);
			*this = temp * matrix;
		}

		// Special operators
		val_type *data(){
			return mData;
		}

		// regular transpose of a 6 dimensional matrix
		Matrix<val_type, ncols, nrows> transpose() const {
			Matrix<val_type, ncols, nrows> result;
	
			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					result(j,i) = mData[i * ncols + j];
				}
			}

			return result;
		}

		operator val_type() {
			COMPILE_ASSERT (nrows == 1);
			COMPILE_ASSERT (nrows == 1);

			return mData[0];
		}

		Matrix operator-() const {
			return *this * -1.;
		}

		Matrix inverse() const {
			return colPivHouseholderQr().inverse();
		}

		const HouseholderQR<matrix_type> householderQr() const {
			return HouseholderQR<matrix_type>(*this);
		}
		const ColPivHouseholderQR<matrix_type> colPivHouseholderQr() const {
			return ColPivHouseholderQR<matrix_type>(*this);
		}

	private:
		val_type mData[nrows * ncols];
};

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline Matrix<val_type, nrows, ncols> operator*(val_type scalar, const Matrix<val_type, nrows, ncols> &matrix) {
	Matrix<val_type, nrows, ncols> result (matrix);

	for (unsigned int i = 0; i < nrows * ncols; i++)
		result.data()[i] *= scalar;

	return result;
}

template <typename val_type, typename other_type, unsigned int nrows, unsigned int ncols>
inline Matrix<val_type, nrows, ncols> operator*(const Matrix<val_type, nrows, ncols> &matrix, other_type scalar) {
	Matrix<val_type, nrows, ncols> result (matrix);

	for (unsigned int i = 0; i < nrows * ncols; i++)
		result.data()[i] *= static_cast<val_type> (scalar);

	return result;
}

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline std::ostream& operator<<(std::ostream& output, const Matrix<val_type, nrows, ncols> &matrix) {
	for (unsigned int i = 0; i < nrows; i++) {
		output << "[ ";
		for (unsigned int j = 0; j < ncols; j++) {
			output << matrix(i,j);

			if (j < ncols - 1)
				output << ", ";
		}
		output << " ]";
		
		if (nrows > 1 && i < nrows - 1)
			output << std::endl;
	}
	return output;
}

}

}

#endif /* SIMPLEMATHFIXED_H */
