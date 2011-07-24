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
#include <assert.h>

#include "compileassert.h"

/** \brief Namespace for a highly inefficient math library
 *
 */
namespace SimpleMath {

/** \brief Namespace for fixed size elements
 */
namespace Fixed {

// forward declaration
template <typename val_type, unsigned int nrows, unsigned int ncols>
class Matrix;

/** \brief Block class that can be used to access blocks of a spatial matrix
 *
 * This class is a proxy class and only contains data on where to find the
 * desired information.
 *
 */
template <typename val_type, unsigned int block_rows, unsigned int block_cols>
class Block {
	public:
	Block () :
		parent_nrows(0),
		parent_ncols(0),
		parent_row_index(0),
		parent_col_index(0),
		transposed(false),
		parent(NULL)
	{ }
	Block (const Block& other) :
		parent_nrows(other.parent_nrows),
		parent_ncols(other.parent_ncols),
		parent_row_index(other.parent_row_index),
		parent_col_index(other.parent_col_index),
		transposed(other.transposed),
		parent(other.parent)
	{ }

	Block (
			val_type *parent_data,
			unsigned int parent_row_start,
			unsigned int parent_col_start,
			unsigned int parent_num_rows,
			unsigned int parent_num_cols)
	{
		parent = parent_data;
		parent_row_index = parent_row_start;
		parent_col_index = parent_col_start;

		parent_nrows = parent_num_rows;
		parent_ncols = parent_num_cols;

		transposed = false;
	}

	/** This operater is only used to copy the data from other into this
	 *
	 */
	Block& operator=(const Block& other) {
		if (this != &other) {
			// copy the data, but we have to ensure, that the sizes match!

			unsigned int i, j;
			for (i = 0; i < block_rows; i++) {
				for (j = 0; j < block_cols; j++) {
					this->operator()(i,j) = other(i,j);
				}
			}
		}

		// copy data depending on other.transposed!

		return *this;
	}

	/** This operater is only used to copy the data from other into this
	 */
	template <unsigned int other_rows, unsigned int other_cols>
	Block& operator=(const Matrix<val_type, other_rows, other_cols>& data_in) {
		assert (parent != NULL);
		// copy the data, but we have to ensure, that the sizes match!
		COMPILE_ASSERT (block_rows == other_rows);
		COMPILE_ASSERT (block_cols == other_cols);

		if (!transposed) {
			for (unsigned int i = 0; i < block_rows; i++) {
				for (unsigned int j = 0; j < block_cols; j++) {
					parent[parent_nrows * (i + parent_row_index) + j + parent_col_index] = data_in(i,j);
				}
			}
		} else {
			for (unsigned int i = 0; i < block_rows; i++) {
				for (unsigned int j = 0; j < block_cols; j++) {
					parent[parent_nrows * (j + parent_row_index) + i + parent_col_index] = data_in(i,j);
				}
			}
		}

		return *this;
	}

	Block transpose() {
		assert (parent != NULL);
		Block result (*this);
		result.transposed = transposed ^ true;
		return result;
	}

	const val_type& operator() (const unsigned int i, const unsigned int j) const {
		assert (parent != NULL);
		assert (i < block_rows);
		assert (j < block_cols);

		if (!transposed)
			return parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];
	
		return parent[parent_nrows * (j + parent_row_index) + i + parent_col_index];
	}

	val_type& operator() (const unsigned int i, const unsigned int j) {
		assert (parent != NULL);
		assert (i < block_rows);
		assert (j < block_cols);

		if (!transposed)
			return parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];
	
		return parent[parent_nrows * (j + parent_row_index) + i + parent_col_index];
	}

	// casting operators
	template <unsigned int other_rows, unsigned int other_cols>
	operator Matrix<val_type, other_rows, other_cols>() {

		if (!transposed) {
			assert (block_rows == other_rows);
			assert (block_cols == other_cols);

			Matrix<val_type, other_rows, other_cols> result;
			for (unsigned int i = 0; i < block_rows; i++) 
				for (unsigned int j = 0; j < block_cols; j++)
					result(i,j) = parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];

			return result;
		} 

		assert (block_rows == other_cols);
		assert (block_cols == other_rows);

		Matrix<val_type, other_rows, other_cols> result;
		for (unsigned int i = 0; i < block_rows; i++) 
			for (unsigned int j = 0; j < block_cols; j++)
				result(j,i) = parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];

		return result;
	}

	unsigned int parent_nrows;
	unsigned int parent_ncols;
	unsigned int parent_row_index;
	unsigned int parent_col_index;
	bool transposed;

	val_type *parent;
};

/** \brief Matrix class for spatial matrices (both spatial transformations and inertias)
 */

template <typename val_type, unsigned int nrows, unsigned int ncols>
class Matrix {
	public:
		unsigned int rows() const {
			return nrows;
		}

		unsigned int cols() const {
			return ncols;
		}

		unsigned int size() const {
			return nrows * ncols;
		}

		typedef Matrix<val_type, nrows, ncols> matrix_type;

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
		const double& operator[](const unsigned int &index) const {
			assert (index	>= 0 && index < nrows * ncols);
			return mData[index];
		};
		double& operator[](const unsigned int &index) {
			assert (index	>= 0 && index < nrows * ncols);
			return mData[index];
		}

		const double& operator()(const unsigned int &row, const unsigned int &col) const {
			assert (row	>= 0 && row < nrows && col	>= 0 && col < ncols);
			return mData[row*ncols + col];
		};
		double& operator()(const unsigned int &row, const unsigned int &col) {
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

		val_type norm() {
			return sqrt(this->squaredNorm());
		}

		void normalize() {
			val_type length = this->norm();

			for (unsigned int i = 0; i < ncols * nrows; i++)
				mData[i] /= length;
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


		// Block accessing functions
		template <unsigned int blockrows, unsigned int blockcols>
		Block<val_type, blockrows, blockcols> block (unsigned int i, unsigned int j) const {
			COMPILE_ASSERT (nrows > blockrows);
			COMPILE_ASSERT (ncols > blockcols);
			return Block<val_type, blockrows, blockcols> (const_cast<double*> (this->mData), i, j, nrows, ncols);
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

		// Operators with other spatial matrices
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
		Matrix<val_type, nrows, other_cols> operator*(const Matrix<val_type, other_rows, other_cols> &matrix) {
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

	private:
		val_type mData[nrows * ncols];
};

template <unsigned int blockrows, unsigned int blockcols>
inline std::ostream& operator<<(std::ostream& output, const Block<double, blockrows, blockcols> &block) {
	output << std::endl;

	unsigned int i,j;
	for (i = 0; i < blockrows; i++) {
		output << "[ ";
		for (j = 0; j < blockcols; j++) {
			output << block(i,j) << " ";
		}
		output << "]" << std::endl;
	}

	return output;
}

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline Matrix<val_type, nrows, ncols> operator*(val_type scalar, const Matrix<val_type, nrows, ncols> &matrix) {
	Matrix<val_type, nrows, ncols> result (matrix);

	for (unsigned int i = 0; i < nrows * ncols; i++)
		result.data()[i] *= scalar;

	return result;
}

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline Matrix<val_type, nrows, ncols> operator*(const Matrix<val_type, nrows, ncols> &matrix, val_type scalar) {
	Matrix<val_type, nrows, ncols> result (matrix);

	for (unsigned int i = 0; i < nrows * ncols; i++)
		result.data()[i] *= scalar;

	return result;
}

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline std::ostream& operator<<(std::ostream& output, const Matrix<val_type, nrows, ncols> &matrix) {
	output << std::endl;
	for (unsigned int i = 0; i < nrows; i++) {
		output << "[ ";
		for (unsigned int j = 0; j < ncols; j++) {
			output << matrix(i,j) << " ";
		}
		output << "]" << std::endl;
	}
	return output;
}

}

}

#endif /* SIMPLEMATHFIXED_H */
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

#ifndef SIMPLEMATHDYNAMIC_H
#define SIMPLEMATHDYNAMIC_H

#include <sstream>
#include <cstdlib>
#include <assert.h>

#include "compileassert.h"

/** \brief Namespace for a highly inefficient math library
 *
 */
namespace SimpleMath {

/** \brief Namespace for elements of varying size.
 */
namespace Dynamic {

// forward declaration
template <typename val_type>
class Matrix;

/** \brief Block class that can be used to access blocks of a matrix.
 *
 * This class is a proxy class and only contains data on where to find the
 * desired information.
 *
 */
template <typename val_type, unsigned int block_rows, unsigned int block_cols>
class Block {
	public:
	Block () :
		parent_nrows(0),
		parent_ncols(0),
		parent_row_index(0),
		parent_col_index(0),
		transposed(false),
		parent(NULL)
	{ }
	Block (const Block& other) :
		parent_nrows(other.parent_nrows),
		parent_ncols(other.parent_ncols),
		parent_row_index(other.parent_row_index),
		parent_col_index(other.parent_col_index),
		transposed(other.transposed),
		parent(other.parent)
	{ }

	Block (
			val_type *parent_data,
			unsigned int parent_row_start,
			unsigned int parent_col_start,
			unsigned int parent_num_rows,
			unsigned int parent_num_cols)
	{
		parent = parent_data;
		parent_row_index = parent_row_start;
		parent_col_index = parent_col_start;

		parent_nrows = parent_num_rows;
		parent_ncols = parent_num_cols;

		transposed = false;
	}

	/** This operater is only used to copy the data from other into this
	 *
	 */
	Block& operator=(const Block& other) {
		if (this != &other) {
			// copy the data, but we have to ensure, that the sizes match!

			unsigned int i, j;
			for (i = 0; i < block_rows; i++) {
				for (j = 0; j < block_cols; j++) {
					this->operator()(i,j) = other(i,j);
				}
			}
		}

		// copy data depending on other.transposed!

		return *this;
	}

	/** This operater is only used to copy the data from other into this
	 */
	Block& operator=(const Matrix<val_type>& data_in) {
		assert (parent != NULL);
		// copy the data, but we have to ensure, that the sizes match!
		assert (block_rows == data_in.rows());
		assert (block_cols == data_in.cols());

		if (!transposed) {
			for (unsigned int i = 0; i < block_rows; i++) {
				for (unsigned int j = 0; j < block_cols; j++) {
					parent[parent_nrows * (i + parent_row_index) + j + parent_col_index] = data_in(i,j);
				}
			}
		} else {
			for (unsigned int i = 0; i < block_rows; i++) {
				for (unsigned int j = 0; j < block_cols; j++) {
					parent[parent_nrows * (j + parent_row_index) + i + parent_col_index] = data_in(i,j);
				}
			}
		}

		return *this;
	}

	Block transpose() {
		assert (parent != NULL);
		Block result (*this);
		result.transposed = transposed ^ true;
		return result;
	}

	const val_type& operator() (const unsigned int i, const unsigned int j) const {
		assert (parent != NULL);
		assert (i < block_rows);
		assert (j < block_cols);

		if (!transposed)
			return parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];
	
		return parent[parent_nrows * (j + parent_row_index) + i + parent_col_index];
	}

	val_type& operator() (const unsigned int i, const unsigned int j) {
		assert (parent != NULL);
		assert (i < block_rows);
		assert (j < block_cols);

		if (!transposed)
			return parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];
	
		return parent[parent_nrows * (j + parent_row_index) + i + parent_col_index];
	}

	// casting operators
	operator Matrix<val_type>() {
		if (!transposed) {
			Matrix<val_type> result (block_rows, block_cols);

			for (unsigned int i = 0; i < block_rows; i++) 
				for (unsigned int j = 0; j < block_cols; j++)
					result(i,j) = parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];

			return result;
		} 

		Matrix<val_type> result (block_cols, block_rows);

		for (unsigned int i = 0; i < block_rows; i++) 
			for (unsigned int j = 0; j < block_cols; j++)
				result(j,i) = parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];

		return result;
	}

	unsigned int parent_nrows;
	unsigned int parent_ncols;
	unsigned int parent_row_index;
	unsigned int parent_col_index;
	bool transposed;

	val_type *parent;
};

/** \brief Class for both matrices and vectors.
 */
template <typename val_type>
class Matrix {
	public:
		Matrix() :
			nrows (0),
			ncols (0),
			mData (NULL) {};
		Matrix(unsigned int rows) :
			nrows (rows),
			ncols (1) {
				mData = new val_type[rows];
			}
		Matrix(unsigned int rows, unsigned int cols) :
			nrows (rows),
			ncols (cols) {
				mData = new val_type[rows * cols];
			}
		unsigned int rows() const {
			return nrows;
		}

		unsigned int cols() const {
			return ncols;
		}

		unsigned int size() const {
			return nrows * ncols;
		}
		void resize (unsigned int rows, unsigned int cols=1) {
			if (nrows * ncols > 0 && mData != NULL) {
				delete[] mData;
			}

			nrows = rows;
			ncols = cols;

			mData = new val_type[nrows * ncols];
		}

		typedef Matrix<val_type> matrix_type;

		Matrix(const Matrix &matrix) :
			nrows (matrix.nrows),
			ncols (matrix.ncols) {
			unsigned int i;
		
			mData = new val_type[nrows * ncols];

			for (i = 0; i < nrows * ncols; i++)
				mData[i] = matrix.mData[i];
		}
		Matrix& operator=(const Matrix &matrix) {
			if (this != &matrix) {
				delete[] mData;

				nrows = matrix.nrows;
				ncols = matrix.ncols;

				mData = new val_type[nrows * ncols];

				unsigned int i;
				for (i = 0; i < nrows * ncols; i++)
					mData[i] = matrix.mData[i];
			}
			return *this;
		}

		~Matrix() {
			if (nrows * ncols > 0 && mData != NULL)
				delete[] mData;

			nrows = 0;
			ncols = 0;
			mData = NULL;
		};

		// comparison
		bool operator==(const Matrix &matrix) const {
			if (nrows != matrix.nrows || ncols != matrix.ncols)
				return false;

			for (unsigned int i = 0; i < nrows * ncols; i++) {
				if (mData[i] != matrix.mData[i])
					return false;
			}
			return true;
		}
		bool operator!=(const Matrix &matrix) const {
			if (nrows != matrix.nrows || ncols != matrix.ncols)
				return true;

			for (unsigned int i = 0; i < nrows * ncols; i++) {
				if (mData[i] != matrix.mData[i])
					return true;
			}
			return false;
		}

		// access operators
		const double& operator[](const unsigned int &index) const {
			assert (index	>= 0 && index < nrows * ncols);
			return mData[index];
		};
		double& operator[](const unsigned int &index) {
			assert (index	>= 0 && index < nrows * ncols);
			return mData[index];
		}

		const double& operator()(const unsigned int &row, const unsigned int &col) const {
			assert (row	>= 0 && row < nrows && col >= 0 && col < ncols);
			return mData[row*ncols + col];
		};
		double& operator()(const unsigned int &row, const unsigned int &col) {
			assert (row	>= 0 && row < nrows && col >= 0 && col < ncols);
			return mData[row*ncols + col];
		};
		
		void zero() {
			for (unsigned int i = 0; i < ncols * nrows; i++)
				mData[i] = 0.;
		}
		void setZero() {
			zero();
		}

		val_type norm() {
			return sqrt(this->squaredNorm());
		}

		void normalize() {
			val_type length = this->norm();

			for (unsigned int i = 0; i < ncols * nrows; i++)
				mData[i] /= length;
		}

		Matrix<val_type> cross(const Matrix<val_type> &other_vector) {
			assert (nrows * ncols == 3);
			assert (other_vector.nrows * other_vector.ncols == 3);

			Matrix<val_type> result (3, 1);
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

		static matrix_type Zero(int rows, int cols = 1) {
			matrix_type result (rows, cols);
			result.setZero();
			return result;
		}

		static matrix_type Constant (int rows, val_type value) {
			matrix_type result (rows, 1);
			unsigned int i;
			for (i = 0; i < result.size(); i++)
				result[i] = value;
			
			return result;
		}

		static matrix_type Constant (int rows, int cols, val_type value) {
			matrix_type result (rows, cols);
			unsigned int i;
			for (i = 0; i < result.size(); i++)
				result[i] = value;
			
			return result;
		}

		void identity() {
			assert (nrows == ncols);

			setZero();
			for (unsigned int i = 0; i < ncols; i++)
				mData[i * ncols + i] = 1.;
		}

		void random() {
			for (unsigned int i = 0; i < nrows * ncols; i++) 
				mData[i] = static_cast<val_type> (rand()) / static_cast<val_type> (RAND_MAX);
		}

		val_type squaredNorm() const {
			assert (ncols == 1 || nrows == 1);
			val_type result = 0;

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result += mData[i] * mData[i];

			return result;
		}

		val_type dot(const matrix_type &matrix) const {
			assert (ncols == 1 || nrows == 1);
			val_type result = 0;

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result += mData[i] * matrix[i];

			return result;
		}


		// Block accessing functions
		template <unsigned int blockrows, unsigned int blockcols>
		Block<val_type, blockrows, blockcols> block (unsigned int i, unsigned int j) const {
			assert (nrows > blockrows);
			assert (ncols > blockcols);
			return Block<val_type, blockrows, blockcols> (const_cast<double*> (this->mData), i, j, nrows, ncols);
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

		// Operators with other spatial matrices
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

		Matrix<val_type> operator*(const Matrix<val_type> &other_matrix) const {
			assert (ncols == other_matrix.nrows);

			Matrix<val_type> result(nrows, other_matrix.ncols);
			
			result.setZero();

			unsigned int i,j, k;
			for (i = 0; i < nrows; i++) {
				for (j = 0; j < other_matrix.cols(); j++) {
					for (k = 0; k < other_matrix.rows(); k++) {
						result(i,j) += mData[i * ncols + k] * other_matrix(k,j);
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
		Matrix<val_type> transpose() const {
			Matrix<val_type> result(ncols, nrows);
	
			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					result(j,i) = mData[i * ncols + j];
				}
			}

			return result;
		}

		operator val_type() {
			assert (nrows == 1);
			assert (nrows == 1);

			return mData[0];
		}

	private:
		unsigned int nrows;
		unsigned int ncols;

		val_type* mData;
};

template <unsigned int blockrows, unsigned int blockcols>
inline std::ostream& operator<<(std::ostream& output, const Block<double, blockrows, blockcols> &block) {
	output << std::endl;

	unsigned int i,j;
	for (i = 0; i < blockrows; i++) {
		for (j = 0; j < blockcols; j++) {
			output.width(12);
			output << block(i,j) << " ";
		}
	}

	return output;
}

template <typename val_type>
inline Matrix<val_type> operator*(val_type scalar, const Matrix<val_type> &matrix) {
	Matrix<val_type> result (matrix);

	for (unsigned int i = 0; i < matrix.rows() * matrix.cols(); i++)
		result.data()[i] *= scalar;

	return result;
}

template <typename val_type>
inline Matrix<val_type> operator*(const Matrix<val_type> &matrix, val_type scalar) {
	Matrix<val_type> result (matrix);

	for (unsigned int i = 0; i < matrix.rows() * matrix.cols(); i++)
		result.data()[i] *= scalar;

	return result;
}

template <typename val_type>
inline std::ostream& operator<<(std::ostream& output, const Matrix<val_type> &matrix) {
	output << std::endl;
	for (unsigned int i = 0; i < matrix.rows(); i++) {
		for (unsigned int j = 0; j < matrix.cols(); j++) {
			output.width(12);
			output << matrix(i,j) << " ";
		}
		output << std::endl;
	}
	return output;
}

}

}

#endif /* SIMPLEMATHDYNAMIC_H */
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

#ifndef SIMPLEMATHMIXED_H
#define SIMPLEMATHMIXED_H

#include <sstream>
#include <cstdlib>
#include <assert.h>

#include "compileassert.h"

/** \brief Namespace for a highly inefficient math library
 *
 */
namespace SimpleMath {

// multiplication
template <typename val_type, unsigned int nrows, unsigned int ncols>
inline Dynamic::Matrix<val_type> operator*(
		const Fixed::Matrix<val_type, nrows, ncols> &matrix_a,
		const Dynamic::Matrix<val_type> &matrix_b) {
	assert (matrix_a.cols() == matrix_b.rows());

	Dynamic::Matrix<val_type> result (nrows, matrix_b.cols());

	result.setZero();

	unsigned int i,j, k;
	for (i = 0; i < nrows; i++) {
		for (j = 0; j < matrix_b.cols(); j++) {
			for (k = 0; k < matrix_b.rows(); k++) {
				result(i,j) += matrix_a(i,k) * matrix_b(k,j);
			}
		}
	}

	return result;
}

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline Dynamic::Matrix<val_type> operator*(
		const Dynamic::Matrix<val_type> &matrix_a,
		const Fixed::Matrix<val_type, nrows, ncols> &matrix_b) {
	assert (matrix_a.cols() == matrix_b.rows());
	
	Dynamic::Matrix<val_type> result (matrix_a.rows(), ncols);

	result.setZero();

	unsigned int i,j, k;
	for (i = 0; i < matrix_a.rows(); i++) {
		for (j = 0; j < matrix_b.cols(); j++) {
			for (k = 0; k < matrix_b.rows(); k++) {
				result(i,j) += matrix_a(i,k) * matrix_b(k,j);
			}
		}
	}

	return result;
}

// equality
template <typename val_type, unsigned int nrows, unsigned int ncols>
inline bool operator==(
		const Fixed::Matrix<val_type, nrows, ncols> &matrix_a,
		const Dynamic::Matrix<val_type> &matrix_b) {
	assert (nrows == matrix_a.rows());
	assert (ncols == matrix_a.cols());

	unsigned int i;
	for (i = 0; i < matrix_a.size(); i++) {
		if (matrix_a[i] != matrix_b[i])
			return false;
	}

	return true;
}

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline bool operator==(
		const Dynamic::Matrix<val_type> &matrix_b,
		const Fixed::Matrix<val_type, nrows, ncols> &matrix_a) {
	assert (nrows == matrix_a.rows());
	assert (ncols == matrix_a.cols());

	unsigned int i;
	for (i = 0; i < matrix_a.size(); i++) {
		if (matrix_a[i] != matrix_b[i])
			return false;
	}

	return true;
}

}

#endif /* SIMPLEMATHMIXED_H */
