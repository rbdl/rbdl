#ifndef SIMPLEMATHFIXED_H
#define SIMPLEMATHFIXED_H

#include <sstream>
#include <cstdlib>
#include <assert.h>

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
template <typename val_type, unsigned int BlockRows, unsigned int BlockCols>
class Block {
	public:
	Block () :
		nrows(BlockRows),
		ncols(BlockCols),
		parent_nrows(0),
		parent_ncols(0),
		parent_row_index(0),
		parent_col_index(0),
		transposed(false),
		parent(NULL)
	{ }
	Block (const Block& other) :
		nrows(BlockRows),
		ncols(BlockCols),
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
			unsigned int parent_num_cols) :
		nrows(BlockRows),
		ncols(BlockCols)
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
			assert (nrows == other.nrows);
			assert (ncols == other.ncols);

			unsigned int i, j;
			for (i = 0; i < nrows; i++) {
				for (j = 0; j < ncols; j++) {
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
		assert (nrows == other_rows);
		assert (ncols == other_cols);

		if (!transposed) {
			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					parent[parent_nrows * (i + parent_row_index) + j + parent_col_index] = data_in(i,j);
				}
			}
		} else {
			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
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
		assert (i < nrows);
		assert (j < ncols);

		if (!transposed)
			return parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];
	
		return parent[parent_nrows * (j + parent_row_index) + i + parent_col_index];
	}

	val_type& operator() (const unsigned int i, const unsigned int j) {
		assert (parent != NULL);
		assert (i < nrows);
		assert (j < ncols);

		if (!transposed)
			return parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];
	
		return parent[parent_nrows * (j + parent_row_index) + i + parent_col_index];
	}

	// casting operators
	template <unsigned int other_rows, unsigned int other_cols>
	operator Matrix<val_type, other_rows, other_cols>() {

		if (!transposed) {
			assert (nrows == other_rows);
			assert (ncols == other_cols);

			Matrix<val_type, other_rows, other_cols> result;
			for (unsigned int i = 0; i < nrows; i++) 
				for (unsigned int j = 0; j < ncols; j++)
					result(i,j) = parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];

			return result;
		} 

		assert (nrows == other_cols);
		assert (ncols == other_rows);

		Matrix<val_type, other_rows, other_cols> result;
		for (unsigned int i = 0; i < nrows; i++) 
			for (unsigned int j = 0; j < ncols; j++)
				result(j,i) = parent[parent_nrows * (i + parent_row_index) + j + parent_col_index];

		return result;
	}

	unsigned int nrows;
	unsigned int ncols;
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
		};
		Matrix& operator=(const Matrix &matrix) {
			if (this != &matrix) {
				unsigned int i;
				for (i = 0; i < nrows * ncols; i++)
					mData[i] = matrix.mData[i];
			}
			return *this;
		};
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
				const val_type v00, const val_type v01, const val_type v02
				) {
			assert (rows * cols == 3);

			mData[0] = v00;
			mData[1] = v01;
			mData[2] = v02;
		}

		Matrix (
				const val_type &v00, const val_type &v01, const val_type &v02,
				const val_type &v10, const val_type &v11, const val_type &v12,
				const val_type &v20, const val_type &v21, const val_type &v22
				) {
			assert (nrows == 3);
			assert (ncols == 3);

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
			assert (rows == 3);
			assert (cols == 3);

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
			assert (nrows == 6);
			assert (ncols == 1);

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
			assert (nrows * ncols == 6);

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
			assert (nrows == 6);
			assert (ncols == 6);

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
			assert (nrows == 6);
			assert (ncols == 6);

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

		static matrix_type Zero() {
			matrix_type result;
			result.zero();
			return result;
		}

		static matrix_type Zero(int ignore_me) {
			matrix_type result;
			result.zero();
			return result;
		}

		void identity() {
			assert (ncols == nrows);

			zero();
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
			return Block<val_type, blockrows, blockcols> (const_cast<double*> (this->mData), i, j, nrows, ncols);
		}

		// Operators with scalars
		Matrix operator*(const val_type &scalar) const {
			matrix_type result (*this);

			for (unsigned int i = 0; i < nrows * ncols; i++)
				result[i] *= scalar;

			return result;
		}
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
			assert (ncols == matrix.rows());

			Matrix<val_type, nrows, other_cols> result;
			
			result.zero();

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
			assert (nrows == 1u);
			assert (ncols == 1u);

			return mData[0];
		}

	private:
		val_type mData[36];
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
Matrix<val_type, nrows, ncols> operator*(val_type scalar, const Matrix<val_type, nrows, ncols> &matrix) {
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
