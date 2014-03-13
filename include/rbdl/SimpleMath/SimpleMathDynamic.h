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

namespace Fixed {
	template <typename val_type, unsigned int ncols, unsigned int nrows> class Matrix;
}


/** \brief Namespace for elements of varying size.
 */
namespace Dynamic {

// forward declaration
template <typename val_type>
class Matrix;

/** \brief Class for both matrices and vectors.
 */
template <typename val_type>
class Matrix {
	public:
		typedef Matrix<val_type> matrix_type;
		typedef val_type value_type;

		Matrix() :
			nrows (0),
			ncols (0),
			mapped_data (false),
			mData (NULL) {};
		Matrix(unsigned int rows) :
			nrows (rows),
			ncols (1),
			mapped_data (false) {
				mData = new val_type[rows];
			}
		Matrix(unsigned int rows, unsigned int cols) :
			nrows (rows),
			ncols (cols),
			mapped_data (false) {
				mData = new val_type[rows * cols];
			}
		Matrix(unsigned int rows, unsigned int cols, val_type *data_ptr) :
			nrows (rows),
			ncols (cols),
			mapped_data (true) {
				mData = data_ptr;
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
			if (nrows * ncols > 0 && mData != NULL && mapped_data == false) {
				delete[] mData;
			}

			nrows = rows;
			ncols = cols;

			mData = new val_type[nrows * ncols];
		}

		void conservativeResize (unsigned int rows, unsigned int cols = 1) {
			Matrix <val_type> result = Matrix<val_type>::Zero(rows, cols);

			unsigned int arows = std::min (rows, nrows);
			unsigned int acols = std::min (cols, ncols);

			for (unsigned int i = 0; i < arows; i++) {
				for (unsigned int j = 0; j < acols; j++) {
					result(i,j) = (*this)(i,j);
				}
			}

			*this = result;	
		}

		Matrix(const Matrix &matrix) :
			nrows (matrix.nrows),
			ncols (matrix.ncols),
			mapped_data (false) {
			unsigned int i;
		
			mData = new val_type[nrows * ncols];

			for (i = 0; i < nrows * ncols; i++)
				mData[i] = matrix.mData[i];
		}
		Matrix& operator=(const Matrix &matrix) {
			if (this != &matrix) {
				if (!mapped_data) {
					delete[] mData;

					nrows = matrix.nrows;
					ncols = matrix.ncols;
					mapped_data = false;

					mData = new val_type[nrows * ncols];

					unsigned int i;
					for (i = 0; i < nrows * ncols; i++)
						mData[i] = matrix.mData[i];
				} else {
					// we overwrite any existing data
					nrows = matrix.nrows;
					ncols = matrix.ncols;
					mapped_data = true;

					unsigned int i;
					for (i = 0; i < nrows * ncols; i++)
						mData[i] = matrix.mData[i];
				}
			}
			return *this;
		}

		CommaInitializer<matrix_type> operator<< (const val_type& value) {
			return CommaInitializer<matrix_type> (*this, value);
		}

		// conversion different val_types
		template <typename other_type>
		Matrix (const Matrix<other_type> &matrix) :
			nrows (matrix.rows()),
			ncols (matrix.cols()),
			mapped_data(false) {

			mData = new val_type[nrows * ncols];

			for (unsigned int i = 0; i < nrows; i++) {
				for (unsigned int j = 0; j < ncols; j++) {
					(*this)(i,j) = static_cast<val_type>(matrix(i,j));
				}
			}
		}

		// conversion from a fixed size matrix
		template <typename other_type, unsigned int fnrows, unsigned int fncols>
		Matrix (const Fixed::Matrix<other_type, fnrows, fncols> &fixed_matrix) :
			nrows (fnrows),
			ncols (fncols),
			mapped_data (false),
			mData (NULL) {
				mData = new val_type[nrows * ncols];

				for (unsigned int i = 0; i < nrows; i++) {
					for (unsigned int j = 0; j < ncols; j++) {
						(*this)(i,j) = static_cast<val_type>(fixed_matrix(i,j));
					}
				}
			}

		template <typename other_matrix_type>
		Matrix (const Block<other_matrix_type, value_type> &block) :
			nrows(block.rows()),
			ncols(block.cols()),
			mapped_data (false) {
				mData = new val_type[nrows * ncols];

				for (unsigned int i = 0; i < nrows; i++) {
					for (unsigned int j = 0; j < ncols; j++) {
						(*this)(i,j) = static_cast<val_type>(block(i,j));
					}
				}

			}

		~Matrix() {
			if (nrows * ncols > 0 && mData != NULL && mapped_data == false) {
				delete[] mData;
				mData = NULL;
			}

			nrows = 0;
			ncols = 0;
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
		const val_type& operator[](const unsigned int &index) const {
			assert (index	>= 0);
			assert (index < nrows * ncols);
			return mData[index];
		};
		val_type& operator[](const unsigned int &index) {
			assert (index	>= 0 && index < nrows * ncols);
			return mData[index];
		}

		const val_type& operator()(const unsigned int &row, const unsigned int &col) const {
			if (!(row	>= 0 && row < nrows && col >= 0 && col < ncols)) {
				std::cout << "row = " << row << " col = " << col << std::endl;
				std::cout << "nrows = " << nrows << " ncols = " << ncols << std::endl;
				std::cout << "invalid read = " << mData[100000] << std::endl;
			}
			assert (row	>= 0 && row < nrows && col >= 0 && col < ncols);
			return mData[row*ncols + col];
		};
		val_type& operator()(const unsigned int &row, const unsigned int &col) {
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

		val_type norm() const {
			return sqrt(this->squaredNorm());
		}

		Matrix<val_type> normalize() {
			val_type length = this->norm();

			for (unsigned int i = 0; i < ncols * nrows; i++)
				mData[i] /= length;

			return *this;
		}

		matrix_type normalized() {
			return matrix_type (*this) / this->norm();
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

		static matrix_type Identity (int rows, int cols = 1) {
			assert (rows == cols);

			matrix_type result (rows, cols);
			result.identity();

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

		// Blocks
		Block<matrix_type, val_type>
			block (unsigned int row_start, unsigned int col_start, unsigned int row_count, unsigned int col_count) {
				return Block<matrix_type, val_type>(*this, row_start, col_start, row_count, col_count);
			}

		template <unsigned int row_count, unsigned int col_count>
		Block<matrix_type, val_type>
			block (unsigned int row_start, unsigned int col_start) {
				return Block<matrix_type, val_type>(*this, row_start, col_start, row_count, col_count);
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
			assert (ncols == 1);

			return mData[0];
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
		unsigned int nrows;
		unsigned int ncols;
		bool mapped_data;

		val_type* mData;
};

template <typename val_type>
inline Matrix<val_type> operator*(val_type scalar, const Matrix<val_type> &matrix) {
	Matrix<val_type> result (matrix);

	for (unsigned int i = 0; i < matrix.rows() * matrix.cols(); i++)
		result.data()[i] *= scalar;

	return result;
}

template <typename val_type, typename other_type>
inline Matrix<val_type> operator*(const Matrix<val_type> &matrix, other_type scalar) {
	Matrix<val_type> result (matrix);

	for (unsigned int i = 0; i < matrix.rows() * matrix.cols(); i++)
		result.data()[i] *= static_cast<val_type>(scalar);

	return result;
}

template <typename val_type>
inline std::ostream& operator<<(std::ostream& output, const Matrix<val_type> &matrix) {
	for (unsigned int i = 0; i < matrix.rows(); i++) {
		output << "[ ";
		for (unsigned int j = 0; j < matrix.cols(); j++) {
			output << matrix(i,j);

			if (j < matrix.cols() - 1)
				output << ", ";
		}
		output << " ]";

		if (matrix.rows() > 1 && i < matrix.rows() - 1)
			output << std::endl;
	}
	return output;
}

}

}

#endif /* SIMPLEMATHDYNAMIC_H */
