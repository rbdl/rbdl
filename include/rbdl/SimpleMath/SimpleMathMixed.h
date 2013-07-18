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
#include <iostream>

#include "compileassert.h"

/** \brief Namespace for a highly inefficient math library
 *
 */
namespace SimpleMath {

// conversion Dynamic->Fixed
template <typename val_type, unsigned int nrows, unsigned int ncols>
inline Fixed::Matrix<val_type, nrows, ncols>::Matrix(const Dynamic::Matrix<val_type> &dynamic_matrix) {
	if (dynamic_matrix.cols() != ncols 
		|| dynamic_matrix.rows() != nrows) {
		std::cerr << "Error: cannot assign a dynamic sized matrix of size " << dynamic_matrix.rows() << "x" << dynamic_matrix.cols() << " to a fixed size matrix of size " << nrows << "x" << ncols << "!" << std::endl;
		abort();
	}
	
	for (unsigned int i = 0; i < nrows * ncols; i++) {
		mData[i] = dynamic_matrix[i];
	}
}

template <typename val_type, unsigned int nrows, unsigned int ncols>
inline Fixed::Matrix<val_type, nrows, ncols>& Fixed::Matrix<val_type, nrows, ncols>::operator=(const Dynamic::Matrix<val_type> &dynamic_matrix) {
	if (dynamic_matrix.cols() != ncols 
		|| dynamic_matrix.rows() != nrows) {
		std::cerr << "Error: cannot assign a dynamic sized matrix of size " << dynamic_matrix.rows() << "x" << dynamic_matrix.cols() << " to a fixed size matrix of size " << nrows << "x" << ncols << "!" << std::endl;
		abort();
	}
	
	for (unsigned int i = 0; i < nrows * ncols; i++) {
		mData[i] = dynamic_matrix[i];
	}

	return *this;
}

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
