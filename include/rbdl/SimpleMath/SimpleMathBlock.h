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

#ifndef SIMPLEMATHBLOCK_H
#define SIMPLEMATHBLOCK_H

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <assert.h>

#include "compileassert.h"

// #include "SimpleMathQR.h"

/** \brief Namespace for a highly inefficient math library
 *
 */
namespace SimpleMath {

/** \brief Namespace for fixed size elements
 */
// forward declaration
template <typename val_type, unsigned int nrows, unsigned int ncols>
class Matrix;

template <typename matrix_type, typename val_type>
class Block {
	public:
		typedef val_type value_type;

		Block() :
			mParentRows(0),
			mParentCols(0),
			mParentRowStart(0),
			mParentColStart(0)
		{ }
		Block (const matrix_type &matrix, const unsigned int row_start, const unsigned int col_start, const unsigned int row_count, const unsigned int col_count) :
			mParentRows (matrix.rows()),
			mParentCols (matrix.cols()),
			mParentRowStart (row_start),
			mParentColStart (col_start),
			mRowCount (row_count),
			mColCount (col_count),
			mTransposed (false) {
				assert (mParentRows >= mParentRowStart + mRowCount);
				assert (mParentCols >= mParentColStart + mColCount);

				// without the following line we could not create blocks from const
				// matrices
				mParentMatrix = const_cast<matrix_type*>(&matrix);
			}

		// copy data from the other block into this
		Block& operator=(const Block &other) {
			if (this != &other) {
				if (mRowCount != other.rows() || mColCount != other.cols()) {
					std::cerr << "Error: cannot assign blocks of different size (left is " << mRowCount << "x" << mColCount << " right is " << other.rows() << "x" << other.cols() << ")!" << std::endl;
					abort();
				}
				
				value_type* temp_values = new value_type [mRowCount * mColCount];

				for (unsigned int i = 0; i < mRowCount; i++) {
					for (unsigned int j = 0; j < mColCount; j++) {
						temp_values[i * mColCount + j] = static_cast<value_type>(other(i,j));
					}
				}

				for (unsigned int i = 0; i < mRowCount; i++) {
					for (unsigned int j = 0; j < mColCount; j++) {
						(*this)(i,j) = temp_values[i * mColCount + j];
					}
				}

				delete[] temp_values;
			}

			return *this;
		}

		template <typename other_matrix_type>
		// copy data from the other block into this
		Block& operator=(const other_matrix_type &other) {
			if (mRowCount != other.rows() || mColCount != other.cols()) {
				std::cerr << "Error: cannot assign blocks of different size (left is " << mRowCount << "x" << mColCount << " right is " << other.rows() << "x" << other.cols() << ")!" << std::endl;
				abort();
			}

			value_type *temp_values = new value_type[mRowCount * mColCount];

			for (unsigned int i = 0; i < mRowCount; i++) {
				for (unsigned int j = 0; j < mColCount; j++) {
					temp_values[i * mColCount + j] = static_cast<value_type>(other(i,j));
				}
			}

			for (unsigned int i = 0; i < mRowCount; i++) {
				for (unsigned int j = 0; j < mColCount; j++) {
					(*this)(i,j) = temp_values[i * mColCount + j];
				}
			}
			
			delete[] temp_values;

			return *this;
		}

		unsigned int rows() const {
			return mRowCount;
		}
		unsigned int cols() const {
			return mColCount;
		}
		const val_type& operator() (const unsigned int i, const unsigned int j) const {
			assert (i < mRowCount);
			assert (j < mColCount);

			if (!mTransposed) {
				return (*mParentMatrix) (i + mParentRowStart, j + mParentColStart);
			}

			return (*mParentMatrix) (j + mParentRowStart, i + mParentColStart);
		}

		val_type& operator() (const unsigned int i, const unsigned int j) {
			assert (i < mRowCount);
			assert (j < mColCount);

			if (!mTransposed) {
				return (*mParentMatrix) (i + mParentRowStart, j + mParentColStart);
			}

			return (*mParentMatrix) (j + mParentRowStart, i + mParentColStart);
		}

		Block transpose() const {
			Block result (*this);
			result.mTransposed = mTransposed ^ true;
			return result;
		}

	private:
		matrix_type *mParentMatrix;
		const unsigned int mParentRows;
		const unsigned int mParentCols;
		const unsigned int mParentRowStart;
		const unsigned int mParentColStart;
		const unsigned int mRowCount;
		const unsigned int mColCount;
		bool mTransposed;
};

template <typename matrix_type, typename val_type>
inline std::ostream& operator<<(std::ostream& output, const Block<matrix_type, val_type> &block) {
	unsigned int i,j;
	for (i = 0; i < block.rows(); i++) {
		output << "[ ";
		for (j = 0; j < block.cols(); j++) {
			output << block(i,j);

			if (j < block.cols() - 1)
				output << ", ";
		}
		output << " ]";
		
		if (block.rows() > 1 && i < block.rows() - 1)
			output << std::endl;
	}

	return output;
}


}

#endif /* SIMPLEMATHBLOCK_H */
