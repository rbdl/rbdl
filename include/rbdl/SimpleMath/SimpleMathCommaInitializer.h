#ifndef _SIMPLE_MATH_COMMA_INITIALIZER_H
#define _SIMPLE_MATH_COMMA_INITIALIZER_H

#include <iostream>
#include <limits>

#include "SimpleMathFixed.h"
#include "SimpleMathDynamic.h"

namespace SimpleMath {

	template <typename matrix_type>
	class CommaInitializer {
		public:
			typedef typename matrix_type::value_type value_type;	

			CommaInitializer(matrix_type &matrix, const value_type &value) :
				mElementWasAdded (false) {
				assert (matrix.cols() > 0 && matrix.rows() > 0);
				mParentMatrix = &matrix;
				mRowIndex = 0;
				mColIndex = 0;
				(*mParentMatrix)(mRowIndex, mColIndex) = value;
			}
			CommaInitializer(matrix_type &matrix, unsigned int row_index, unsigned int col_index) :
				mRowIndex (row_index),
				mColIndex (col_index),
				mElementWasAdded (false) {
				assert (matrix.cols() > 0 && matrix.rows() > 0);
				mParentMatrix = &matrix;
				mRowIndex = row_index;
				mColIndex = col_index;
			}
			~CommaInitializer() {
				if (!mElementWasAdded 
						&& (mColIndex + 1 < mParentMatrix->cols() || mRowIndex + 1 < mParentMatrix->rows())) {
					std::cerr << "Error: too few elements passed to CommaInitializer! Expected " << mParentMatrix->size() << " but was given " << mRowIndex * mParentMatrix->cols() + mColIndex + 1 << std::endl;
					abort();
				}
			}
			CommaInitializer<matrix_type> operator, (const value_type &value) {
				mColIndex++;
				if (mColIndex >= mParentMatrix->cols()) {
						mRowIndex++;
						mColIndex = 0;
				}
				if (mRowIndex == mParentMatrix->rows() && mColIndex == 0 ) {
					std::cerr << "Error: too many elements passed to CommaInitializer! Expected " << mParentMatrix->size() << " but was given " << mRowIndex * mParentMatrix->cols() + mColIndex + 1 << std::endl;
					abort();
				}
				(*mParentMatrix)(mRowIndex, mColIndex) = value;
				mElementWasAdded = true;

				return CommaInitializer (*mParentMatrix, mRowIndex, mColIndex);
			}

		private:
			CommaInitializer() {}

			matrix_type *mParentMatrix;
			unsigned int mRowIndex;
			unsigned int mColIndex;
			bool mElementWasAdded;
	};

}

/* _SIMPLE_MATH_COMMA_INITIALIZER_H */
#endif
