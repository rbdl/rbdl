#ifndef _SIMPLE_MATH_CHOLESKY_H
#define _SIMPLE_MATH_CHOLESKY_H

#include <iostream>
#include <limits>

#include "SimpleMathDynamic.h"

namespace SimpleMath {

	template <typename matrix_type>
	class LLT {
		public:
			typedef typename matrix_type::value_type value_type;	

		private:
			LLT () {}

			typedef Dynamic::Matrix<value_type> MatrixXXd;
			typedef Dynamic::Matrix<value_type> VectorXd;
			
			bool mIsFactorized;
			MatrixXXd mL;

		public:
			LLT (const matrix_type &matrix) :
				mIsFactorized(false),
				mL(matrix) {
					compute();
			}
			LLT compute() {
				for (int i = 0; i < mL.rows(); i++) {
					for (int j = 0; j < mL.rows(); j++) {
						if (j > i) {
							mL(i,j) = 0.;
							continue;
						}
						double s = mL(i,j);
						for (int k = 0; k < j; k++) {
							s = s - mL(i,k) * mL(j,k);
						}
						if (i > j) {
							mL(i,j) = s / mL(j,j);
						} else if (s > 0.) {
							mL (i,i) = sqrt (s);
						} else {
							std::cerr << "Error computing Cholesky decomposition: matrix not symmetric positive definite!" << std::endl;
							assert (false);
						}
					}
				}

				mIsFactorized = true;

				return *this;
			}
			Dynamic::Matrix<value_type> solve (
					const Dynamic::Matrix<value_type> &rhs
					) const {
				assert (mIsFactorized);

				VectorXd y (mL.rows());
				for (unsigned int i = 0; i < mL.rows(); i++) {
					double temp = rhs[i];

					for (unsigned int j = 0; j < i; j++) {
						temp = temp - mL(i,j) * y[j];
					}

					y[i] = temp / mL(i,i);
				}

				VectorXd x (mL.rows());
				for (int i = mL.rows() - 1; i >= 0; i--) {
					double temp = y[i];

					for (unsigned int j = i + 1; j < mL.rows(); j++) {
						temp = temp - mL(j, i) * x[j];
					}

					x[i] = temp / mL(i,i);
				}

				return x;
			}
			Dynamic::Matrix<value_type> matrixL() const {
				return mL;
			}
	};

}

/* _SIMPLE_MATH_CHOLESKY_H */
#endif
