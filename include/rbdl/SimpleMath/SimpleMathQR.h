#ifndef _SIMPLE_MATH_QR_H
#define _SIMPLE_MATH_QR_H

#include <iostream>
#include <limits>

#include "SimpleMathFixed.h"
#include "SimpleMathDynamic.h"
#include "SimpleMathBlock.h"

namespace SimpleMath {

	template <typename matrix_type>
	class HouseholderQR {
		public:
			typedef typename matrix_type::value_type value_type;	

		private:
			HouseholderQR() {}

			typedef Dynamic::Matrix<value_type> MatrixXXd;
			typedef Dynamic::Matrix<value_type> VectorXd;
			
			bool mIsFactorized;
			MatrixXXd mQ;
			MatrixXXd mR;

		public:
			HouseholderQR(const matrix_type &matrix) :
				mIsFactorized(false),
				mQ(matrix.rows(), matrix.rows()),
				mR(matrix) {
					compute();
			}
			HouseholderQR compute() {
				mQ = Dynamic::Matrix<value_type>::Identity (mR.rows(), mR.rows());

				for (unsigned int i = 0; i < mR.cols(); i++) {
					unsigned int block_rows = mR.rows() - i;
					unsigned int block_cols = mR.cols() - i;

					MatrixXXd current_block = mR.block(i,i, block_rows, block_cols);
					VectorXd column = current_block.block(0, 0, block_rows, 1);

					value_type alpha = - column.norm();
					if (current_block(0,0) < 0) {
						alpha = - alpha;
					}

					VectorXd v = current_block.block(0, 0, block_rows, 1);
					v[0] = v[0] - alpha;

					MatrixXXd Q (MatrixXXd::Identity(mR.rows(), mR.rows()));
					Q.block(i, i, block_rows, block_rows) = MatrixXXd (Q.block(i, i, block_rows, block_rows))
						- MatrixXXd(v * v.transpose() / (v.squaredNorm() * 0.5));

					mR = Q * mR;

					// Normalize so that we have positive diagonal elements
					if (mR(i,i) < 0) {
						mR.block(i,i,block_rows, block_cols) = MatrixXXd(mR.block(i,i,block_rows, block_cols)) * -1.;
						Q.block(i,i,block_rows, block_rows) = MatrixXXd(Q.block(i,i,block_rows, block_rows)) * -1.;
					}

					mQ = mQ * Q;
				}

				mIsFactorized = true;

				return *this;
			}
			Dynamic::Matrix<value_type> solve (
					const Dynamic::Matrix<value_type> &rhs
					) const {
				assert (mIsFactorized);

				VectorXd y = mQ.transpose() * rhs;
				VectorXd x = VectorXd::Zero(mR.cols());

				for (int i = mR.cols() - 1; i >= 0; --i) {
					value_type z = y[i];

					for (unsigned int j = i + 1; j < mR.cols(); j++) {
						z = z - x[j] * mR(i,j);
					}

					if (fabs(mR(i,i)) < std::numeric_limits<value_type>::epsilon() * 10) {
						std::cerr << "HouseholderQR: Cannot back-substitute as diagonal element is near zero!" << std::endl;
						abort();
					}
					x[i] = z / mR(i,i);
				}

				return x;
			}
			Dynamic::Matrix<value_type> inverse() const {
				assert (mIsFactorized);

				VectorXd rhs_temp = VectorXd::Zero(mQ.cols());
				MatrixXXd result (mQ.cols(), mQ.cols());

				for (unsigned int i = 0; i < mQ.cols(); i++) {
					rhs_temp[i] = 1.;

					result.block(0, i, mQ.cols(), 1) = solve(rhs_temp);

					rhs_temp[i] = 0.;
				}

				return result;
			}
			Dynamic::Matrix<value_type> matrixQ () const {
				return mQ;
			}
			Dynamic::Matrix<value_type> matrixR () const {
				return mR;
			}
	};

	template <typename matrix_type>
	class ColPivHouseholderQR {
		public:
			typedef typename matrix_type::value_type value_type;	

		private:
			ColPivHouseholderQR() {}

			typedef Dynamic::Matrix<value_type> MatrixXXd;
			typedef Dynamic::Matrix<value_type> VectorXd;
			
			bool mIsFactorized;
			MatrixXXd mQ;
			MatrixXXd mR;
			unsigned int *mPermutations;
			value_type mThreshold;
			unsigned int mRank;

		public:
			ColPivHouseholderQR(const matrix_type &matrix) :
				mIsFactorized(false),
				mQ(matrix.rows(), matrix.rows()),
				mR(matrix),
				mThreshold (std::numeric_limits<value_type>::epsilon() * matrix.cols()) {
					mPermutations = new unsigned int [matrix.cols()];
					for (unsigned int i = 0; i < matrix.cols(); i++) {
						mPermutations[i] = i;
					}
					compute();
			}
			~ColPivHouseholderQR() {
				delete[] mPermutations;
			}

			ColPivHouseholderQR& setThreshold (const value_type& threshold) {
				mThreshold = threshold;
				
				return *this;
			}
			ColPivHouseholderQR& compute() {
				mQ = Dynamic::Matrix<value_type>::Identity (mR.rows(), mR.rows());

				for (unsigned int i = 0; i < mR.cols(); i++) {
					unsigned int block_rows = mR.rows() - i;
					unsigned int block_cols = mR.cols() - i;

					// find and swap the column with the highest norm
					unsigned int col_index_norm_max = i;
					value_type col_norm_max = VectorXd(mR.block(i,i, block_rows, 1)).squaredNorm();

					for (unsigned int j = i + 1; j < mR.cols(); j++) {
						VectorXd column = mR.block(i, j, block_rows, 1);
						
						if (column.squaredNorm() > col_norm_max) {
							col_index_norm_max = j;
							col_norm_max = column.squaredNorm();
						}
					}

					if (col_norm_max < mThreshold) {
						// if all entries of the column is close to zero, we bail out
						break;
					}


					if (col_index_norm_max != i) {
						VectorXd temp_col = mR.block(0, i, mR.rows(), 1);
						mR.block(0,i,mR.rows(),1) = mR.block(0, col_index_norm_max, mR.rows(), 1);
						mR.block(0, col_index_norm_max, mR.rows(), 1) = temp_col;

						unsigned int temp_index = mPermutations[i];
						mPermutations[i] = mPermutations[col_index_norm_max];
						mPermutations[col_index_norm_max] = temp_index;
					}

					MatrixXXd current_block = mR.block(i,i, block_rows, block_cols);
					VectorXd column = current_block.block(0, 0, block_rows, 1);

					value_type alpha = - column.norm();
					if (current_block(0,0) < 0) {
						alpha = - alpha;
					}

					VectorXd v = current_block.block(0, 0, block_rows, 1);
					v[0] = v[0] - alpha;

					MatrixXXd Q (MatrixXXd::Identity(mR.rows(), mR.rows()));

					Q.block(i, i, block_rows, block_rows) = MatrixXXd (Q.block(i, i, block_rows, block_rows))
						- MatrixXXd(v * v.transpose() / (v.squaredNorm() * 0.5));

					mR = Q * mR;

					// Normalize so that we have positive diagonal elements
					if (mR(i,i) < 0) {
						mR.block(i,i,block_rows, block_cols) = MatrixXXd(mR.block(i,i,block_rows, block_cols)) * -1.;
						Q.block(i,i,block_rows, block_rows) = MatrixXXd(Q.block(i,i,block_rows, block_rows)) * -1.;
					}

					mQ = mQ * Q;
				}

				mIsFactorized = true;

				return *this;
			}
			Dynamic::Matrix<value_type> solve (
					const Dynamic::Matrix<value_type> &rhs
					) const {
				assert (mIsFactorized);

				VectorXd y = mQ.transpose() * rhs;
				VectorXd x = VectorXd::Zero(mR.cols());

				for (int i = mR.cols() - 1; i >= 0; --i) {
					value_type z = y[i];

					for (unsigned int j = i + 1; j < mR.cols(); j++) {
						z = z - x[mPermutations[j]] * mR(i,j);
					}

					if (fabs(mR(i,i)) < std::numeric_limits<value_type>::epsilon() * 10) {
						std::cerr << "HouseholderQR: Cannot back-substitute as diagonal element is near zero!" << std::endl;
						abort();
					}
					x[mPermutations[i]] = z / mR(i,i);
				}

				return x;
			}
			Dynamic::Matrix<value_type> inverse() const {
				assert (mIsFactorized);

				VectorXd rhs_temp = VectorXd::Zero(mQ.cols());
				MatrixXXd result (mQ.cols(), mQ.cols());

				for (unsigned int i = 0; i < mQ.cols(); i++) {
					rhs_temp[i] = 1.;

					result.block(0, i, mQ.cols(), 1) = solve(rhs_temp);

					rhs_temp[i] = 0.;
				}

				return result;
			}

			Dynamic::Matrix<value_type> matrixQ () const {
				return mQ;
			}
			Dynamic::Matrix<value_type> matrixR () const {
				return mR;
			}
			Dynamic::Matrix<value_type> matrixP () const {
				MatrixXXd P = MatrixXXd::Identity(mR.cols(), mR.cols());
				MatrixXXd identity = MatrixXXd::Identity(mR.cols(), mR.cols());
				for (unsigned int i = 0; i < mR.cols(); i++) {
					P.block(0,i,mR.cols(),1) = identity.block(0,mPermutations[i], mR.cols(), 1);
				}
				return P;
			}

			unsigned int rank() const {
				value_type abs_threshold = fabs(mR(0,0)) * mThreshold;

				for (unsigned int i = 1; i < mR.cols(); i++) {
					if (fabs(mR(i,i) < abs_threshold))
						return i;
				}

				return mR.cols();
			}
	};

}

/* _SIMPLE_MATH_QR_H */
#endif
