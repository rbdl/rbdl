#ifndef SPATIALALGEBRA_H
#define SPATIALALGEBRA_H

#include <sstream>

namespace SpatialAlgebra {

class SpatialVector;
class SpatialMatrix;

/** \brief Vector class
 */
class SpatialVector {
	public:
		SpatialVector() {};
		SpatialVector(const SpatialVector &vector) {
			unsigned int i;
			for (i = 0; i < 6; i++)
				mData[i] = vector.mData[i];
		};
		SpatialVector& operator=(const SpatialVector &vector) {
			if (this != &vector) {
				unsigned int i;
				for (i = 0; i < 6; i++)
					mData[i] = vector.mData[i];
			}
			return *this;
		};
		~SpatialVector() {};

		SpatialVector (const double &v0, const double &v1, const double &v2,
				const double &v3, const double &v4, const double &v5) {
			mData[0] = v0;
			mData[1] = v1;
			mData[2] = v2;
			mData[3] = v3;
			mData[4] = v4;
			mData[5] = v5;
		};

		// comparison
		bool operator==(const SpatialVector &vector) const {
			for (unsigned int i = 0; i < 6; i++) {
				if (mData[i] != vector.mData[i])
					return false;
			}
			return true;
		}

		// access operators
		const double& operator[](const unsigned int &index) const {
			assert (index	>= 0 && index < 6);
			return mData[index];
		};
		double& operator[](const unsigned int &index) {
			assert (index	>= 0 && index < 6);
			return mData[index];
		}

		const double& operator()(const unsigned int &index) const {
			assert (index	>= 0 && index < 6);
			return mData[index];
		};
		double& operator()(const unsigned int &index) {
			assert (index	>= 0 && index < 6);
			return mData[index];
		};
		
		void set(const double &v0, const double &v1, const double &v2,
				const double &v3, const double &v4, const double &v5) {
			mData[0] = v0;
			mData[1] = v1;
			mData[2] = v2;
			mData[3] = v3;
			mData[4] = v4;
			mData[5] = v5;		};
		void zero() {
			set(0., 0., 0., 0., 0., 0.);
		}

		// Operators with scalars
		SpatialVector operator*(const double &scalar) const {
			return SpatialVector(
					mData[0] * scalar,
					mData[1] * scalar,
					mData[2] * scalar,
					mData[3] * scalar,
					mData[4] * scalar,
					mData[5] * scalar
					);
		};
		void operator*=(const double &scalar) {
			mData[0] *= scalar;
			mData[1] *= scalar;
			mData[2] *= scalar;
			mData[3] *= scalar;
			mData[4] *= scalar;
			mData[5] *= scalar;
		};
		void operator/=(const double &scalar) {
			mData[0] /= scalar;
			mData[1] /= scalar;
			mData[2] /= scalar;
			mData[3] /= scalar;
			mData[4] /= scalar;
			mData[5] /= scalar;
		};

		// Operators with other spatial vectors
		SpatialVector operator+(const SpatialVector &vector) const {
			return SpatialVector(
					mData[0] + vector.mData[0],
					mData[1] + vector.mData[1],
					mData[2] + vector.mData[2],
					mData[3] + vector.mData[3],
					mData[4] + vector.mData[4],
					mData[5] + vector.mData[5]
					);
		}
		void operator+=(const SpatialVector &vector) {
			mData[0] += vector.mData[0];
			mData[1] += vector.mData[1];
			mData[2] += vector.mData[2];
			mData[3] += vector.mData[3];
			mData[4] += vector.mData[4];
			mData[5] += vector.mData[5];
		}

		// ToDo
		SpatialVector cross(const SpatialVector &vector) const {
		}
	
	private:
		double mData[6];
};

/** \brief Matrix class
 */
class SpatialMatrix {
	public:
		SpatialMatrix() {};
		SpatialMatrix(const SpatialMatrix &matrix) {
			unsigned int i;
			for (i = 0; i < 36; i++)
				mData[i] = matrix.mData[i];
		};
		SpatialMatrix& operator=(const SpatialMatrix &matrix) {
			if (this != &matrix) {
				unsigned int i;
				for (i = 0; i < 36; i++)
					mData[i] = matrix.mData[i];
			}
			return *this;
		};
		~SpatialMatrix() {};

		SpatialMatrix (
				const double &v00, const double &v01, const double &v02,
				const double &v03, const double &v04, const double &v05,

				const double &v10, const double &v11, const double &v12,
				const double &v13, const double &v14, const double &v15,

				const double &v20, const double &v21, const double &v22,
				const double &v23, const double &v24, const double &v25,

				const double &v30, const double &v31, const double &v32,
				const double &v33, const double &v34, const double &v35,

				const double &v40, const double &v41, const double &v42,
				const double &v43, const double &v44, const double &v45,

				const double &v50, const double &v51, const double &v52,
				const double &v53, const double &v54, const double &v55
				) {
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

		// comparison
		bool operator==(const SpatialMatrix &matrix) const {
			for (unsigned int i = 0; i < 36; i++) {
				if (mData[i] != matrix.mData[i])
					return false;
			}
			return true;
		}

		// access operators
		const double& operator[](const unsigned int &index) const {
			assert (index	>= 0 && index < 36);
			return mData[index];
		};
		double& operator[](const unsigned int &index) {
			assert (index	>= 0 && index < 36);
			return mData[index];
		}

		const double& operator()(const unsigned int &row, const unsigned int &col) const {
			assert (row	>= 0 && row < 6 && row	>= 0 && row < 6);
			return mData[row*6 + col];
		};
		double& operator()(const unsigned int &row, const unsigned int &col) {
			assert (row	>= 0 && row < 6 && row	>= 0 && row < 6);
			return mData[row*6 + col];
		};
		
		void set(
				const double &v00, const double &v01, const double &v02,
				const double &v03, const double &v04, const double &v05,

				const double &v10, const double &v11, const double &v12,
				const double &v13, const double &v14, const double &v15,

				const double &v20, const double &v21, const double &v22,
				const double &v23, const double &v24, const double &v25,

				const double &v30, const double &v31, const double &v32,
				const double &v33, const double &v34, const double &v35,

				const double &v40, const double &v41, const double &v42,
				const double &v43, const double &v44, const double &v45,

				const double &v50, const double &v51, const double &v52,
				const double &v53, const double &v54, const double &v55
				) {
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

		void zero() {
			set(
					0., 0., 0., 0., 0., 0.,
					0., 0., 0., 0., 0., 0.,
					0., 0., 0., 0., 0., 0.,
					0., 0., 0., 0., 0., 0.,
					0., 0., 0., 0., 0., 0.,
					0., 0., 0., 0., 0., 0.
				 );
		}

		void identity() {
			set(
					1., 0., 0., 0., 0., 0.,
					0., 1., 0., 0., 0., 0.,
					0., 0., 1., 0., 0., 0.,
					0., 0., 0., 1., 0., 0.,
					0., 0., 0., 0., 1., 0.,
					0., 0., 0., 0., 0., 1.
				 );
		}

		// Operators with scalars
		SpatialMatrix operator*(const double &scalar) const {
			return SpatialMatrix(
					mData[0] * scalar,
					mData[1] * scalar,
					mData[2] * scalar,
					mData[3] * scalar,
					mData[4] * scalar,
					mData[5] * scalar,

					mData[6 + 0] * scalar,
					mData[6 + 1] * scalar,
					mData[6 + 2] * scalar,
					mData[6 + 3] * scalar,
					mData[6 + 4] * scalar,
					mData[6 + 5] * scalar,

					mData[12 + 0] * scalar,
					mData[12 + 1] * scalar,
					mData[12 + 2] * scalar,
					mData[12 + 3] * scalar,
					mData[12 + 4] * scalar,
					mData[12 + 5] * scalar,

					mData[18 + 0] * scalar,
					mData[18 + 1] * scalar,
					mData[18 + 2] * scalar,
					mData[18 + 3] * scalar,
					mData[18 + 4] * scalar,
					mData[18 + 5] * scalar,

					mData[24 + 0] * scalar,
					mData[24 + 1] * scalar,
					mData[24 + 2] * scalar,
					mData[24 + 3] * scalar,
					mData[24 + 4] * scalar,
					mData[24 + 5] * scalar,

					mData[30 + 0] * scalar,
					mData[30 + 1] * scalar,
					mData[30 + 2] * scalar,
					mData[30 + 3] * scalar,
					mData[30 + 4] * scalar,
					mData[30 + 5] * scalar
					);
		}
		void operator*=(const double &scalar) {
			for (unsigned int i = 0; i < 36; i++)
				mData[i] *= scalar;
		};
		void operator/=(const double &scalar) {
			for (unsigned int i = 0; i < 36; i++)
				mData[i] /= scalar;
		}

		// Operators with other spatial matrices
		SpatialMatrix operator+(const SpatialMatrix &matrix) const {
			return SpatialMatrix(
					mData[0] + matrix.mData[0],
					mData[1] + matrix.mData[1],
					mData[2] + matrix.mData[2],
					mData[3] + matrix.mData[3],
					mData[4] + matrix.mData[4],
					mData[5] + matrix.mData[5],

					mData[6 + 0] + matrix.mData[6 + 0],
					mData[6 + 1] + matrix.mData[6 + 1],
					mData[6 + 2] + matrix.mData[6 + 2],
					mData[6 + 3] + matrix.mData[6 + 3],
					mData[6 + 4] + matrix.mData[6 + 4],
					mData[6 + 5] + matrix.mData[6 + 5],

					mData[12 + 0] + matrix.mData[12 + 0],
					mData[12 + 1] + matrix.mData[12 + 1],
					mData[12 + 2] + matrix.mData[12 + 2],
					mData[12 + 3] + matrix.mData[12 + 3],
					mData[12 + 4] + matrix.mData[12 + 4],
					mData[12 + 5] + matrix.mData[12 + 5],

					mData[18 + 0] + matrix.mData[18 + 0],
					mData[18 + 1] + matrix.mData[18 + 1],
					mData[18 + 2] + matrix.mData[18 + 2],
					mData[18 + 3] + matrix.mData[18 + 3],
					mData[18 + 4] + matrix.mData[18 + 4],
					mData[18 + 5] + matrix.mData[18 + 5],

					mData[24 + 0] + matrix.mData[24 + 0],
					mData[24 + 1] + matrix.mData[24 + 1],
					mData[24 + 2] + matrix.mData[24 + 2],
					mData[24 + 3] + matrix.mData[24 + 3],
					mData[24 + 4] + matrix.mData[24 + 4],
					mData[24 + 5] + matrix.mData[24 + 5],

					mData[30 + 0] + matrix.mData[30 + 0],
					mData[30 + 1] + matrix.mData[30 + 1],
					mData[30 + 2] + matrix.mData[30 + 2],
					mData[30 + 3] + matrix.mData[30 + 3],
					mData[30 + 4] + matrix.mData[30 + 4],
					mData[30 + 5] + matrix.mData[30 + 5]
					);
		}
		void operator+=(const SpatialMatrix &matrix) {
			for (unsigned int i = 0; i < 36; i++)
				mData[i] += matrix.mData[i];
		}
		SpatialMatrix operator*(const SpatialMatrix &matrix) {
			SpatialMatrix result;
			result.zero();

			unsigned int i,j, k;
			for (i = 0; i < 6; i++) {
				for (j = 0; j < 6; j++) {
					for (k = 0; k < 6; k++) {
						result(i,j) += mData[i * 6 + k] * matrix.mData[k * 6 + j];
					}
				}
			}
				
			return result;
		}
		SpatialVector operator*(const SpatialVector &vector) {
			SpatialVector result;
			result.zero();

			unsigned i,j;
			for (i = 0; i < 6; i++) {
				for (j = 0; j < 6; j++) {
					result[i] = mData[i * 6 + j] * vector[j];
				}
			}

			return result;
		}

	private:
		double mData[36];
};

inline std::ostream& operator<<(std::ostream& output, const SpatialVector &vector) {
	output << vector[0] << " " << vector[1] << " " << vector[2] << " "
		<< vector[3] << " " << vector[4] << " " << vector[5];
	return output;
}

inline std::ostream& operator<<(std::ostream& output, const SpatialMatrix &matrix) {
	output << "[ " <<matrix(0,0) << " " << matrix(0,1) << " " << matrix(0,2) << " "
		<< matrix(0,3) << " " << matrix(0,4) << " " << matrix(0,5) << " ]" << std::endl;

	output << "[ " <<matrix(1,0) << " " << matrix(1,1) << " " << matrix(1,2) << " "
		<< matrix(1,3) << " " << matrix(1,4) << " " << matrix(1,5) << " ]" << std::endl;

	output << "[ " <<matrix(2,0) << " " << matrix(2,1) << " " << matrix(2,2) << " "
		<< matrix(2,3) << " " << matrix(2,4) << " " << matrix(2,5) << " ]" << std::endl;

	output << "[ " <<matrix(3,0) << " " << matrix(3,1) << " " << matrix(3,2) << " "
		<< matrix(3,3) << " " << matrix(3,4) << " " << matrix(3,5) << " ]" << std::endl;

	output << "[ " <<matrix(4,0) << " " << matrix(4,1) << " " << matrix(4,2) << " "
		<< matrix(4,3) << " " << matrix(4,4) << " " << matrix(4,5) << " ]" << std::endl;

	output << "[ " <<matrix(5,0) << " " << matrix(5,1) << " " << matrix(5,2) << " "
		<< matrix(5,3) << " " << matrix(5,4) << " " << matrix(5,5) << " ]" << std::endl;

	return output;
}

}

#endif /* SPATIALALGEBRA_H */

