#ifndef _SPATIALALGEBRAOPERATORS_H
#define _SPATIALALGEBRAOPERATORS_H

namespace SpatialAlgebra {

typedef Eigen::Matrix< double, 6, 1> SpatialVector;
typedef Eigen::Matrix< double, 6, 6> SpatialMatrix;

namespace Operators {
	inline SpatialMatrix crossm (const SpatialVector &v) {
		return SpatialMatrix (
		        0,  -v[2],  v[1],         0,          0,         0,
		 v[2],          0, -v[0],         0,          0,         0, 
		-v[1],   v[0],         0,         0,          0,         0,
		        0,  -v[5],  v[4],         0,  -v[2],  v[1],
		 v[5],          0, -v[3],  v[2],          0, -v[0],
		-v[4],   v[3],         0, -v[1],   v[0],         0
			);
	}
	inline SpatialVector crossm (const SpatialVector &v1, const SpatialVector &v2) {
		return crossm(v1) * v2;
	}
	inline SpatialMatrix crossf (const SpatialVector &v) {
		return SpatialMatrix (
		        0,  -v[2],  v[1],         0,  -v[5],  v[4],
		 v[2],          0, -v[0],  v[5],          0, -v[3],
		-v[1],   v[0],         0, -v[4],   v[3],         0,
		        0,          0,         0,         0,  -v[2],  v[1],
		        0,          0,         0,  v[2],          0, -v[0],
		        0,          0,         0, -v[1],   v[0],         0
			);
	}
	inline SpatialVector crossf (const SpatialVector &v1, const SpatialVector &v2) {
		return crossf(v1) * v2;
	}

	inline SpatialMatrix spatial_adjoint(const SpatialMatrix &m) {
		SpatialMatrix res (m);
		res.block<3,3>(3,0) = m.block<3,3>(0,3);
		res.block<3,3>(0,3) = m.block<3,3>(3,0);
		return res;
	}

	inline SpatialMatrix spatial_inverse(const SpatialMatrix &m) {
		SpatialMatrix res(m);
		res.block<3,3>(0,0) = m.block<3,3>(0,0).transpose();
		res.block<3,3>(3,0) = m.block<3,3>(3,0).transpose();
		res.block<3,3>(0,3) = m.block<3,3>(0,3).transpose();
		res.block<3,3>(3,3) = m.block<3,3>(3,3).transpose();
		return res;
	}

	inline SpatialVector SpatialLinSolve (const SpatialMatrix &A, const SpatialVector &b) {
		return A.partialPivLu().solve(b);
	}

	inline Eigen::Matrix< double, 3, 3> get_rotation (const SpatialMatrix &m) {
		return m.block<3,3>(0,0);

	}
	inline Eigen::Matrix< double, 3, 1> get_translation (const SpatialMatrix &m) {
		Eigen::Matrix< double, 3, 1> result (-m.data()[26], m.data()[20], -m.data()[19]);
		return result;
	}
}

}

/* _SPATIALALGEBRAOPERATORS_H*/
#endif
