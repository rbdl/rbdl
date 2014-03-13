#ifndef _RBDL_QUATERNION_H
#define _RBDL_QUATERNION_H

#include <cmath>

namespace RigidBodyDynamics {

namespace Math {

/** Quaternion 
 *
 * order: x,y,z,w
 */
class Quaternion : public Vector4d {
	public:
		Quaternion () :
			Vector4d (0.f, 0.f, 0.f, 1.f)
		{}
		Quaternion (const Vector4d vec4) :
			Vector4d (vec4)
		{}
		Quaternion (double x, double y, double z, double w):
			Vector4d (x, y, z, w)
		{}
		Quaternion operator* (const double &s) const {
			return Quaternion (
					(*this)[0] * s,
					(*this)[1] * s,
					(*this)[2] * s,
					(*this)[3] * s
					);
		}
		/** This function is equivalent to multiplicate their corresponding rotation matrices */
		Quaternion operator* (const Quaternion &q) const {
			return Quaternion (
					q[3] * (*this)[0] + q[0] * (*this)[3] + q[1] * (*this)[2] - q[2] * (*this)[1],
					q[3] * (*this)[1] + q[1] * (*this)[3] + q[2] * (*this)[0] - q[0] * (*this)[2],
					q[3] * (*this)[2] + q[2] * (*this)[3] + q[0] * (*this)[1] - q[1] * (*this)[0],
					q[3] * (*this)[3] - q[0] * (*this)[0] - q[1] * (*this)[1] - q[2] * (*this)[2]
					);
		}
		Quaternion& operator*=(const Quaternion &q) {
			set (
					q[3] * (*this)[0] + q[0] * (*this)[3] + q[1] * (*this)[2] - q[2] * (*this)[1],
					q[3] * (*this)[1] + q[1] * (*this)[3] + q[2] * (*this)[0] - q[0] * (*this)[2],
					q[3] * (*this)[2] + q[2] * (*this)[3] + q[0] * (*this)[1] - q[1] * (*this)[0],
					q[3] * (*this)[3] - q[0] * (*this)[0] - q[1] * (*this)[1] - q[2] * (*this)[2]
					);
			return *this;
		}

		static Quaternion fromGLRotate (double angle, double x, double y, double z) {
			double st = std::sin (angle * M_PI / 360.f);
			return Quaternion (
						st * x,
						st * y,
						st * z,
						cosf (angle * M_PI / 360.f)
						);
		}

		Quaternion slerp (double alpha, const Quaternion &quat) const {
			// check whether one of the two has 0 length
			double s = std::sqrt (squaredNorm() * quat.squaredNorm());

			// division by 0.f is unhealthy!
			assert (s != 0.);

			double angle = acos (dot(quat) / s);
			if (angle == 0. || std::isnan(angle)) {
				return *this;
			}
			assert(!std::isnan(angle));

			double d = 1. / std::sin (angle);
			double p0 = std::sin ((1. - alpha) * angle);
			double p1 = std::sin (alpha * angle);

			if (dot (quat) < 0.) {
				return Quaternion( ((*this) * p0 - quat * p1) * d);
			}
			return Quaternion( ((*this) * p0 + quat * p1) * d);
		}

		static Quaternion fromAxisAngle (const Vector3d &axis, double angle_rad) {
			double d = axis.norm();
			double s2 = std::sin (angle_rad * 0.5) / d;
			return Quaternion (
					axis[0] * s2,
					axis[1] * s2,
					axis[2] * s2,
					std::cos(angle_rad * 0.5)
					);
		}

		static Quaternion fromMatrix (const Matrix3d &mat) {
			double w = std::sqrt (1. + mat(0,0) + mat(1,1) + mat(2,2)) * 0.5;
			return Quaternion (
					(mat(1,2) - mat(2,1)) / (w * 4.),
					(mat(2,0) - mat(0,2)) / (w * 4.),
					(mat(0,1) - mat(1,0)) / (w * 4.),
					w);
		}

		static Quaternion fromZYXAngles (const Vector3d &zyx_angles) {
			return Quaternion::fromAxisAngle (Vector3d (1., 0., 0.), zyx_angles[2]) 
				* Quaternion::fromAxisAngle (Vector3d (0., 1., 0.), zyx_angles[1])
				* Quaternion::fromAxisAngle (Vector3d (0., 0., 1.), zyx_angles[0]);
		}

		Matrix3d toMatrix() const {
			double x = (*this)[0];
			double y = (*this)[1];
			double z = (*this)[2];
			double w = (*this)[3];
			return Matrix3d (
					1 - 2*y*y - 2*z*z,
					2*x*y + 2*w*z,
					2*x*z - 2*w*y,

					2*x*y - 2*w*z,
					1 - 2*x*x - 2*z*z,
					2*y*z + 2*w*x,

					2*x*z + 2*w*y,
					2*y*z - 2*w*x,
					1 - 2*x*x - 2*y*y

/*
					1 - 2*y*y - 2*z*z,
					2*x*y - 2*w*z,
					2*x*z + 2*w*y,

					2*x*y + 2*w*z,
					1 - 2*x*x - 2*z*z,
					2*y*z - 2*w*x,

					2*x*z - 2*w*y,
					2*y*z + 2*w*x,
					1 - 2*x*x - 2*y*y
					*/
			);
		}

		Quaternion conjugate() const {
			return Quaternion (
					-(*this)[0],
					-(*this)[1],
					-(*this)[2],
					(*this)[3]);
		}

		Quaternion timeStep (const Vector3d &omega, double dt) {
			double omega_norm = omega.norm();
			return (*this) * Quaternion::fromAxisAngle (omega / omega_norm, dt * omega_norm);
		}

		Vector3d rotate (const Vector3d &vec) const {
			Vector3d vn (vec);
			Quaternion vec_quat (vn[0], vn[1], vn[2], 0.f), res_quat;

			res_quat = vec_quat * (*this);
			res_quat = conjugate() * res_quat;

			return Vector3d (res_quat[0], res_quat[1], res_quat[2]);
		}
};

}

}

/* _RBDL_QUATERNION_H */
#endif
