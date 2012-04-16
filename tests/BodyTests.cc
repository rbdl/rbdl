#include <UnitTest++.h>

#include <iostream>

#include "rbdl_mathutils.h"
#include "Body.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

/* Tests whether the spatial inertia matches the one specified by its
 * parameters
 */
TEST ( TestComputeSpatialInertiaFromAbsoluteRadiiGyration ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialMatrix reference_inertia (
			4.843, -1.98, -2.145, 0, -1.43, 1.32,
			-1.98, 6.334, -1.716, 1.43, 0, -1.65,
			-2.145, -1.716, 7.059, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

//	cout << LogOutput.str() << endl;

	CHECK_ARRAY_CLOSE (reference_inertia.data(), body.mSpatialInertia.data(), 36, TEST_PREC);
}

TEST ( TestBodyConstructorSpatialRigidBodyInertiaMultiplyMotion ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialMatrix spatial_inertia = body.mSpatialInertia;
	SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
				body.mMass,
				body.mCenterOfMass,
				body.mInertia
				);

	SpatialVector mv (1.1, 1.2, 1.3, 1.4, 1.5, 1.6);
	SpatialVector fv_matrix = spatial_inertia * mv;
	SpatialVector fv_rbi = rbi * mv;

	CHECK_ARRAY_CLOSE (
			fv_matrix.data(),
			fv_rbi.data(),
			6,
			TEST_PREC
			);
}
TEST ( TestBodyConstructorSpatialRigidBodyInertia ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialMatrix spatial_inertia = body.mSpatialInertia;
	SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
				body.mMass,
				body.mCenterOfMass,
				body.mInertia
				);

	cout << "Spatial Inertia = " << endl << spatial_inertia << endl;
	cout << "rbi = " << endl << rbi.toMatrix() << endl;
	cout << "rbi.m = " << rbi.m << endl;
	cout << "rbi.h = " << rbi.h.transpose() << endl;
	cout << "rbi.I = " << endl << rbi.I << endl;

	CHECK_ARRAY_CLOSE (
			spatial_inertia.data(),
			rbi.toMatrix().data(),
			36,
			TEST_PREC
			);
}
