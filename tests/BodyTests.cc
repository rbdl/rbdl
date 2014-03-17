#include <UnitTest++.h>

#include <iostream>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Body.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

/* Tests whether the spatial inertia matches the one specified by its
 * parameters
 */
TEST ( TestComputeSpatialInertiaFromAbsoluteRadiiGyration ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	Matrix3d inertia_C (
			1.4, 0., 0.,
			0., 2., 0., 
			0., 0., 3.);

	SpatialMatrix reference_inertia (
			4.843, -1.98, -2.145, 0, -1.43, 1.32,
			-1.98, 6.334, -1.716, 1.43, 0, -1.65,
			-2.145, -1.716, 7.059, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

//	cout << LogOutput.str() << endl;

	Body com_body(1.1, Vector3d (0., 0., 0.), Vector3d (1.4, 2., 3.));

	CHECK_ARRAY_CLOSE (reference_inertia.data(), body.mSpatialInertia.data(), 36, TEST_PREC);
	CHECK_ARRAY_CLOSE (inertia_C.data(), body.mInertia.data(), 9, TEST_PREC);
}

TEST ( TestBodyConstructorMassComInertia ) {
	double mass = 1.1;
	Vector3d com (1.5, 1.2, 1.3);
	Matrix3d inertia_C (
			8.286, -3.96, -4.29,
			-3.96, 10.668, -3.432,
			-4.29, -3.432, 11.118
			);

	Body body (mass, com, inertia_C);

	SpatialMatrix reference_inertia (
			11.729, -5.94, -6.435, 0, -1.43, 1.32,
			-5.94, 15.002, -5.148, 1.43, 0, -1.65,
			-6.435, -5.148, 15.177, -1.32, 1.65, 0,
			0, 1.43, -1.32, 1.1, 0, 0,
			-1.43, 0, 1.65, 0, 1.1, 0,
			1.32, -1.65, 0, 0, 0, 1.1
			);

	CHECK_ARRAY_CLOSE (reference_inertia.data(), body.mSpatialInertia.data(), 36, TEST_PREC);
	CHECK_ARRAY_CLOSE (inertia_C.data(), body.mInertia.data(), 9, TEST_PREC);
}

TEST ( TestBodyJoinNullbody ) {
	ClearLogOutput();
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));
	Body nullbody (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));

	SpatialMatrix spatial_inertia = body.mSpatialInertia;

	body.Join (Xtrans(Vector3d (0., 0., 0.)), nullbody);
	// cout << LogOutput.str() << endl;
	// cout << "old spatial inertia = " << endl << spatial_inertia << endl;
	// cout << "new spatial inertia = " << endl << body.mSpatialInertia << endl;
	// cout << "error = " << endl << body.mSpatialInertia - spatial_inertia << endl;

	CHECK_EQUAL (1.1, body.mMass);
	CHECK_ARRAY_CLOSE (Vector3d (1.5, 1.2, 1.3).data(), body.mCenterOfMass.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (spatial_inertia.data(), body.mSpatialInertia.data(), 36, TEST_PREC);
}

TEST ( TestBodyJoinTwoBodies ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));

	// cout << "body_a = " << endl << body_a.mSpatialInertia << endl;
	// cout << "body_b = " << endl << body_b.mSpatialInertia << endl;

	Body body_joined (body_a);
	body_joined.Join (Xtrans(Vector3d (0., 0., 0.)), body_b);

	// cout << LogOutput.str() << endl;
	// cout << "body_joined = " << endl << body_joined.mSpatialInertia << endl;

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	CHECK_EQUAL (2.2, body_joined.mMass);
	CHECK_ARRAY_EQUAL (Vector3d (0., 1.3, 0.).data(), body_joined.mCenterOfMass.data(), 3);
	CHECK_ARRAY_CLOSE (reference_inertia.data(), body_joined.mSpatialInertia.data(), 36, TEST_PREC);
}

TEST ( TestBodyJoinTwoBodiesDisplaced ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (-1.1, 1.3, 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));

	// cout << "body_a = " << endl << body_a.mSpatialInertia << endl;
	// cout << "body_b = " << endl << body_b.mSpatialInertia << endl;

	Body body_joined (body_a);
	body_joined.Join (Xtrans(Vector3d (1.1, 1.3, 0.)), body_b);

	// cout << LogOutput.str() << endl;
	// cout << "body_joined = " << endl << body_joined.mSpatialInertia << endl;

	SpatialMatrix reference_inertia (
			9.918, 0, 0, 0, -0, 2.86,
			0, 9.062, 0, 0, 0, -0,
			0, 0, 12.98, -2.86, 0, 0,
			0, 0, -2.86, 2.2, 0, 0,
			-0, 0, 0, 0, 2.2, 0,
			2.86, -0, 0, 0, 0, 2.2
			);

	CHECK_EQUAL (2.2, body_joined.mMass);
	CHECK_ARRAY_EQUAL (Vector3d (0., 1.3, 0.).data(), body_joined.mCenterOfMass.data(), 3);
	CHECK_ARRAY_CLOSE (reference_inertia.data(), body_joined.mSpatialInertia.data(), 36, TEST_PREC);
}

TEST ( TestBodyJoinTwoBodiesRotated ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.3, 3.2));

	// cout << "body_a = " << endl << body_a.mSpatialInertia << endl;
	// cout << "body_b = " << endl << body_b.mSpatialInertia << endl;

	Body body_joined (body_a);
	body_joined.Join (Xrotx(-M_PI*0.5), body_b);
	
	// cout << LogOutput.str() << endl;
	// cout << "body_joined = " << endl << body_joined.mSpatialInertia << endl;

	SpatialMatrix reference_inertia (
			6.2, 0., 0., 0., 0., 0.,
			0., 6.4, 0., 0., 0., 0.,
			0., 0., 6.6, 0., 0., 0.,
			0., 0., 0., 2.2, 0., 0.,
			0., 0., 0., 0., 2.2, 0.,
			0., 0., 0., 0., 0., 2.2
				);

	CHECK_EQUAL (2.2, body_joined.mMass);
	CHECK_ARRAY_EQUAL (Vector3d (0., 0., 0.).data(), body_joined.mCenterOfMass.data(), 3);
	CHECK_ARRAY_CLOSE (reference_inertia.data(), body_joined.mSpatialInertia.data(), 36, TEST_PREC);
}

TEST ( TestBodyJoinTwoBodiesRotatedAndTranslated ) {
	ClearLogOutput();
	Body body_a(1.1, Vector3d (0., 0., 0.), Vector3d (3.1, 3.2, 3.3));
	Body body_b(1.1, Vector3d (-1., 1., 0.), Vector3d (3.2, 3.1, 3.3));

	// cout << "body_a = " << endl << body_a.mSpatialInertia << endl;
	// cout << "body_b = " << endl << body_b.mSpatialInertia << endl;

	Body body_joined (body_a);
	body_joined.Join (Xrotz(M_PI*0.5) * Xtrans(Vector3d (1., 1., 0.)), body_b);
	
	// cout << LogOutput.str() << endl;
	// cout << "body_joined = " << endl << body_joined.mSpatialInertia << endl;

	SpatialMatrix reference_inertia (
			6.2, 0., 0., 0., 0., 0.,
			0., 6.4, 0., 0., 0., 0.,
			0., 0., 6.6, 0., 0., 0.,
			0., 0., 0., 2.2, 0., 0.,
			0., 0., 0., 0., 2.2, 0.,
			0., 0., 0., 0., 0., 2.2
				);

	CHECK_EQUAL (2.2, body_joined.mMass);
	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.).data(), body_joined.mCenterOfMass.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (reference_inertia.data(), body_joined.mSpatialInertia.data(), 36, TEST_PREC);
}

TEST ( TestBodyConstructorSpatialRigidBodyInertiaMultiplyMotion ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialMatrix spatial_inertia = body.mSpatialInertia;
	SpatialRigidBodyInertia rbi = SpatialRigidBodyInertia(
				body.mMass,
				body.mCenterOfMass * body.mMass,
				body.mSpatialInertia.block<3,3>(0,0)
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
				body.mCenterOfMass * body.mMass,
				body.mSpatialInertia.block<3,3>(0,0)
				);

//	cout << "Spatial Inertia = " << endl << spatial_inertia << endl;
//	cout << "rbi = " << endl << rbi.toMatrix() << endl;
//	cout << "rbi.m = " << rbi.m << endl;
//	cout << "rbi.h = " << rbi.h.transpose() << endl;
//	cout << "rbi.I = " << endl << rbi.I << endl;

	CHECK_ARRAY_CLOSE (
			spatial_inertia.data(),
			rbi.toMatrix().data(),
			36,
			TEST_PREC
			);
}

TEST ( TestBodyConstructorCopySpatialRigidBodyInertia ) {
	Body body(1.1, Vector3d (1.5, 1.2, 1.3), Vector3d (1.4, 2., 3.));

	SpatialMatrix spatial_inertia = body.mSpatialInertia;

	SpatialRigidBodyInertia rbi;
	rbi.createFromMatrix (spatial_inertia);

//	cout << "Spatial Inertia = " << endl << spatial_inertia << endl;
//	cout << "rbi = " << endl << rbi.toMatrix() << endl;
//	cout << "rbi.m = " << rbi.m << endl;
//	cout << "rbi.h = " << rbi.h.transpose() << endl;
//	cout << "rbi.I = " << endl << rbi.I << endl;

	CHECK_ARRAY_CLOSE (
			spatial_inertia.data(),
			rbi.toMatrix().data(),
			36,
			TEST_PREC
			);
}
