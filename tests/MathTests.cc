#include <UnitTest++.h>

#include "Logging.h"
#include "mathutils.h"
#include <iostream>

const double TEST_PREC = 1.0e-14;

using namespace std;

struct MathFixture {
};

TEST (GaussElimPivot) {
	ClearLogOutput();

	cmlMatrix A;
	A.resize(3,3);
	cmlVector b(3);
	cmlVector x(3);

	A.set(0, 2, 1,
			1, 1, 5,
			0, 0, 1);
	b.set(1,2,3);

	cmlVector test_result (3);
	test_result.set (-12, -1, 3);

	LinSolveGaussElimPivot (A, b, x);

	CHECK_ARRAY_CLOSE (test_result.data(), x.data(), 3, TEST_PREC);

	A.set(0, -2, 1,
			1, 1, 5,
			0, 0, 1);

	LinSolveGaussElimPivot (A, b, x);
	test_result.set(-14, 1, 3);

	CHECK_ARRAY_CLOSE (test_result.data(), x.data(), 3, TEST_PREC);
}
