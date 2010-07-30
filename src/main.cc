#include <iostream>

#include "featherstone.h"

using namespace std;

int main (int argc, char* argv[]) {
	vec3d translation;
	
	translation.x = 0;
	translation.y = -1;
	translation.z = 0;

	vec3d axis;
	axis.x = 1.;
	axis.y = 0.;
	axis.z = 0.;
	
	Body* thigh = RootBody.CreateChild (RevoluteJoint, axis);
	thigh->mOriginTranslation[0] = 0.;
	thigh->mOriginTranslation[1] = 0.;
	thigh->mOriginTranslation[2] = 0.;

	Body* shank = thigh->CreateChild (RevoluteJoint, axis);
	shank->mOriginTranslation[0] = 0.;
	shank->mOriginTranslation[1] = 1.;
	shank->mOriginTranslation[2] = 0.;

	RootBody.Print ("Root");
	thigh->Print ("Thigh");
	shank->Print ("Shank");

	return 0;
}
