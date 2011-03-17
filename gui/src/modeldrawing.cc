#include "Model.h"
#include "Joint.h"
#include "Body.h"

#include <sstream>

#include <GL/gl.h>
#include <QDebug>

#include "glprimitives.h"

using namespace RigidBodyDynamics;

void compute_body_center_and_dimensions (Model* model, unsigned int body_id, Vector3d &body_center, Vector3d &body_dimensions) {
	int j;

	// draw the body as a green box that extends from the origin to the
	// next joint
	Vector3d body_min (0., 0., 0.);
	Vector3d body_max (0., 0., 0.);

	if (model->mu.at(body_id).size() == 0) {
		// if there is no child we assume the body to extend to the COM.
		for (j = 0; j < 3; j++) {
			body_min[j] = std::min(body_min[j], model->mBodies[body_id].mCenterOfMass[j]);	
			body_max[j] = std::max(body_max[j], model->mBodies[body_id].mCenterOfMass[j]);	
		}
	}
	else {
		// if there are one or more children, we compute the body
		// dimensions such that the boundaries reach to the joint origins
		for (j = 0; j < model->mu.at(body_id).size(); j++) {
			unsigned int child_id = model->mu[body_id][j];
			Vector3d child_translation = model->X_T[child_id].get_translation();

			int k;
			for (k = 0; k < 3; k++) {
				body_min[k] = std::min (body_min[k], child_translation[k]);
				body_max[k] = std::max (body_max[k], child_translation[k]);
			}
		}
	}

	body_dimensions = body_max - body_min;
	for (j = 0; j < 3; j++) {
		assert (body_dimensions[j] >= 0.);
		if (body_dimensions[j] == 0.) {
			body_min[j] = -0.1;
			body_max[j] = 0.1;
			body_dimensions[j] = 0.2;
		}
	}

	body_center = body_dimensions * 0.5 - Vector3d (0.1, 0.1, 0.1);

	for (j = 0; j < 3; j++) {
		if (body_dimensions[j] != 0.2) {
			body_dimensions[j] -= 0.2;
			body_center[j] += 0.1;
		}
	}
}

void draw_model (Model* model) {
	glLineWidth (1.);

	glDisable(GL_LIGHTING);
	glBegin (GL_LINES);
	glColor3f (1., 0., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (1., 0., 0.);
	glColor3f (0., 1., 0.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 1., 0.);
	glColor3f (0., 0., 1.);
	glVertex3f (0., 0., 0.);
	glVertex3f (0., 0., 1.);
	glEnd();
	glEnable(GL_LIGHTING);

	glEnable (GL_DEPTH_TEST);

	unsigned int i;
	for (i = 1; i < model->q.size(); i++) {
		Matrix3d rotation = model->GetBodyWorldOrientation(i);
		Vector3d translation = model->GetBodyOrigin(i);
		VisualizationPrimitive* body_visualization = model->GetBodyVisualizationPrimitive(i);

		std::ostringstream model_X;
		model_X << model->X_base[i];

		glPushMatrix();
		GLfloat orientation[16];
		int j,k;
		for (j = 0; j < 4; j++) {
			for (k = 0; k < 4; k++) {
				if (j < 3 && k < 3)
					orientation[j * 4 + k] = rotation(j,k);
				else 
					orientation[j * 4 + k] = 0.;
			} 
		}
		orientation[3 * 4 + 3] = 1.;

		// draw an orientation system of the current body
		glTranslated (translation[0], translation[1], translation[2]);
		glMultMatrixf(orientation);

		// Draw the joint
		/// \todo add visualizations for prismatic joints
		glPushMatrix();
		// align the orientation such that the Z-axis aligns with the joint axis
		SpatialAlgebra::SpatialVector joint_axis (model->S.at(i));
		Vector3d target_direction (joint_axis[0], joint_axis[1], joint_axis[2]);

		target_direction.normalize();
		float rot_angle = acos (cml::dot(target_direction, Vector3d (0., 0., 1.)));
		rot_angle = rot_angle;
		if (fabs(rot_angle) > 0.0001) {
			Vector3d rotation_axis = cml::cross (target_direction, Vector3d (0., 0., 1.));
			glRotatef (rot_angle * 180./M_PI,
					rotation_axis[0],
					rotation_axis[1],
					rotation_axis[2]
					);
		}
		glColor3f (1., 0., 0.);
		glScalef (0.07, 0.07, 0.2);
		glprimitives_torus();
		glPopMatrix ();

		// Draw a small coordinate system
		glDisable(GL_LIGHTING);
		glBegin (GL_LINES);
		glColor3f (0.8, 0.2, 0.2);
		glVertex3f (0., 0., 0.);
		glVertex3f (1., 0., 0.);
		glColor3f (0.2, 0.8, 0.2);
		glVertex3f (0., 0., 0.);
		glVertex3f (0., 1., 0.);
		glColor3f (0.2, 0.2, 0.8);
		glVertex3f (0., 0., 0.);
		glVertex3f (0., 0., 1.);
		glEnd();
		glEnable(GL_LIGHTING);

		// Draw the body
		if (body_visualization) {
			Vector3d body_center, body_dimensions;

			if (body_visualization->type == VisualizationPrimitiveBox) {
				Vector3d box_size = body_visualization->max - body_visualization->min;
				glPushMatrix();
				glTranslated (
						body_visualization->min[0] + box_size[0] * 0.5,
						body_visualization->min[1] + box_size[1] * 0.5,
						body_visualization->min[2] + box_size[2] * 0.5
						);
				glScaled (
						box_size[0] * 0.5,
						box_size[1] * 0.5,
						box_size[2] * 0.5
						);

				glEnable(GL_COLOR_MATERIAL);
				glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
				glColor3f (
						body_visualization->color[0],
						body_visualization->color[1],
						body_visualization->color[2]
						);
				glprimitives_cube();
				glColor3f (1.0f, 1.0f, 1.0f);
				glDisable(GL_COLOR_MATERIAL);

				glPopMatrix();
			} else if (body_visualization->type == VisualizationPrimitiveSphere) {
				glPushMatrix();
				glTranslated (
						body_visualization->center[0],
						body_visualization->center[1],
						body_visualization->center[2]
						);
				glScaled (
						body_visualization->radius,
						body_visualization->radius,
						body_visualization->radius
						);

				glEnable(GL_COLOR_MATERIAL);
				glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
				glColor3f (
						body_visualization->color[0],
						body_visualization->color[1],
						body_visualization->color[2]
						);
				glprimitives_sphere();
				glColor3f (1.0f, 1.0f, 1.0f);
				glDisable(GL_COLOR_MATERIAL);

				glPopMatrix();
			}

			// draw the COM as a small cube of red color (we ignore the depth
			// buffer for better visibility 
			glTranslated (
					model->mBodies[i].mCenterOfMass[0],
					model->mBodies[i].mCenterOfMass[1],
					model->mBodies[i].mCenterOfMass[2]
					); 

			glDisable (GL_DEPTH_TEST);
			glEnable(GL_COLOR_MATERIAL);
			glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
			glColor3f (0.9, 0., 0.1);
			glScalef (0.025, 0.025, 0.025);
			glprimitives_cube();
			glColor3f (1.0f, 1.0f, 1.0f);
			glDisable(GL_COLOR_MATERIAL);

			glEnable (GL_DEPTH_TEST);
		}
		glPopMatrix();
	}
}
