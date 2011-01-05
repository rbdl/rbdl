#include "Model.h"
#include "Joint.h"
#include "Body.h"

#include <sstream>

#include <GL/gl.h>
#include <QDebug>

#include "glprimitives.h"

void draw_model (Model* model) {
	glLineWidth (1.);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

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

	glColor3f (1., 1., 1.);
	glDisable(GL_COLOR_MATERIAL);
	
	unsigned int i;
	for (i = 1; i < model->q.size(); i++) {
		// Draw only bodies with masses
		if (model->mBodies[i].mMass != 0) {
			Matrix3d rotation = model->X_base[i].get_rotation().transpose();
			Vector3d translation = rotation * model->X_base[i].get_translation() * -1.;
			rotation = rotation.transpose();
			
			std::ostringstream model_X;
			model_X << model->X_base[i];

			qDebug() << "body " << i << ": " << model_X.str().c_str();
	//		qDebug() << "i = " << i << " translation = " << translation[0] << translation[1] << translation[2];	

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
	
			glBegin (GL_LINES);
			glVertex3d (0., 0., 0.);
			glVertex3d (translation[0], translation[1], translation[2]);
			glEnd ();
	
			glTranslated (translation[0], translation[1], translation[2]);
			glMultMatrixf(orientation);

			glBegin (GL_LINES);
			glColor3f (0.8, 0.8, 0.8);
			glVertex3f (0., 0., 0.);
			glVertex3f (1., 0., 0.);
			glVertex3f (0., 0., 0.);
			glVertex3f (0., 1., 0.);
			glVertex3f (0., 0., 0.);
			glVertex3f (0., 0., 1.);
			glEnd();

			glEnable(GL_COLOR_MATERIAL);
			glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
			glColor3f (static_cast<GLfloat>(i) * 0.3, 0., 1 - static_cast<GLfloat>(i) * 0.3);
			glScalef (0.2, 0.2, 0.2);
			glprimitives_cube();
			glColor3f (1.0f, 1.0f, 1.0f);
			glDisable(GL_COLOR_MATERIAL);

			glPopMatrix();
		}
	}
}
