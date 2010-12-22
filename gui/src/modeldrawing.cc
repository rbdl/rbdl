#include "Model.h"
#include "Joint.h"
#include "Body.h"

#include <GL/gl.h>
#include <QDebug>

void draw_model (Model* model) {
	glLineWidth (1.);
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

	unsigned int i;
	for (i = 1; i < model->q.size(); i++) {
		// Draw only bodies with masses
		if (model->mBodies[i].mMass != 0) {
			Matrix3d rotation = model->X_base[i].get_rotation().transpose();
			Vector3d translation = rotation * model->X_base[i].get_translation() * -1.;
			rotation = rotation.transpose();
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

			glPopMatrix();
		}
	}
}
