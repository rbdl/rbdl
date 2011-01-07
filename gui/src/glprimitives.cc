#include "glprimitives.h"

#include <cmath>
#include <iostream>
#include <cstdlib>

#define GL_GLEXT_PROTOTYPES

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>

enum BufferIndex {
	BufferIndexCubePosition = 0,
	BufferIndexCubeColor,
	BufferIndexCubeIndex,
	BufferIndexCubeNormal,
	BufferIndexDiscPosition,
	BufferIndexDiscNormal,
	BufferIndexTorusPosition,
	BufferIndexTorusIndex,
	BufferIndexTorusNormal,
	BufferIndexLast
};

static GLuint BufferNames[BufferIndexLast];

const unsigned int disc_slices = 12;

static GLsizeiptr disc_position_size = 0;
static GLfloat *disc_position_data = NULL;
static GLsizeiptr disc_normal_size = 0;
static GLfloat *disc_normal_data = NULL;
static GLsizei disc_element_count = 0;

static GLsizeiptr torus_position_size = 0;
static GLfloat *torus_position_data = NULL;
static GLsizeiptr torus_normal_size = 0;
static GLfloat *torus_normal_data = NULL;
static GLsizei torus_element_count = 0;
	
GLsizeiptr cube_position_size  = 8 * 3 * sizeof (GLfloat);
GLfloat cube_position_data[] = {
	-1.0f, -1.0f,  1.0f,
	 1.0f, -1.0f,  1.0f,
	 1.0f,  1.0f,  1.0f,
	-1.0f,  1.0f,  1.0f,
	-1.0f, -1.0f, -1.0f,
	 1.0f, -1.0f, -1.0f,
	 1.0f,  1.0f, -1.0f,
	-1.0f,  1.0f, -1.0f
};

GLsizeiptr cube_color_size = 8 * 3 * sizeof (GLubyte);
GLubyte cube_color_data[] = {
	 27, 227,  27,
	 27,  27, 227,
	227,  27,  27,
	 27, 227, 227,
	227, 227,  27,
	227,  27, 227,
	227, 227, 227,
	 27,  27,  27
};

GLsizeiptr cube_index_size = 6 * 4 * sizeof (GLubyte);
GLubyte cube_index_data[] = {
	0, 1, 2, 3,
	6, 2, 1, 5,
	7, 6, 5, 4,
	3, 7, 4, 0,
	3, 2, 6, 7,
	0, 4, 5, 1
};

GLsizeiptr cube_normal_size = 6 * 3 * sizeof (GLfloat);
GLfloat cube_normal_data[] = {
	 0.0f,  0.0f,  1.0f,
	 1.0f,  0.0f,  0.0f,
	 0.0f,  0.0f, -1.0f,
	-1.0f,  0.0f,  0.0f,
	 0.0f,  1.0f,  0.0f,
	 0.0f, -1.0f,  0.0f,
};

void check_opengl_error (const char* file, int line) {
	GLenum gl_error = glGetError();
	if (gl_error) {
		std::cerr << "OpenGL Error: " << gluErrorString (gl_error) << " at " << file << ":" << line <<  std::endl;
		exit(1);
	}
}

void glprimitives_cube_init () {
	// Vertices
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexCubePosition]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, cube_position_size, cube_position_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);

	// Color
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexCubeColor]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, cube_color_size, cube_color_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);

	// Indexes
	glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, BufferNames[BufferIndexCubeIndex]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ELEMENT_ARRAY_BUFFER, cube_index_size, cube_index_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);

	// Normals
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexCubeNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, cube_normal_size, cube_normal_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);
}

void glprimitives_disc_init() {
	disc_element_count = (disc_slices + 2);
	disc_position_size = disc_element_count * 3 * sizeof (GLfloat);
	disc_position_data = new GLfloat[disc_element_count * 3];

	disc_normal_size = disc_element_count * 3 * sizeof (GLfloat);
	disc_normal_data = new GLfloat[disc_element_count * 3];

	disc_position_data[0] = 0.f;
	disc_position_data[1] = 0.f;
	disc_position_data[2] = 0.f;

	disc_normal_data[0] = 0.f;
	disc_normal_data[1] = 0.f;
	disc_normal_data[2] = 1.f;

	unsigned int i;
	GLfloat rad_inc = static_cast<GLfloat> (M_PI * 2 / static_cast<GLfloat> (disc_slices));
	for (i = 1; i <= disc_slices; i++) {
		GLfloat rad_val = static_cast<GLfloat> (i - 1) * rad_inc;
		GLfloat c_val = cos (rad_val);
		GLfloat s_val = sin (rad_val);
		disc_position_data[i * 3] = c_val;
		disc_position_data[i * 3 + 1] = s_val;
		disc_position_data[i * 3 + 2] = 0.;

		disc_normal_data[i * 3] = 0.f;
		disc_normal_data[i * 3 + 1] = 0.f;
		disc_normal_data[i * 3 + 2] = 1.f;
	}

	disc_position_data[i * 3] = 1.;
	disc_position_data[i * 3 + 1] = 0.;
	disc_position_data[i * 3 + 2] = 0.;

	disc_normal_data[i * 3] = 0.f;
	disc_normal_data[i * 3 + 1] = 0.f;
	disc_normal_data[i * 3 + 2] = 1.f;

	// Vertices
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexDiscPosition]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, disc_position_size, disc_position_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);

	// Normals
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexDiscNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, disc_normal_size, disc_normal_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);
}

void glprimitives_disc_destroy() {
	delete[] disc_position_data;
	disc_position_data = NULL;
	delete[] disc_normal_data;
	disc_normal_data = NULL;
}

void glprimitives_torus_init() {
	torus_element_count = (disc_slices + 1) * 2;
	torus_position_size = torus_element_count * 3 * sizeof (GLfloat);
	torus_position_data = new GLfloat[torus_element_count * 3];

	torus_normal_size = torus_element_count * 3 * sizeof (GLfloat);
	torus_normal_data = new GLfloat[torus_element_count * 3];

	unsigned int i;
	GLfloat rad_inc = static_cast<GLfloat> (M_PI * 2 / static_cast<GLfloat> (disc_slices));
	for (i = 0; i < (torus_element_count - 2); i = i + 2) {
		GLfloat rad_val = static_cast<GLfloat> (i) * rad_inc * 0.5f;
		GLfloat c_val = cos (rad_val);
		GLfloat s_val = sin (rad_val);

		torus_position_data[i * 3] = c_val;
		torus_position_data[i * 3 + 1] = s_val;
		torus_position_data[i * 3 + 2] = 0.5;

		torus_position_data[i * 3 + 3] = c_val;
		torus_position_data[i * 3 + 4] = s_val;
		torus_position_data[i * 3 + 5] = -0.5;

		torus_normal_data[i * 3] = c_val;
		torus_normal_data[i * 3 + 1] = s_val;
		torus_normal_data[i * 3 + 2] = 0.f;

		torus_normal_data[i * 3 + 3] = c_val;
		torus_normal_data[i * 3 + 4] = s_val;
		torus_normal_data[i * 3 + 5] = 0.;
	}

	torus_position_data[i * 3] = 1.;
	torus_position_data[i * 3 + 1] = 0.;
	torus_position_data[i * 3 + 2] = 0.5;

	torus_normal_data[i * 3] = 1.;
	torus_normal_data[i * 3 + 1] = 0.;
	torus_normal_data[i * 3 + 2] = 0.f;

	torus_position_data[i * 3 + 3] = 1.;
	torus_position_data[i * 3 + 4] = 0.;
	torus_position_data[i * 3 + 5] = -0.5;

	torus_normal_data[i * 3 + 3] = 1.;
	torus_normal_data[i * 3 + 4] = 0.;
	torus_normal_data[i * 3 + 5] = 0.;

	// Vertices
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexTorusPosition]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, torus_position_size, torus_position_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);

	// Normals
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexTorusNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, torus_normal_size, torus_normal_data, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);
}

void glprimitives_torus_destroy() {
	delete[] torus_position_data;
	torus_position_data = NULL;
	delete[] torus_normal_data;
	torus_normal_data = NULL;
}

void glprimitives_init () {
	// initialize VBO for the cube
	glGenBuffers(BufferIndexLast, BufferNames);
	/*
	std::cout << "PositionBuffer = " << BufferNames[BufferIndexCubePosition] << std::endl;
	std::cout << "ColorBuffer = " << BufferNames[BufferIndexCubeColor] << std::endl;
	std::cout << "IndexBuffer = " << BufferNames[BufferIndexCubeIndex] << std::endl;
	std::cout << "NormalBuffer = " << BufferNames[BufferIndexCubeNormal] << std::endl;
	*/
	check_opengl_error (__FILE__, __LINE__);

	glprimitives_cube_init();
	glprimitives_disc_init();
	glprimitives_torus_init();
}

void glprimitives_destroy () {
	glprimitives_disc_destroy();
	glDeleteBuffers(BufferIndexLast, BufferNames);
}

void glprimitives_cube () {
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexCubePosition]);
	check_opengl_error (__FILE__, __LINE__);
	glVertexPointer (3, GL_FLOAT, 0, 0);
	check_opengl_error (__FILE__, __LINE__);
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexCubeColor]);
	check_opengl_error (__FILE__, __LINE__);
	glColorPointer (3, GL_UNSIGNED_BYTE, 0, 0);
	check_opengl_error (__FILE__, __LINE__);
	glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, BufferNames[BufferIndexCubeIndex]);
	check_opengl_error (__FILE__, __LINE__);
	glIndexPointer(GL_UNSIGNED_BYTE, 0, (void *) 0);

	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexCubeNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glNormalPointer (GL_FLOAT, 0, 0);
	check_opengl_error (__FILE__, __LINE__);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_NORMAL_ARRAY);
//	glEnableClientState (GL_COLOR_ARRAY);

	glDrawElements(GL_QUADS, 36, GL_UNSIGNED_BYTE, 0);
//	glDrawArrays (GL_LINES, 0, 8);

	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_NORMAL_ARRAY);
//	glDisableClientState (GL_COLOR_ARRAY);
	glDisable(GL_COLOR_MATERIAL);

	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void glprimitives_disc () {
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexDiscPosition]);
	check_opengl_error (__FILE__, __LINE__);
	glVertexPointer (3, GL_FLOAT, 0, 0);
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexDiscNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glNormalPointer (GL_FLOAT, 0, 0);
	check_opengl_error (__FILE__, __LINE__);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_NORMAL_ARRAY);

	glDrawArrays (GL_TRIANGLE_FAN, 0, disc_element_count);

	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_NORMAL_ARRAY);
	glDisableClientState (GL_COLOR_ARRAY);
	glDisable(GL_COLOR_MATERIAL);

	glBindBuffer (GL_ARRAY_BUFFER, 0);
}

void glprimitives_torus () {
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexTorusPosition]);
	check_opengl_error (__FILE__, __LINE__);
	glVertexPointer (3, GL_FLOAT, 0, 0);
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexTorusNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glNormalPointer (GL_FLOAT, 0, 0);
	check_opengl_error (__FILE__, __LINE__);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_NORMAL_ARRAY);

	glDrawArrays (GL_QUAD_STRIP, 0, torus_element_count);

	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_NORMAL_ARRAY);
	glDisable(GL_COLOR_MATERIAL);

	glBindBuffer (GL_ARRAY_BUFFER, 0);

	glPushMatrix();
	glTranslatef (0., 0., 0.5);
	glprimitives_disc();
	glTranslatef (0., 0., -1.);
	glRotatef (180, 0, 1., 0.);
	glprimitives_disc();
	glPopMatrix();
}
