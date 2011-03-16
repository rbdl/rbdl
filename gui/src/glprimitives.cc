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
	BufferIndexCubeNormal,
	BufferIndexDiscPosition,
	BufferIndexDiscNormal,
	BufferIndexTorusPosition,
	BufferIndexTorusIndex,
	BufferIndexTorusNormal,
	BufferIndexSpherePosition,
	BufferIndexSphereNormal,
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
	
GLsizeiptr cube_position_size  = 6 * 4 * 3 * sizeof (GLfloat);
GLfloat cube_position_data[] = {
	 1, 1, 1,  -1, 1, 1,  -1,-1, 1,   1,-1, 1,    // v0-v1-v2-v3
	 1, 1, 1,   1,-1, 1,   1,-1,-1,   1, 1,-1,    // v0-v3-v4-v5
	 1, 1, 1,   1, 1,-1,  -1, 1,-1,  -1, 1, 1,    // v0-v5-v6-v1
	-1, 1, 1,  -1, 1,-1,  -1,-1,-1,  -1,-1, 1,    // v1-v6-v7-v2
	-1,-1,-1,   1,-1,-1,   1,-1, 1,  -1,-1, 1,    // v7-v4-v3-v2
	 1,-1,-1,  -1,-1,-1,  -1, 1,-1,   1, 1,-1     // v4-v7-v6-v5
};   

GLsizeiptr cube_normal_size = 6 * 4 * 3 * sizeof (GLfloat);
GLfloat cube_normal_data[] = {
	 0, 0, 1,   0, 0, 1,  0, 0, 1,  0, 0, 1, // v0-v1-v2-v3
	 1, 0, 0,   1, 0, 0,  1, 0, 0,  1, 0, 0, // v0-v3-v4-v5
	 0, 1, 0,   0, 1, 0,  0, 1, 0,  0, 1, 0, // v0-v5-v6-v1
	-1, 0, 0,  -1, 0, 0, -1, 0, 0, -1, 0, 0, // v1-v6-v7-v2
	 0,-1, 0,   0,-1, 0,  0,-1, 0,  0,-1, 0, // v7-v4-v3-v2
	 0, 0,-1,   0, 0,-1,  0, 0,-1,  0, 0,-1  // v4-v7-v6-v5

};

GLfloat sphere_x = 0.525731112119133606;
GLfloat sphere_z = 0.850650808352039932;

GLfloat sphere_vertices[60][3] = {
   {sphere_x, 0.f, sphere_z  },  { 0.0, sphere_z, sphere_x }, 	{-sphere_x, 0.f, sphere_z },
   { 0.0, sphere_z, sphere_x },  {-sphere_z, sphere_x, 0.f }, 	{-sphere_x, 0.f, sphere_z }, 
   { 0.0, sphere_z, sphere_x },  {0.f, sphere_z, -sphere_x }, 	{-sphere_z, sphere_x, 0.f }, 
   { sphere_z, sphere_x, 0.f },  {0.f, sphere_z, -sphere_x }, 	{ 0.0, sphere_z, sphere_x }, 
   {sphere_x, 0.f, sphere_z  },  { sphere_z, sphere_x, 0.f }, 	{ 0.0, sphere_z, sphere_x }, 
                                                              
   {sphere_x, 0.f, sphere_z  },  {sphere_z, -sphere_x, 0.f }, 	{ sphere_z, sphere_x, 0.f },  
   {sphere_z, -sphere_x, 0.f },  {sphere_x, 0.f, -sphere_z }, 	{ sphere_z, sphere_x, 0.f }, 
   {sphere_z, sphere_x, 0.f  },  {sphere_x, 0.f, -sphere_z }, 	{0.f, sphere_z, -sphere_x }, 
   {sphere_x, 0.f, -sphere_z },  {-sphere_x, 0.f, -sphere_z}, 	{0.f, sphere_z, -sphere_x }, 
   {sphere_x, 0.f, -sphere_z },  {0.f, -sphere_z, -sphere_x}, 	{-sphere_x, 0.f, -sphere_z}, 
                                                              
   {sphere_x, 0.f, -sphere_z },  {sphere_z, -sphere_x, 0.f }, 	{0.f, -sphere_z, -sphere_x}, 
   {sphere_z, -sphere_x, 0.f },  {0.f, -sphere_z, sphere_x }, 	{0.f, -sphere_z, -sphere_x}, 
   {0.f, -sphere_z, sphere_x },  {-sphere_z, -sphere_x, 0.f}, 	{0.f, -sphere_z, -sphere_x}, 
   {0.f, -sphere_z, sphere_x },  {-sphere_x, 0.f, sphere_z }, 	{-sphere_z, -sphere_x, 0.f}, 
   {0.f, -sphere_z, sphere_x },  {sphere_x, 0.f, sphere_z  }, 	{-sphere_x, 0.f, sphere_z }, 
                                                              
   {sphere_z, -sphere_x, 0.f },  {sphere_x, 0.f, sphere_z  }, 	{0.f, -sphere_z, sphere_x }, 
   {-sphere_z, -sphere_x, 0.f},  {-sphere_x, 0.f, sphere_z }, 	{-sphere_z, sphere_x, 0.f },
   {-sphere_x, 0.f, -sphere_z},  {-sphere_z, -sphere_x, 0.f}, 	{-sphere_z, sphere_x, 0.f }, 
   {0.f, sphere_z, -sphere_x },  {-sphere_x, 0.f, -sphere_z}, 	{-sphere_z, sphere_x, 0.f }, 
   {-sphere_z, -sphere_x, 0.f},  {-sphere_x, 0.f, -sphere_z}, 	{0.f, -sphere_z, -sphere_x} 
};

GLfloat sphere_normals[60][3];

GLsizeiptr sphere_element_count = 60;
GLsizeiptr sphere_position_size = sphere_element_count * 3 * sizeof (GLfloat);
GLsizeiptr sphere_normal_size   = sphere_element_count * 3 * sizeof (GLfloat);

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

	int i;
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

void glprimitives_sphere_init () {
	// Vertices
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexSpherePosition]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, sphere_position_size, sphere_vertices, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);

	// Normals
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexSphereNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glBufferData (GL_ARRAY_BUFFER, sphere_normal_size, sphere_vertices, GL_STATIC_DRAW);
	check_opengl_error (__FILE__, __LINE__);
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
	glprimitives_sphere_init();
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
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexCubeNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glNormalPointer (GL_FLOAT, 0, 0);
	check_opengl_error (__FILE__, __LINE__);

	glEnable(GL_COLOR_MATERIAL);
//	glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_NORMAL_ARRAY);

	glEnable(GL_RESCALE_NORMAL);

	glDrawArrays (GL_QUADS, 0, 6 * 4);

	glDisable(GL_RESCALE_NORMAL);

	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_NORMAL_ARRAY);
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

	glEnable(GL_RESCALE_NORMAL);

	glDrawArrays (GL_TRIANGLE_FAN, 0, disc_element_count);

	glDisable(GL_RESCALE_NORMAL);

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

	glEnable(GL_RESCALE_NORMAL);

	glDrawArrays (GL_QUAD_STRIP, 0, torus_element_count);

	glDisable(GL_RESCALE_NORMAL);

	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_NORMAL_ARRAY);
	glDisable(GL_COLOR_MATERIAL);

	glBindBuffer (GL_ARRAY_BUFFER, 0);

	// Draw the caps (i.e. discs at the open ends)
	glPushMatrix();
		glTranslatef (0., 0., 0.5);
		glprimitives_disc();
		glTranslatef (0., 0., -1.);
		glRotatef (180, 0, 1., 0.);
		glprimitives_disc();
	glPopMatrix();
}

void glprimitives_sphere () {
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexSpherePosition]);
	check_opengl_error (__FILE__, __LINE__);
	glVertexPointer (3, GL_FLOAT, 0, 0);
	glBindBuffer (GL_ARRAY_BUFFER, BufferNames[BufferIndexSphereNormal]);
	check_opengl_error (__FILE__, __LINE__);
	glNormalPointer (GL_FLOAT, 0, 0);
	check_opengl_error (__FILE__, __LINE__);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnableClientState (GL_VERTEX_ARRAY);
	glEnableClientState (GL_NORMAL_ARRAY);

	glEnable(GL_RESCALE_NORMAL);

	glDrawArrays (GL_TRIANGLES, 0, sphere_element_count);
//	glDrawElements (GL_TRIANGLES, 20 * 3, GL_UNSIGNED_INT, NULL);
//	glDrawArrays (GL_QUAD_STRIP, 0, sphere_element_count);

	glDisable(GL_RESCALE_NORMAL);

	glDisableClientState (GL_VERTEX_ARRAY);
	glDisableClientState (GL_NORMAL_ARRAY);
	glDisableClientState (GL_COLOR_ARRAY);
	glDisable(GL_COLOR_MATERIAL);

	glBindBuffer (GL_ARRAY_BUFFER, 0);
}
