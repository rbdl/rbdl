#include "glprimitives.h"

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
	BufferIndexLast
};

static GLuint BufferNames[BufferIndexLast];

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

void glprimitives_init () {
	// initialize VBO for the cube
	glGenBuffers(BufferIndexLast, BufferNames);
	std::cout << "PositionBuffer = " << BufferNames[BufferIndexCubePosition] << std::endl;
	std::cout << "ColorBuffer = " << BufferNames[BufferIndexCubeColor] << std::endl;
	std::cout << "IndexBuffer = " << BufferNames[BufferIndexCubeIndex] << std::endl;
	std::cout << "NormalBuffer = " << BufferNames[BufferIndexCubeNormal] << std::endl;
	check_opengl_error (__FILE__, __LINE__);

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
	
	std::cout << "glprimitives initialized" << std::endl;
}

void glprimitives_destroy () {
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

	static bool passed = false;
	if (!passed) {
		std::cout << "passed once" << std::endl;
		passed = true;
	}
}
