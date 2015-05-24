/*
 * This is a horribly ugly test code to get a cube spinning when the IMU moves.
 * I checked it in to have as a backup to test if the IMU works as wanted...
 * I think it only compiles on mac (because GLUT not GL) (works on Yosemite 2015-05-24)
 *
 * Most of it is taken from an old demo code I found, I left the original header 
 * (GPL) as is below.
 * 
 * - Torkel Danielsson
 */


/* ============================================================================
**
** Demonstration of spinning cube
** Copyright (C) 2005  Julien Guertault
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
**
** ========================================================================= */


#include  <iostream>
#include  <stdlib.h>
#include	<GLUT/glut.h>
#include	<math.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imu_receiver.h"

quaternion_t quaternion;
IMUReceiver imuReceiver;

void normaliseQuaternion(quaternion_t *q) {
  float mag2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
  float mag = sqrt(mag2);
  q->w /= mag;
  q->x /= mag;
  q->y /= mag;
  q->z /= mag;
}

glm::mat4 matrixFromQuaternion(quaternion_t q) {
  normaliseQuaternion(&q);
  float x2 = q.x * q.x;
  float y2 = q.y * q.y;
  float z2 = q.z * q.z;
  float xy = q.x * q.y;
  float xz = q.x * q.z;
  float yz = q.y * q.z;
  float wx = q.w * q.x;
  float wy = q.w * q.y;
  float wz = q.w * q.z;
 
  // This calculation would be a lot more complicated for non-unit length quaternions
  // Note: The constructor of Matrix4 expects the Matrix in column-major format like expected by
  //   OpenGL
  return glm::mat4( 1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f,
      2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f,
      2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f);
}

/*
** Function called to update rendering
*/
void		DisplayFunc(void)
{
  static float alpha = 0;
  
  quaternion = imuReceiver.getQuaternion();  
  glm::quat rotation;
  rotation = glm::quat(quaternion.w, quaternion.x, quaternion.y, quaternion.z); 
  //std::cout << "quaternion: " << rotation.w << " " << rotation.x << " " << rotation.y << " " << rotation.z << std::endl;
  rotation = glm::normalize(rotation);
  //std::cout << "quaternion: " << rotation.w << " " << rotation.x << " " << rotation.y << " " << rotation.z << std::endl;

  float yaw = atan2(2.0f * (quaternion.x * quaternion.y + quaternion.w * quaternion.z), quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z);   
  float pitch = -asin(2.0f * (quaternion.x * quaternion.z - quaternion.w * quaternion.y));
  float roll = atan2(2.0f * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  pitch *= 180.0f / 3.14159265f;
  yaw *= 180.0f / 3.14159265f; 
  roll *= 180.0f / 3.14159265f;

  glm::mat4 rotMatrix = glm::mat4_cast(rotation);

  /* Clear the buffer, clear the matrix */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(0, 0, -10);

  GLfloat m[16];
  glGetFloatv (GL_MODELVIEW_MATRIX, m);
  glm::mat4 modelViewMatrix = glm::make_mat4(m);

  glLoadMatrixf(glm::value_ptr(modelViewMatrix * rotMatrix));

  /* We tell we want to draw quads */
  glBegin(GL_QUADS);
  
  /* Every four calls to glVertex, a quad is drawn */
  glColor3f(0, 0, 0); glVertex3f(-1, -1, -1);
  glColor3f(0, 0, 1); glVertex3f(-1, -1,  1);
  glColor3f(0, 1, 1); glVertex3f(-1,  1,  1);
  glColor3f(0, 1, 0); glVertex3f(-1,  1, -1);
  
  glColor3f(1, 0, 0); glVertex3f( 1, -1, -1);
  glColor3f(1, 0, 1); glVertex3f( 1, -1,  1);
  glColor3f(1, 1, 1); glVertex3f( 1,  1,  1);
  glColor3f(1, 1, 0); glVertex3f( 1,  1, -1);

  glColor3f(0, 0, 0); glVertex3f(-1, -1, -1);
  glColor3f(0, 0, 1); glVertex3f(-1, -1,  1);
  glColor3f(1, 0, 1); glVertex3f( 1, -1,  1);
  glColor3f(1, 0, 0); glVertex3f( 1, -1, -1);

  glColor3f(0, 1, 0); glVertex3f(-1,  1, -1);
  glColor3f(0, 1, 1); glVertex3f(-1,  1,  1);
  glColor3f(1, 1, 1); glVertex3f( 1,  1,  1);
  glColor3f(1, 1, 0); glVertex3f( 1,  1, -1);

  glColor3f(0, 0, 0); glVertex3f(-1, -1, -1);
  glColor3f(0, 1, 0); glVertex3f(-1,  1, -1);
  glColor3f(1, 1, 0); glVertex3f( 1,  1, -1);
  glColor3f(1, 0, 0); glVertex3f( 1, -1, -1);

  glColor3f(0, 0, 1); glVertex3f(-1, -1,  1);
  glColor3f(0, 1, 1); glVertex3f(-1,  1,  1);
  glColor3f(1, 1, 1); glVertex3f( 1,  1,  1);
  glColor3f(1, 0, 1); glVertex3f( 1, -1,  1);

  /* No more quads */
  glEnd();

  /* Rotate a bit more */
  alpha = alpha + 0.1;

  /* End */
  glFlush();
  glutSwapBuffers();

  /* Update again and again */
  glutPostRedisplay();
}

/*
** Function called when the window is created or resized
*/
void		ReshapeFunc(int width, int height)
{
  glMatrixMode(GL_PROJECTION);

  glLoadIdentity();
  gluPerspective(20, width / (float) height, 5, 15);
  glViewport(0, 0, width, height);

  glMatrixMode(GL_MODELVIEW);
  glutPostRedisplay();
}

/*
** Function called when a key is hit
*/
void		KeyboardFunc(unsigned char key, int x, int y)
{
  int foo;

  foo = x + y; /* Has no effect: just to avoid a warning */
  if ('q' == key || 'Q' == key || 27 == key)
      exit(0);
}


int		main(int argc, char **argv)
{
  imuReceiver.run();


  /* Creation of the window */
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(500, 500);
  glutCreateWindow("Spinning cube");

  /* OpenGL settings */
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);

  /* Declaration of the callbacks */
  glutDisplayFunc(&DisplayFunc);
  glutReshapeFunc(&ReshapeFunc);
  glutKeyboardFunc(&KeyboardFunc);

  /* Loop */
  glutMainLoop();

  /* Never reached */
  return 0;
}

/* ========================================================================= */
