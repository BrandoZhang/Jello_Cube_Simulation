/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/


#ifndef _SHOWCUBE_H_
#define _SHOWCUBE_H_

void showCube(struct world * jello);

void showBoundingBox();

void drawArrow(struct point origin, struct point destination, float headWidth, float headLength, GLfloat red, GLfloat green, GLfloat blue);

void showCoordinate();

void showInclinedPlane(struct world * jello);

#endif
