/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"

int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

void showCube(struct world * jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;

  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }

  
  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\

 
  if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    for (i=0; i<=7; i++)
      for (j=0; j<=7; j++)
        for (k=0; k<=7; k++)
        {
          if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
            continue;

          glBegin(GL_POINTS); // draw point
            glColor4f(0,0,0,0);  
            glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
          glEnd();

          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;

          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
            glColor4f(0,0,1,1);
            PROCESS_NEIGHBOUR(1,0,0);
            PROCESS_NEIGHBOUR(0,1,0);
            PROCESS_NEIGHBOUR(0,0,1);
            PROCESS_NEIGHBOUR(-1,0,0);
            PROCESS_NEIGHBOUR(0,-1,0);
            PROCESS_NEIGHBOUR(0,0,-1);
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0,1,0,1);
            PROCESS_NEIGHBOUR(1,1,0);
            PROCESS_NEIGHBOUR(-1,1,0);
            PROCESS_NEIGHBOUR(-1,-1,0);
            PROCESS_NEIGHBOUR(1,-1,0);
            PROCESS_NEIGHBOUR(0,1,1);
            PROCESS_NEIGHBOUR(0,-1,1);
            PROCESS_NEIGHBOUR(0,-1,-1);
            PROCESS_NEIGHBOUR(0,1,-1);
            PROCESS_NEIGHBOUR(1,0,1);
            PROCESS_NEIGHBOUR(-1,0,1);
            PROCESS_NEIGHBOUR(-1,0,-1);
            PROCESS_NEIGHBOUR(1,0,-1);

            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)
            PROCESS_NEIGHBOUR(1,1,-1)
            PROCESS_NEIGHBOUR(-1,1,-1)
            PROCESS_NEIGHBOUR(-1,-1,-1)
            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(1,0,0,1);
            PROCESS_NEIGHBOUR(2,0,0);
            PROCESS_NEIGHBOUR(0,2,0);
            PROCESS_NEIGHBOUR(0,0,2);
            PROCESS_NEIGHBOUR(-2,0,0);
            PROCESS_NEIGHBOUR(0,-2,0);
            PROCESS_NEIGHBOUR(0,0,-2);
          }           
          glEnd();
        }
    glEnable(GL_LIGHTING);
  }
  
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL); 
    
    for (face=1; face <= 6; face++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
      
      if ((face==1) || (face==3) || (face==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;
      

      for (i=0; i <= 7; i++) // reset buffers
        for (j=0; j <= 7; j++)
        {
          normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
          counter[i][j]=0;
        }

      /* process triangles, accumulate normals for Gourad shading */
  
      for (i=0; i <= 6; i++)
        for (j=0; j <= 6; j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

      
        /* the actual rendering */
        for (j=1; j<=7; j++) 
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation
         
          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<=7; i++)
          {
            glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
              normal[i][j].z / counter[i][j]);
            glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
            glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
              normal[i][j-1].z / counter[i][j-1]);
            glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
          }
          glEnd();
        }
        
        
    }  
  } // end for loop over faces
  glFrontFace(GL_CCW);
}

void showBoundingBox()
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  // front face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,-2,-2);
    glVertex3f(i,-2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(2,-2,j);
  }

  // back face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,2,-2);
    glVertex3f(i,2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,2,j);
    glVertex3f(2,2,j);
  }

  // left face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(-2,i,-2);
    glVertex3f(-2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(-2,2,j);
  }

  // right face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(2,i,-2);
    glVertex3f(2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(2,-2,j);
    glVertex3f(2,2,j);
  }
  
  glEnd();

  return;
}

/* Draws 3D arrow */
void drawArrow(struct point origin, struct point destination, float headWidth, float headLength, GLfloat red, GLfloat green, GLfloat blue)
{
    double length;
    struct point arrow, upVec, rightVec, arrowHeadLeft, arrowHeadRight, temp;
    GLfloat prevColor[4];
    
    /* Calculates the vertex coordinates of arrow heads */
    pMAKE(0.0, 1.0, 0.0, upVec);  // up vector of the world
    pDIFFERENCE(destination, origin, arrow);
    pNORMALIZE(arrow);
    CROSSPRODUCTp(arrow, upVec, rightVec);
    if (rightVec.x == 0.0 && rightVec.y == 0.0 && rightVec.z == 0)
    {
        pMAKE(1.0, 0.0, 0.0, rightVec);  // hard fix
    }

    pMULTIPLY(arrow, headLength, arrowHeadRight);
    pDIFFERENCE(destination, arrowHeadRight, arrowHeadRight);
    pMULTIPLY(rightVec, headWidth, temp);
    pSUM(arrowHeadRight, temp, arrowHeadRight);
    pMULTIPLY(temp, -2.0, temp);
    pSUM(arrowHeadRight, temp, arrowHeadLeft);

    glGetFloatv(GL_CURRENT_COLOR, prevColor);  // fetch previous applied color
    glColor3f(red, green, blue);
    
    glDisable(GL_CULL_FACE);
    glBegin(GL_TRIANGLES);
        glVertex3d(destination.x, destination.y, destination.z);
        glVertex3d(arrowHeadLeft.x, arrowHeadLeft.y, arrowHeadLeft.z);
        glVertex3d(arrowHeadRight.x, arrowHeadRight.y, arrowHeadRight.z);
    glEnd();
    glBegin(GL_LINES);
        glVertex3d(origin.x, origin.y, origin.z);
        glVertex3d(destination.x, destination.y, destination.z);
    glEnd();
    glColor3fv(prevColor);  // restores previous color
    glEnable(GL_CULL_FACE);
}


/* Displays the positive x-axis, y-axis, and z-axis in current coordinate.
 * Refer to `openGLHelloWorld.cpp`
 */
void showCoordinate()
{
    double prevLineWidth;
    glGetDoublev(GL_LINE_WIDTH, &prevLineWidth);  // record original line width

    glLineWidth(4.0);  // set axes line width
    struct point origin, xPositive, yPositive, zPositive;
    pMAKE(0.0, 0.0, 0.0, origin);
    pMAKE(1.0, 0.0, 0.0, xPositive);
    pMAKE(0.0, 1.0, 0.0, yPositive);
    pMAKE(0.0, 0.0, 1.0, zPositive);
    /* x-axis */

    drawArrow(origin, xPositive, 0.1, 0.1, 1.0, 0.0, 0.0);  // Red
    /* y-axis */
    drawArrow(origin, yPositive, 0.1, 0.1, 0.0, 1.0, 0.0);  // Green
    /* z-axis */
    drawArrow(origin, zPositive, 0.1, 0.1, 0.0, 0.0, 1.0);  // Blue

    glLineWidth(prevLineWidth);  // restore line width setting
    return;
}

/* Computes the intersection point of a segment with a plane.
 * If there's only one intersection, return true; otherwise, return false.
 */
bool segmentPlaneIntersection(double a, double b, double c, double d, struct point segOrig, struct point segDest, int & ptIdx, struct point * intersections)
{
    double denominator, signedDist;
    pDIFFERENCE(segDest, segOrig, segDest);  // segDest becomes a vector pointing from segOrig to original segDest
    denominator = a * segDest.x + b * segDest.y + c * segDest.z;

    /* If the segment is parallel or on the plane, return false.
     * Even if the segment is on the plane, for this bounding box scenario, it's fine.
     */
    if (denominator == 0)
        return false;

    signedDist = - (a * segOrig.x + b * segOrig.y + c * segOrig.z + d) / denominator;

    /* If the intersection is outside the segment */
    if (signedDist < 0.0 || signedDist > 1.0)
        return false;

    struct point intersectPt;
    pMULTIPLY(segDest, signedDist, segDest);
    pSUM(segOrig, segDest, intersectPt);
    pCPY(intersectPt, intersections[ptIdx]);
    ptIdx++;
    return true;
}

/* Displays the inclined plane */
void showInclinedPlane(struct world * jello)
{
    int numIntersection = 0;
    struct point segOrigin, segDest, tempPt;
    struct point intersections[12];  // maximum number of possible intersections is 6

    /* Evaluates intersections on 12 edges */
    pMAKE(-2.0, -2.0, -2.0, segOrigin);

    pMAKE(-2.0, -2.0, 2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(-2.0, 2.0, -2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(2.0, -2.0, -2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);

    pMAKE(2.0, 2.0, -2.0, segOrigin);

    pMAKE(-2.0, 2.0, -2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(2.0, -2.0, -2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(2.0, 2.0, 2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);

    pMAKE(-2.0, 2.0, 2.0, segOrigin);

    pMAKE(-2.0, -2.0, 2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(2.0, 2.0, 2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(-2.0, 2.0, -2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);

    pMAKE(2.0, -2.0, 2.0, segOrigin);

    pMAKE(-2.0, -2.0, 2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(2.0, 2.0, 2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);
    pMAKE(2.0, -2.0, -2.0, segDest);
    segmentPlaneIntersection(jello->a, jello->b, jello->c, jello->d, segOrigin, segDest, numIntersection, intersections);


    if (numIntersection == 0)
        return;

    // TODO: Sort intersections?


    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_POLYGON);
    glColor4f(1.0, 0.0, 0.0, 0.6);  // red and translucent
    for (int i = 0; i < numIntersection; ++i)
    {
        glVertex3d(intersections[i].x, intersections[i].y, intersections[i].z);
    }
    glEnd();
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
}