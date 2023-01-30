/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

/* Computes elastic force according to Hook's law
 */
void computeElasticForce(struct point p1, struct point p2, double kHook, double restLength, struct point & resultForce)
{
    struct point L;
    double length, scalar;
    pDIFFERENCE(p1, p2, L);  // L: vector pointing from p2 to p1
    pNORMALIZE(L);  // normalizes L, and assigns the original norm of L to `length`
    scalar = (-kHook) * (length - restLength);
    pMULTIPLY(L, scalar, resultForce);
}

/* Computes damping force */
void computeDampingForce(struct point p1, struct point p2, struct point v1, struct point v2, double kd, struct point & resultForce)
{
    struct point L, relativeVelocity;
    double length, scalar;
    pDIFFERENCE(v1, v2, relativeVelocity);  // relativeVelocity: v1 - v2
    pDIFFERENCE(p1, p2, L);  // L: vector pointing from p2 to p1
    pNORMALIZE(L);  // normalizes L, and assigns the original norm of L to `length`
    DOTPRODUCT(relativeVelocity, L, scalar);  // scalar = (v1 - v2) L/|L|
    scalar *= (-kd);  // scalar = -kd (v1 - v2) L/|L|
    pMULTIPLY(L, scalar, resultForce);
}

void processNeighbourForce(int i, int j, int k, 
                           int di, int dj, int dk, 
                           double restLength, 
                           struct world * jello, struct point & F)
{
    int ip, jp, kp;
    ip = i + di;
    jp = j + dj;
    kp = k + dk;
    if ( !( (ip > 7) || (ip < 0) || \
            (jp > 7) || (jp < 0) || \
            (kp > 7) || (kp < 0) ) )
    {
        struct point resultForce;
        pMAKE(0.0, 0.0, 0.0, resultForce);
        computeElasticForce(jello->p[i][j][k], jello->p[ip][jp][kp], jello->kElastic, restLength, resultForce);
        pSUM(F, resultForce, F);
        computeDampingForce(jello->p[i][j][k], jello->p[ip][jp][kp], jello->v[i][j][k], jello->v[ip][jp][kp], jello->dElastic, resultForce);
        pSUM(F, resultForce, F);
    }
}

void computeStructuralSpring(int i, int j, int k, struct world * jello, struct point & F)
{
    processNeighbourForce(i, j, k, 1, 0, 0, jello->restLenStruct, jello, F);
    processNeighbourForce(i, j, k, 0, 1, 0, jello->restLenStruct, jello, F);
    processNeighbourForce(i, j, k, 0, 0, 1, jello->restLenStruct, jello, F);
    processNeighbourForce(i, j, k, -1, 0, 0, jello->restLenStruct, jello, F);
    processNeighbourForce(i, j, k, 0, -1, 0, jello->restLenStruct, jello, F);
    processNeighbourForce(i, j, k, 0, 0, -1, jello->restLenStruct, jello, F);
}

void computeSheerSpring(int i, int j, int k, struct world * jello, struct point & F)
{
    /* Computes sheer spring force on faces */
    processNeighbourForce(i, j, k, 1, 1, 0, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, -1, 1, 0, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, -1, -1, 0, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, 1, -1, 0, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, 0, 1, 1, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, 0, -1, 1, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, 0, -1, -1, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, 0, 1, -1, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, 1, 0, 1, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, -1, 0, 1, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, -1, 0, -1, jello->restLenSheerFace, jello, F);
    processNeighbourForce(i, j, k, 1, 0, -1, jello->restLenSheerFace, jello, F);

    /* Computes internal sheer spring force */
    processNeighbourForce(i, j, k, 1, 1, 1, jello->restLenSheerInternal, jello, F);
    processNeighbourForce(i, j, k, -1, 1, 1, jello->restLenSheerInternal, jello, F);
    processNeighbourForce(i, j, k, -1, -1, 1, jello->restLenSheerInternal, jello, F);
    processNeighbourForce(i, j, k, 1, -1, 1, jello->restLenSheerInternal, jello, F);
    processNeighbourForce(i, j, k, 1, 1, -1, jello->restLenSheerInternal, jello, F);
    processNeighbourForce(i, j, k, -1, 1, -1, jello->restLenSheerInternal, jello, F);
    processNeighbourForce(i, j, k, -1, -1, -1, jello->restLenSheerInternal, jello, F);
    processNeighbourForce(i, j, k, 1, -1, -1, jello->restLenSheerInternal, jello, F);
}

void computeBendSpring(int i, int j, int k, struct world * jello, struct point & F)
{
    processNeighbourForce(i, j, k, 2, 0, 0, jello->restLenBend, jello, F);
    processNeighbourForce(i, j, k, 0, 2, 0, jello->restLenBend, jello, F);
    processNeighbourForce(i, j, k, 0, 0, 2, jello->restLenBend, jello, F);
    processNeighbourForce(i, j, k, -2, 0, 0, jello->restLenBend, jello, F);
    processNeighbourForce(i, j, k, 0, -2, 0, jello->restLenBend, jello, F);
    processNeighbourForce(i, j, k, 0, 0, -2, jello->restLenBend, jello, F);
}

void computeForceField(int i, int j, int k, struct world * jello, struct point & F)
{
    double unitLen, xRatio, yRatio, zRatio;
    int xIdx, yIdx, zIdx;  // round down position of jello->p in force field
    struct point F000, F100, F110, F010, F001, F101, F111, F011, temp;

    unitLen = jello->sideLenForceField / (jello->resolution - 1);

    /* Aligns the origin of jello->p (0, 0, 0) to
       that of force field (-jello->sideLenForceField / 2,
                            -jello->sideLenForceField / 2,
                            -jello->sideLenForceField / 2),
       then calculates the indices in force field */
    xIdx = int((jello->p[i][j][k].x + jello->sideLenForceField / 2) / unitLen);
    yIdx = int((jello->p[i][j][k].y + jello->sideLenForceField / 2) / unitLen);
    zIdx = int((jello->p[i][j][k].z + jello->sideLenForceField / 2) / unitLen);
    xIdx = clamp(xIdx, 0, jello->resolution - 2);
    yIdx = clamp(yIdx, 0, jello->resolution - 2);
    zIdx = clamp(zIdx, 0, jello->resolution - 2);

    /* Tri-linear interpolation */
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx, yIdx, zIdx)], F000);
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx + 1, yIdx, zIdx)], F100);
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx + 1, yIdx + 1, zIdx)], F110);
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx, yIdx + 1, zIdx)], F010);
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx, yIdx, zIdx + 1)], F001);
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx + 1, yIdx, zIdx + 1)], F101);
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx + 1, yIdx + 1, zIdx + 1)], F111);
    pCPY(jello->forceField[idxIn1D(jello->resolution, xIdx, yIdx + 1, zIdx + 1)], F011);

    xRatio = (jello->p[i][j][k].x - (unitLen * xIdx - jello->sideLenForceField / 2)) / unitLen;
    yRatio = (jello->p[i][j][k].y - (unitLen * yIdx - jello->sideLenForceField / 2)) / unitLen;
    zRatio = (jello->p[i][j][k].z - (unitLen * zIdx - jello->sideLenForceField / 2)) / unitLen;
    pMULTIPLY(F000, (1 - xRatio) * (1 - yRatio) * (1 - zRatio), F000);
    pMULTIPLY(F100, xRatio * (1 - yRatio) * (1 - zRatio), F100);
    pMULTIPLY(F110, xRatio * yRatio * (1 - zRatio), F110);
    pMULTIPLY(F010, (1 - xRatio) * yRatio * (1 - zRatio), F010);
    pMULTIPLY(F001, (1 - xRatio) * (1 - yRatio) * zRatio, F001);
    pMULTIPLY(F101, xRatio * (1 - yRatio) * zRatio, F101);
    pMULTIPLY(F111, xRatio * yRatio * zRatio, F111);
    pMULTIPLY(F011, (1 - xRatio) * yRatio * zRatio, F011);
    pMAKE(0.0, 0.0, 0.0, temp);
    pSUM(temp, F000, temp);
    pSUM(temp, F100, temp);
    pSUM(temp, F110, temp);
    pSUM(temp, F010, temp);
    pSUM(temp, F001, temp);
    pSUM(temp, F101, temp);
    pSUM(temp, F111, temp);
    pSUM(temp, F011, temp);
    pSUM(F, temp, F);
}

bool collisionDetected(int i, int j, int k, struct world* jello, struct point& pos)
{
    double maxPos = jello->sideLenForceField / 2;
    double minPos = -maxPos;

    /* Detects collision with bounding box */
    if ((jello->p[i][j][k].x < minPos) || (jello->p[i][j][k].x > maxPos)\
        || (jello->p[i][j][k].y < minPos) || (jello->p[i][j][k].y > maxPos)\
        || (jello->p[i][j][k].z < minPos) || (jello->p[i][j][k].z > maxPos))
    {
        double posX, posY, posZ;
        posX = clamp(jello->p[i][j][k].x, minPos, maxPos);
        posY = clamp(jello->p[i][j][k].y, minPos, maxPos);
        posZ = clamp(jello->p[i][j][k].z, minPos, maxPos);
        pMAKE(posX, posY, posZ, pos);
        return true;
    }

    /* Detects collision with inclined plane */
    if (jello->incPlanePresent == 1)
    {
        double signedDistJelloCenter, signedDistCurrentPt;
        // Figures out the signed distance of jello center w.r.t the inclined plane.
        signedDistJelloCenter = jello->a * (jello->p[3][3][3].x + jello->p[4][4][4].x) / 2 \
            + jello->b * (jello->p[3][3][3].y + jello->p[4][4][4].y) / 2 \
            + jello->c * (jello->p[3][3][3].z + jello->p[4][4][4].z) / 2 \
            + jello->d;  // temp use, the real signed distance should be divided by norm of (a, b, c)
        // Figures out the signed distance of current point w.r.t the inclined plane.
        signedDistCurrentPt = jello->a * jello->p[i][j][k].x \
            + jello->b * jello->p[i][j][k].y \
            + jello->c * jello->p[i][j][k].z \
            + jello->d;  // the real signed distance should be divided by norm of (a, b, c)
        // If jello center and current point are on different side,
        // i.e., they have different signs for their signed distance,
        // collision detected, compute the collision point.
        if (signedDistJelloCenter * signedDistCurrentPt < 0)  // in this program, this result will not overflow
        {
            double length;  // original length of planeNormal
            struct point planeNormal;
            pMAKE(jello->a, jello->b, jello->c, planeNormal);
            pNORMALIZE(planeNormal);
            pMULTIPLY(planeNormal, signedDistCurrentPt, planeNormal);
            pDIFFERENCE(jello->p[i][j][k], planeNormal, pos);
            return true;
        }
    }
    return false;
}

void computeCollisionForce(int i, int j, int k, struct world * jello, struct point & F)
{
    struct point collisionPos;
    if (collisionDetected(i, j, k, jello, collisionPos))
    {
        struct point zeroVelocity, resultForce;
        pMAKE(0.0, 0.0, 0.0, zeroVelocity);
        pMAKE(0.0, 0.0, 0.0, resultForce);
        computeElasticForce(jello->p[i][j][k], collisionPos, jello->kCollision, 0.0, resultForce);
        pSUM(F, resultForce, F);
        computeDampingForce(jello->p[i][j][k], collisionPos, jello->v[i][j][k], zeroVelocity, jello->dCollision, resultForce);
        pSUM(F, resultForce, F);
    }
}

void computeMouseForce(int i, int j, int k, struct world * jello, struct point & F)
{
    // skip if no mouse force
    if (mouseForceStrength.x == 0.0 \
        && mouseForceStrength.y == 0.0 \
        && mouseForceStrength.z == 0.0)
        return;

    struct point zeroVelocity, resultForce;
    pMAKE(0.0, 0.0, 0.0, zeroVelocity);
    pMAKE(0.0, 0.0, 0.0, resultForce);
    computeElasticForce(jello->p[i][j][k], mouseForceDest, jello->kMouseForce, 0.0, resultForce);
    pSUM(F, resultForce, F);
    computeDampingForce(jello->p[i][j][k], mouseForceDest, jello->v[i][j][k], zeroVelocity, jello->dMouseForce, resultForce);
    pSUM(F, resultForce, F);
}

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
    int i, j, k;

    for (i = 0; i <= 7; i++)
      for (j = 0; j <= 7; j++)
        for (k = 0; k <= 7; k++)
        {
            pMAKE(0.0, 0.0, 0.0, a[i][j][k]);
            computeCollisionForce(i, j, k, jello, a[i][j][k]);
            computeMouseForce(i, j, k, jello, a[i][j][k]);
            computeStructuralSpring(i, j, k, jello, a[i][j][k]);
            computeSheerSpring(i, j, k, jello, a[i][j][k]);
            computeBendSpring(i, j, k, jello, a[i][j][k]);
            if (jello->resolution > 0) {
                computeForceField(i, j, k, jello, a[i][j][k]);
            }

            pMULTIPLY(a[i][j][k], 1.0 / jello->mass, a[i][j][k]);
        }

    if (!(mouseForceStrength.x == 0.0 \
        && mouseForceStrength.y == 0.0 \
        && mouseForceStrength.z == 0.0))
        pMAKE(0.0, 0.0, 0.0, mouseForceStrength);  // reset to no mouse force

}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}
