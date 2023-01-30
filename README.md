# Simulating a Jello Cube

## Overview

This project simulates a jello cube as a mass-spring system. The jello cube can be influenced by bounding box/inclined plane, external force field, and real-time user drag. 

   |       Contacting Bounding Box                  |              Real-time Interaction               |           Rotated by Force Field            |
   :-----------------------------------------------:|:------------------------------------------------:|:-------------------------------------------:|
   ![Contacting Bounding Box](/docs/moveLeft.png)   | ![Real-time Interaction](/docs/userDragLong.png) | ![Rotated by Force Field](/docs/rotate.png) |


## Functionalities

### Core Accomplishments

1. **Model a cube of jello as a mass-spring system.**
The jello cube is modeled by 8x8x8 discrete mass point with equal mass, with structure/shear/bend springs connected between them.
For each mass point, calculating the Elastic Force according to Hook's Law (F=kx) and Damping Force (F=-kv) as well.
Finally, computing the acceleration of every mass point and applying either Euler or RK4 integrator to solve the next move.

2. **Simulate the dynamics of jello in an external force field.**
For a given external force field with resolution NxNxN (2 <= N <= 30), calculating the effect on the cube with tri-linear interpolation.
The points outside external force field will be clamped.

3. **Handle the collision detection and response for the jello cube hitting any of the walls of the bounding box.**
When a mass point contacts the bounding box, generating a spring with rest length 0 between the collision point on wall and this mass point, then applying resulting Elastic Force and Damping Force to make it bounce off.

### Additional Features

1. **Implement inclined plane display and collision detection.**
To display the inclined plane inside the Axis-Aligned Bounding Box (AABB), the ray-plane intersections are calculated and used as the vertices of section polygon.
Since the jello can be initialized on any side of the plane, for collision detection, the program evaluates the side of both current mass point and that of jello center.
If these two points are in different side of the plane, the program considers it as a collision and invokes collision response.

2. **Make the animation interactive.**
Implement a user-friendly UI to apply mouse-control force.
Simply using the mouse and left button to select and drag the jello.
If the jello is selected, a purple arrow will display, with origin always attached to the animated jello and heading to some point.
The force will be applied only when user release the left button.
To ease the selection, users are encourage to enable `triangle display` mode by pressing `v` first.

3. **Display coordinate.**
Press `c` to display x-axis, y-axis, and z-axis with arrows indicating the direction of increment.

4. **Display FPS.**
Press `f` to display current window resolution and estimated Frame Per Second (FPS) in console.
Notice that this may only work on Windows Executable.

5. **Display translucent jello cube and inclined plane.**
To make the simulation more fancy, the program can display a translucent jello cube and inclined plane by adjusting alpha channel and enabling blending.

## How to run

Both Windows (x86, Win 11) and macOS (x86, 12.6) executable are tested. Executable files (`jello.exe` or `jello`) are provided.

On Windows:
```powershell
.\jello.exe <PATH_TO_WORLD_FILE>

# Example: 
# .\jello.exe .\world\jello.w
```

On macOS:
```shell
./jello <PATH_TO_WORLD_FILE>

# Example:
# ./jello ./world/gravity.w
```

## Acknowledge

The starter code (_i.e._, API and window) comes from course [CSCI 520 Computer Animation and Simulation](https://viterbi-web.usc.edu/~jbarbic/cs520-s23/), taught by [Professor Jernej Barbič](http://viterbi-web.usc.edu/~jbarbic/).