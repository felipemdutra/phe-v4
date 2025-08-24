# PHE-v4 - Physics Engine

PHE-v4 is a C++ physics engine designed for rigid body dynamics, constraints,
and soft-body simulations, built on top of my OpenGL wrapper library for 
rendering. It aims to allow realistic simulations in real time, and 
performantly.

### Features

- Rigid Body Dynamics
    - 3D rigid bodies with mass, inertia, linear and angular motion.
    - Quaternions for rotation to avoid gimbal lock.
- Constraints and Pendulums
    - Support for any number of bobs in a pendulum, allowing for the simulation
      of chaotic systems, like double pendulums.

### Planned features

- Soft Bodies with Runge-Kutta 4 integration.

## Requirements

- C++ 17 or later
- OpenGL 4
- GLEW
- glfw3
- GLM
- WrapGL ([WrapGL GitHub page](https://github.com/felipemdutra/wrapgl.git))

> **Note**: WrapGL is an OpenGL wrapper I made to streamline OpenGL projects
  like this one.

## Contributing

Contributions are welcomed. Contact me via email (felipemd.dutra@gmail.com)
for any information, help or questions.

## Demos

I'll update the README with demos and code examples soon.

