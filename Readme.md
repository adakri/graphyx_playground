# graphyxPlayground

**(This project is still not fully pushed, still a few things to sort out)**.

This project is my very simplistic way of documenting a simple tutorial for graphics programming covering the OpenGL graphics pipeline as well as the compute capabilities using OpenGL as well as CUDA. 

## Story Time
When starting a long time ago (a couple of years ago) to explore OpenGL and graphics programming I was overwhelmed, I remember I had my first programming course with PyGL and it covered basic rendering stuff. The issue is that the course had a bunch of pre made classes that did a lot of stuff behind the scenes. At the end of that course I understood the basics of the theory behind computer graphics (as in projective geometry, an overview of the rudimentary graphics pipeline in OpenGL) but never felt comfortable with the framework. Even in the advanced graphics courses (dealing with advanced glsl, ray tracing etc..), I always worked in groups, used pre made software and only focused on the task at hand. And when I wanted to relearn correctly all these things (which I am no way claiming I master) when dealing with otherwise excellent tutorials like [learnopengl](https://learnopengl.com/) I either felt too underwhelmed at the start but very confused when dealing with the more advanced things. So when I was at my last year, I had a GPGPU project dealing with [Particle systems simulation](https://github.com/adakri/HPC_GPU_NBody_visualisation), I had the idea that is this project: Revisiting a very simple graphics/compute task (Boids simulation, for which I have [another project](https://github.com/adakri/Projet_rentree_cpp)) and try my best to document it to explore as simplistically as possible the graphics compute/rendering paradigm for interactive simulation.


# Acknowledgements
In addition to the two other personal projects I have mentionned, I have extensively used the internet (for the greater good I hope) and you will find a lot of stackoverflow links that docuent the code, otherwise I have to mention these [two]((https://github.com/sarckk/boids)) [projects](https://github.com/L0rentz/Boids-2D) from which I have delibratly taken and heavily adapted a bunch of stuff.  

# The ideas I experimented with and documented

(The specifics are TBD).

* Conventional rendering pipeline for multi body simulation (and triangle rasterization ingeneral in OpenGL).
* UI integration with DearImGUI (*TBD*).
* GPU compute using compute shaders.
* Geometry shaders.
* GPU compute with CUDA and OpenGL interop (TBD).


# SetUp

This project was tested on Ubuntu 22.04 with opengl 4.3 minimum version, on a GTX1060 Nvidia GPU.
Since this projects attempts to set up a full framework for interactive simulation, a bunch of modules and librairies need to be added

TODO

# Results

<img src="./imgs/sim.gif" width="640" height="480"/>
