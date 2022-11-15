# How to Make a 2D Soft-Body Platformer with Position Based Dynamics
This project was my senior thesis at Yale. I built it in Spring 2022 towards completion of my B.S. in Computer Science. This repository contains my project code, which is a supplement to my [thesis](/Senior%20Thesis.pdf).

For this project, I wanted to make a video game that included soft-body dynamics – squishy objects. So, I built a video game engine and a demo game level on top of a state-of-the-art physics engine, Jan Bender's [PositionBasedDynamics](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics). This is a fantastic physics library which uses the Position Based Dynamics technique to a simulate a large range of phenomena, including rigid bodies and soft bodies. The advantage of this method is that provides accuracy _and_ interactivity, which is ideal for physics-based puzzle games, for example. Like many complex research techniques, the library is difficult to understand: so this project provides the first guide to using it, in order to make PBD accessible to all hobbyist C++ developers.

Here is the demo game level I made using my engine. You can see the cube character jump around the level, interacting with all the squishy objects in order to reach its goal, the top platform, in the smallest number of jumps.

[<img width="644" alt="Demo Game Level" src="https://user-images.githubusercontent.com/17149360/191396096-8a910d95-e692-4dac-8ce5-f18ffc291659.png">](https://www.youtube.com/watch?v=Djj6FgJyC9E&ab_channel=ZacS)

## Abstract

The hobbyist game developer community has often pushed the limits of platform games. One popular feature in these games is soft-body dynamics, since it adds interest and greatly extends the range of possible motion. The PositionBasedDynamics library provides a fast and accurate implementation of soft bodies, along with several other physically-based features, but there are currently few guides to help amateur developers learn to use it. In this paper, we describe a method for building a game engine by making small, modular extensions to the PositionBasedDynamics library. 

Beyond demonstrating how to build the engine, the aim of the paper is to show that this existing soft-body implementation can be used to make fun platformers. The result is a simple, enjoyable physics puzzle game level which uses deformable objects to present interesting challenges to the player. The hobbyist game developer community has always used the tools available to them to make an incredible range of wonderfully innovative games --- with this paper, we aim to make one more tool accessible to them.

## Citations

This project is primarily an extension of Jan Bender's [PositionBasedDynamics](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics). For a list of citations, please see my main [thesis](/Senior%20Thesis.pdf).

## Acknowledgements

Special thanks to Professor K. for advising me and giving me invaluable thanks throughout. Many thanks also to [Professor Jan Bender](https://animation.rwth-aachen.de/person/1/), who built the amazing [PositionBasedDynamics library](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics) – the core of this project – and kindly answered my questions during development.
