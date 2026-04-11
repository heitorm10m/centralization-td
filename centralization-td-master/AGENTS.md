# Agent Instructions

## MANDATORY READING BEFORE ANY TASK

Before writing, modifying, or reviewing any code in this repository,
you MUST fully read the file:

    docs/PHYSICS_REFERENCE.md

This file contains the physics, equations, and assumptions of the 
reference paper (Dao et al. 2023). All code must conform to it.

If there is any conflict between existing code and PHYSICS_REFERENCE.md,
PHYSICS_REFERENCE.md is the ground truth. Flag the conflict before acting.

## Hard Rules

1. Never simplify physics without explicitly warning the user
2. Never replace a vector sum with a scalar sum or vice versa
3. Never use linear penalty contact where the paper uses smooth Heaviside
4. Always implement analytical Jacobians, never use numerical differentiation
5. Before implementing any equation, cite its number from the paper
6. When asked to implement something, show which part of 
   PHYSICS_REFERENCE.md you used
7. If PHYSICS_REFERENCE.md does not exist yet, stop and ask the 
   user to provide it before doing anything physics-related

## Reference Paper

Dao, N.H. et al. "Modeling of a detailed bow spring centralizer 
description in stiff-string torque and drag calculation."
Geoenergy Science and Engineering 222 (2023) 211457

## Note

docs/PHYSICS_REFERENCE.md does not exist yet. It will be added soon.
Until it exists, do not implement any physics equations.
Ask the user to provide the content before proceeding.
