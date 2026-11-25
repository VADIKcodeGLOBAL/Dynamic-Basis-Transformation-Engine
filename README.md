Join the Project!
This project is currently in its conceptual stage, and looking for visionary developers, mathematicians, and technical artists to bring it to life. If you are passionate about linear algebra, non-Euclidean geometry, or low-level engine architecture, your contribution would be invaluable. Let's redefine how space works in games!

# Dynamic-Basis-Transformation-Engine
A conceptual game engine focused on real-time non-orthonormal basis transformations and non-Euclidean spatial logic.

## Overview
**Dynamic Basis Transformation Engine (DBTE)** is a conceptual game engine architecture where the fundamental laws of Euclidean space are treated as dynamic variables. Unlike traditional engines that use a fixed orthonormal basis (where $x, y, z$ are always perpendicular and unit-length), **DBTE** exposes the world's basis vectors as primary gameplay and rendering parameters.

By manipulating the **basis vectors** directly in the global or local coordinate systems, developers can create organic non-Euclidean effects, spatial distortions, and unique gameplay mechanics without modifying individual mesh data.

---

## Core Concepts

### 1. The Fluid Basis
In a standard engine, the basis is fixed:
* $\vec{i} = (1, 0, 0)$
* $\vec{j} = (0, 1, 0)$
* $\vec{k} = (0, 0, 1)$

**DBTE** allows these vectors to be non-orthogonal, non-unit, and time-dependent. This enables:
* **Shear Transformations:** Tilting the entire world or specific zones.
* **Variable Scaling:** Compressing or expanding the "fabric" of space.
* **Basis Interpolation:** Smoothly transitioning between different coordinate systems to create "mind-bending" visual puzzles.

### 2. Shader-Centric Spatial Mapping
Instead of relying on standard CPU-side Transform components, DBTE utilizes a custom **Basis-Aware Vertex Shader**. Every vertex position $P$ is calculated as a linear combination of the dynamic basis:

$$P_{transformed} = x\vec{i}_{dyn} + y\vec{j}_{dyn} + z\vec{k}_{dyn}$$ 

This offloads the spatial deformation to the GPU, allowing for real-time manipulation of complex environments.

### 3. Non-Euclidean Topologies
By mapping basis transformations to the player's position, the engine can simulate:
* **Infinite Corridors:** Where the forward basis vector shrinks as you move.
* **Escher-like Geometry:** Where "up" ($\vec{j}$) gradually rotates into "forward" ($\vec{k}$).
* **Spatial Pockets:** Areas where the internal volume is mathematically larger than its external appearance.

---

## Technical Features (Proposed)
* **Custom Math Library:** Specialized for non-orthonormal matrix operations and inverse transformations for physics.
* **Basis Volumes:** Trigger zones that interpolate the world's basis as the player enters them.
* **Linear Algebra Physics:** A collision detection system that operates within distorted coordinate spaces.
* **Procedural Visuals:** Built-in support for "Shear" and "Skew" effects that don't break shadow mapping.

---

## Project status

DBTE is being built as a **C++ library plus engine plugins**, starting with Godot 4, rather than as a new engine.
Stage 1 of 4 is in progress: architecture, repository skeleton, CI and a first Godot prototype.

| Stage | Result |
| --- | --- |
| 1. Architecture and skeleton | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), build, CI, [global-basis prototype](prototypes/global-basis) |
| 2. dbte-core | Deformers, the mapping and its inverse, validation, GPU packing, unit tests |
| 3. Godot plugin | `addons/dbte`: nodes, shaders and editor tools |
| 4. Demo | A hub and three rooms: shear, Escher corridor, infinite corridor |

## Repository layout

| Path | Contents |
| --- | --- |
| `core/` | dbte-core: engine-agnostic C++20 library and its tests |
| `prototypes/global-basis/` | Stage 1 Godot prototype: one global basis, shadows included |
| `integrations/godot/` | Godot plugin (Stage 3) |
| `demo/godot/` | Demo project (Stage 4) |
| `docs/` | Architecture and math |

## Build dbte-core

Requires CMake 3.24+ and a C++20 compiler (Visual Studio 2022, GCC 13+ or Clang 18+).

```sh
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build --build-config Release --output-on-failure
```

## Try the prototype

Open `prototypes/global-basis/project.godot` in [Godot 4.7](https://godotengine.org/download/) and press F5.
The [prototype README](prototypes/global-basis/README.md) lists the controls and what to look for.

---

How to contribute: Check out the Issues tab, join our discussions, or submit a Pull Request. Every idea counts.
