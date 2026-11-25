# Prototype: one global basis

Stage 1 prototype. It checks the riskiest assumptions before the core library is written:

- a basis set from a script reaches every material through Godot **global shader uniforms**;
- the vertex shader deforms geometry **and its shadows** (Godot runs the same vertex
  shader in the shadow pass);
- a point computed on the CPU with the same formula lands exactly on the GPU-deformed
  geometry (the red marker).

Everything is in logical space; the shader maps it to visible space:

```math
p_W = o + B\,(p_L - o), \qquad B = \begin{bmatrix} \vec{i} & \vec{j} & \vec{k} \end{bmatrix}
```

## Run it

1. Install [Godot 4.7](https://godotengine.org/download/) (the standard build, not .NET).
   It is a single executable, no installer needed.
2. In the Project Manager choose **Import**, select `prototypes/global-basis/project.godot`
   and open it.
3. Press **F5** (Run Project).

The project uses the Forward+ renderer (Vulkan on Windows and Linux).

## Controls

| Key | Action |
| --- | --- |
| 1 to 4 | Presets: identity, shear, stretch and squash, skew on all axes |
| Space | Wave between the identity and the current preset |
| Tab | Deformation on/off |
| M | Show/hide the CPU marker |
| Mouse drag | Orbit the camera |
| Wheel | Zoom |
| Esc | Quit |

## What to check

- With preset 2 (shear) the pillars lean along X and **their shadows lean with them**.
- The red marker stays on the top corner of the pillar in the +X +Z corner of the grid in every preset.
- The grid on the surfaces is drawn in logical coordinates, so it skews together with space.
- Nothing pops in or out at the screen edges while the basis changes.
