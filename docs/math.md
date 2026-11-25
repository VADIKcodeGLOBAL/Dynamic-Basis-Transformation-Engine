# DBTE math reference

Status: dbte-core 0.2.0, GPU data layout version 1.

This is the specification that dbte-core (C++) and the shaders both implement. The
formulas must match exactly, so change both sides in the same pull request, and bump the
layout version in `core/include/dbte/gpu_layout.hpp` when the packed data changes.
[ARCHITECTURE.md](ARCHITECTURE.md) explains why the model looks like this; this document
says exactly what is computed.

## Notation

- p is a point in logical space L, q = Φ(p) is its image in visible space W.
- J(p) = ∂Φ/∂p is the Jacobian. Its columns are the images of the coordinate axes, which
  are the local basis vectors i, j, k at p.
- Matrices are column-major: M v = v.x i + v.y j + v.z k. I is the identity.
- All values are 32-bit floats.

## Shared helpers

| Helper | Definition |
| --- | --- |
| `clamp(v, lo, hi)` | `min(max(v, lo), hi)` |
| `smoothstep01(t)` | c² (3 − 2c) with c = clamp(t, 0, 1) |
| `smoothstep01'(t)` | 6c (1 − c); zero outside (0, 1), so the step is continuously differentiable |
| `sinc(x)` | sin(x) / x; 1 − x²/6 for \|x\| < 10⁻⁴ |
| `tanh(x)`, `sech²(x)` | with e = exp(−2\|x\|): sign(x) (1 − e)/(1 + e) and 4e/(1 + e)², which stay accurate for large \|x\| |
| `cof(M)` | cofactor matrix with columns j × k, k × i, i × j; equals det(M) M⁻ᵀ |

## Frames

Bend, twist, taper and corridor compression are defined in a rigid local frame with
rotation R (its columns are the local axes in L) and origin o:

```math
l = R^{\mathsf T}(p - o), \qquad \Phi(p) = o + R\,\varphi(l), \qquad J(p) = R\,J_l(l)\,R^{\mathsf T}
```

The inverse goes the same way: q_l = Rᵀ(q − o), then p = o + R φ⁻¹(q_l).

## The chain

Deformers apply in index order, and only those whose layer mask shares a bit with the
geometry's mask:

```math
\Phi = \Phi_n \circ \dots \circ \Phi_1, \qquad J(p) = J_n(p_{n-1}) \cdots J_1(p_0)
```

- Tangents: t_W = J t_L.
- Normals: n_W = normalize(cof(J) n_L). If cof(J) n_L has zero length, keep n_L.
- Inverse: the links in reverse order, each with its own inverse below.
- Camera frame: the rotation R of the polar decomposition J = R S.

## Deformers

Every deformer has a weight w. For all of them w = 0 is the identity.

### Global basis (type 1)

Parameters: origin o, basis B, weight w.

```math
B_w = (1 - w)\,I + w\,B, \qquad \Phi(p) = o + B_w\,(p - o), \qquad J = B_w, \qquad \Phi^{-1}(q) = o + B_w^{-1}(q - o)
```

It folds when det B_w ≤ 0. The weight blends the matrices linearly, which can pass through
det = 0 for bases with large rotations. To animate between two bases, interpolate them on
the CPU with `interpolate_bases` and keep w = 1.

### Affine volume (type 2 sphere, type 3 box)

Parameters: centre c, basis B, inner distance r_in, outer distance r_out, weight w₀. A box
also has half extents e and an orientation quaternion q_b.

Distance d(p) and its gradient:

- Sphere: d = |p − c|, ∇d = (p − c)/d.
- Box: l = q_b⁻¹ (p − c) (rotated by the conjugate quaternion), m = max(|l| − e, 0) per
  component, d = |m|, ∇d = q_b (m ⊙ sign(l)) / d. So d is 0 inside the box.
- ∇d = 0 where d = 0.

```math
t = \frac{d - r_{in}}{r_{out} - r_{in}}, \qquad w = w_0\,\big(1 - \operatorname{smoothstep01}(t)\big), \qquad \nabla w = -\frac{w_0\,\operatorname{smoothstep01}'(t)}{r_{out} - r_{in}}\,\nabla d
```

```math
M = B - I, \qquad u = M\,(p - c), \qquad \Phi(p) = p + w\,u, \qquad J = I + w\,M + u\,\nabla w^{\mathsf T}
```

J is continuous everywhere. Its derivative jumps at d = r_in, at d = r_out and, for a box,
on the planes |l_k| = e_k.

Inverse:

1. Inside: p = c + (I + w₀ M)⁻¹ (q − c), accepted if d(p) ≤ r_in.
2. Outside: p = q, accepted if d(q) ≥ r_out.
3. In the soft edge: Newton's method from p₀ = c + (I + w(q) M)⁻¹ (q − c). Each step
   p ← p − s J(p)⁻¹ (Φ(p) − q) starts with s = 1 and halves s until the residual drops. At
   most 8 iterations; converged when |Φ(p) − q| ≤ 10⁻⁵ max(1, |q|).

Fold-over. With A = I + w M the matrix determinant lemma gives
det J = det(A) (1 + ∇wᵀ A⁻¹ u). The gradient term grows as the edge gets thinner. Example:
the shear j = (0.6, 1, 0) gives det J = 1 − 0.3 d s′(d) on the worst diagonal of the xy
plane, where s′ is the derivative of the smoothstep with respect to d. With
r_in = 4, r_out = 6 the minimum is −0.136 and space folds; with r_in = 1, r_out = 6 it is
0.651. The stronger the deformation, the wider the edge must be.

### Bend (type 4)

After Barr (1984). Local frame: the bent segment runs along local y from y = 0 to y = L
and curves towards +z (for k > 0) around an axis parallel to x, with radius 1/|k|. Before
the segment space is unchanged; beyond it space follows rigidly.

Parameters: curvature k₀, length L, weight w. The effective curvature is k = w k₀, so the
weight unbends the segment smoothly.

```math
\hat y = \operatorname{clamp}(y, 0, L), \qquad \theta = k\,\hat y
```

```math
\varphi(x, y, z) = \begin{pmatrix} x \\ \hat y\,\operatorname{sinc}\theta - z\sin\theta + (y - \hat y)\cos\theta \\ \hat y\,\sin\tfrac{\theta}{2}\operatorname{sinc}\tfrac{\theta}{2} + z\cos\theta + (y - \hat y)\sin\theta \end{pmatrix}
```

ŷ sinc θ equals sin θ / k and ŷ sin(θ/2) sinc(θ/2) equals (1 − cos θ)/k; this form is
exact for k → 0.

```math
J_l = \begin{bmatrix} (1, 0, 0)^{\mathsf T} & s\,(0, \cos\theta, \sin\theta)^{\mathsf T} & (0, -\sin\theta, \cos\theta)^{\mathsf T} \end{bmatrix}, \qquad s = \begin{cases} 1 - k z & 0 < y < L \\ 1 & \text{otherwise} \end{cases}
```

det J = s, so space folds where k z ≥ 1, beyond the centre of the bend. Keep the geometry
inside the radius: the validator checks half_extents.z. At y = 0 and y = L, J jumps for
z ≠ 0 (the stretch along the axis changes from 1 − kz to 1). Normals of faces aligned with
the local axes stay continuous there.

Inverse (k = 0 or L = 0 is the identity):

1. Bent segment: θ = atan2(kY, 1 − kZ), y = θ/k. If 0 ≤ y ≤ L:
   s = hypot(kY, 1 − kZ), z = (2Z − k(Y² + Z²)) / (1 + s), x = X.
2. Before the segment (Y < 0): p = q.
3. Beyond: Θ = kL, E = (L sinc Θ, L sin(Θ/2) sinc(Θ/2)) is the end of the arc in (y, z),
   (r_y, r_z) = (Y, Z) − E, y = L + r_y cos Θ + r_z sin Θ, z = −r_y sin Θ + r_z cos Θ.

The inverse is unique for |kL| ≤ π/2 and k z < 1. The validator rejects |kL| > π.

### Twist (type 5)

After Barr (1984): cross-sections rotate about local y.

Parameters: length L, angle Θ₀, weight w; Θ = w Θ₀.

```math
\theta = \Theta\,\operatorname{smoothstep01}(y / L), \qquad \theta' = \frac{\Theta\,\operatorname{smoothstep01}'(y / L)}{L}
```

```math
X = x\cos\theta + z\sin\theta, \qquad Y = y, \qquad Z = -x\sin\theta + z\cos\theta
```

```math
J_l = \begin{bmatrix} (\cos\theta, 0, -\sin\theta)^{\mathsf T} & (\theta' Z, 1, -\theta' X)^{\mathsf T} & (\sin\theta, 0, \cos\theta)^{\mathsf T} \end{bmatrix}, \qquad \det J = 1
```

Inverse: θ from Y, then x = X cos θ − Z sin θ, z = X sin θ + Z cos θ.

### Taper (type 6)

After Barr (1984): cross-sections scale in local x and z.

Parameters: length L, end scales s_x and s_z, weight w.

```math
a_x = w\,(s_x - 1), \quad a_z = w\,(s_z - 1), \quad h = \operatorname{smoothstep01}(y / L), \quad h' = \frac{\operatorname{smoothstep01}'(y / L)}{L}
```

```math
f_x = 1 + a_x h, \qquad f_z = 1 + a_z h, \qquad \varphi = (f_x\,x,\; y,\; f_z\,z)
```

```math
J_l = \begin{bmatrix} (f_x, 0, 0)^{\mathsf T} & (a_x h' x, 1, a_z h' z)^{\mathsf T} & (0, 0, f_z)^{\mathsf T} \end{bmatrix}, \qquad \det J = f_x f_z
```

Inverse: (X / f_x(Y), Y, Z / f_z(Y)). Space folds where a scale factor reaches 0.

### Corridor compression (type 7)

Distance along local y is compressed relative to the player. Near the player space is
almost unchanged; far points never appear further than the maximum distance; the entrance
(the local origin) stays in place, so the corridor joins the room before it without a gap.

Parameters: maximum distance a₀, the player's local coordinate y_p (updated every frame),
weight w. Off when w ≤ 0 or a₀ ≤ 0; otherwise a = a₀ / w.

```math
Y = a\left(\tanh\frac{y - y_p}{a} + \tanh\frac{y_p}{a}\right), \qquad X = x, \qquad Z = z, \qquad J_l = \operatorname{diag}\!\left(1,\ \operatorname{sech}^2\frac{y - y_p}{a},\ 1\right)
```

So Y(0) = 0, dY/dy = 1 at the player, and points ahead approach Y(y_p) + a without
reaching it. det J > 0 everywhere: the compression never folds.

Inverse: v = Y/a − tanh(y_p/a). If |v| < 1, y = y_p + a atanh(v). Otherwise the point has
no preimage; the core clamps v and reports no convergence.

## Validator

`validate()` checks one deformer, in closed form where possible:

| Deformer | Smallest det J | Other checks |
| --- | --- | --- |
| Global basis | det B_w | |
| Affine volume | det(I + w₀ M) inside; the soft edge sampled on a 24³ grid over the bounding box of the outer edge, refined by a coordinate search | r_in ≥ 0, r_out > r_in, box extents ≥ 0, unit quaternion |
| Bend | 1 − \|k\| half_extents.z | \|kL\| ≤ π, L > 0, rigid frame |
| Twist | 1 | L > 0, rigid frame |
| Taper | f_x f_z sampled along the segment | L > 0, rigid frame |
| Corridor compression | sech² over half_extents.y, for information only | a₀ > 0, rigid frame |

A fold is reported when the smallest det J is at most ε (10⁻³ by default).
`validate_region()` samples det J of a whole chain over a box.

## GPU data layout

See [ARCHITECTURE.md, section 6](ARCHITECTURE.md#6-cpugpu-data-contract-v1) and
`core/include/dbte/gpu_layout.hpp`.

## References

- A. H. Barr, "Global and Local Deformations of Solid Primitives", SIGGRAPH 1984.
- K. Shoemake, T. Duff, "Matrix Animation and Polar Decomposition", Graphics Interface 1992.
- N. J. Higham, "Computing the Polar Decomposition, with Applications", SIAM Journal on
  Scientific and Statistical Computing 7(4), 1986.
