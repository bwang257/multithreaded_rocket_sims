### Quaternions

```
Describes what Quaternions are with example,
along with implementation notes
```

Rotations of complex numbers $a+bi$ in a 2D number system occur by applying Euler's formula: multiplying by `cos θ + i sin θ` spins a point by θ. Quaternions are the 3D analogue.  A quaternion is
$$
q = w + xi + yj + zk    \quad  w, x, y, z ∈ ℝ
$$

often written as a scalar/vector pair $q = (w, \mathbf{v})\text{ with }\mathbf{v} = (x, y, z)$. Quaternions allow encoding of 3D rotations compactly (don't need rotation matrix), compose by multiplication, interpolate smoothly (Slerp), never suffer gimbal lock, and are cheap to normalize. 

**Properties**
$i^2 = j^2 = k^2 = ijk = −1$ 

where:
$$ij = k, \quad jk = i, \quad ki = j$$
$$ji = -k, \quad kj = -i, \quad ik = -j$$
Quaternion multiplication is associative but not commutative, which matches the properties of rotations we are trying to model (the $\mathbf{v}_1 \times \mathbf{v}_2$ term is the non-commutative part).

Expanding the rules for $q_1 = (w_1, \mathbf{v}_1)$ and $q_2 = (w_2, \mathbf{v}_2)$:
$$q_1 q_2 = \bigl(, w_1 w_2 - \mathbf{v}_1 \cdot \mathbf{v}_2, \space \space w_1 \mathbf{v}_2 + w_2 \mathbf{v}_1 + \mathbf{v}_1 \times \mathbf{v}_2 ,\bigr)$$

**Conjugate, Norm, Inverse**
$$\bar{q} = (w, -\mathbf{v}), \qquad \lVert q \rVert^2 = w^2 + x^2 + y^2 + z^2 = q \bar{q}, \qquad q^{-1} = \frac{\bar{q}}{\lVert q \rVert^2}$$

For a **unit quaternion** ($\lVert q \rVert = 1$), $q^{-1} = \bar{q}$. This is the case we always use for rotation, and it makes inversion free.

Useful identity: $\overline{pq} = \bar{q}\,\bar{p}$ — the order reverses, like matrix transpose/inverse.


**Rotating a 3D Vector**
A rotation by angle $\theta$ about a unit axis $\hat{\mathbf{u}}$ is

$$q = \left( \cos\frac{\theta}{2}, \; \sin\frac{\theta}{2}\,\hat{\mathbf{u}} \right)$$

Embed the 3D vector $\mathbf{v}$ as a *pure* quaternion $p = (0, \mathbf{v})$, then:
$$p' = q\,p\,q^{-1} = q\,p\,\bar{q} \quad (q \text{ unit})$$

The result $p'$ is again pure, and its vector part is the rotated vector $\mathbf{v}'$.

(Composition) To rotate by $q_1$ then by $q_2$:
$$q_{\text{total}} = q_2 q_1 \qquad \text{(applied right-to-left, like matrices)}$$
Noting that- $q_2 (q_1 p \bar{q}_1) \bar{q}_2 = (q_2 q_1)\, p \,\overline{(q_2 q_1)}$. 

#### Example
Rotate $\mathbf{v} = (1,0,0)$ by $90^\circ$ about the $z$-axis. Expect $(0,1,0)$.

Axis $\hat{\mathbf{u}} = (0,0,1)$, $\theta = 90^\circ$, so $\theta/2 = 45^\circ$:
$$q = \left( \tfrac{\sqrt{2}}{2}, \; \left(0, 0, \tfrac{\sqrt{2}}{2}\right) \right), \qquad p = \bigl( 0, \; (1,0,0) \bigr)$$

**$qp$:**

$$\text{scalar} = \tfrac{\sqrt{2}}{2}\cdot 0 - \left(0,0,\tfrac{\sqrt{2}}{2}\right)\cdot(1,0,0) = 0$$

$$\text{vector} = \tfrac{\sqrt{2}}{2}(1,0,0) + 0\cdot\left(0,0,\tfrac{\sqrt{2}}{2}\right) + \left(0,0,\tfrac{\sqrt{2}}{2}\right)\times(1,0,0) = \left(\tfrac{\sqrt{2}}{2}, 0, 0\right) + \left(0, \tfrac{\sqrt{2}}{2}, 0\right) = \left(\tfrac{\sqrt{2}}{2}, \tfrac{\sqrt{2}}{2}, 0\right)$$

**multiply by $\bar{q} = \left(\tfrac{\sqrt{2}}{2}, \left(0,0,-\tfrac{\sqrt{2}}{2}\right)\right)$:**
$$\text{scalar} = 0\cdot\tfrac{\sqrt{2}}{2} - \left(\tfrac{\sqrt{2}}{2},\tfrac{\sqrt{2}}{2},0\right)\cdot\left(0,0,-\tfrac{\sqrt{2}}{2}\right) = 0 \quad \text{  - still pure}$$
$$\text{vector} = \tfrac{\sqrt{2}}{2}\left(\tfrac{\sqrt{2}}{2},\tfrac{\sqrt{2}}{2},0\right) + \left(\tfrac{\sqrt{2}}{2},\tfrac{\sqrt{2}}{2},0\right)\times\left(0,0,-\tfrac{\sqrt{2}}{2}\right) = \left(\tfrac{1}{2}, \tfrac{1}{2}, 0\right) + \left(-\tfrac{1}{2}, \tfrac{1}{2}, 0\right) = (0,1,0) \quad$$

### Coding Implementation:
Doing two full quaternion products is wasteful. Expanding the sandwich for unit $q = (w, \mathbf{u})$ gives:
$$\mathbf{t} = 2\,(\mathbf{u} \times \mathbf{v}), \qquad \mathbf{v}' = \mathbf{v} + w\,\mathbf{t} + \mathbf{u} \times \mathbf{t}$$

Two cross products, roughly 15 flops. You can also convert it to a matrix:

If you're rotating many vectors by the same $q$, build the $3\times3$ once (with $w,x,y,z$ unit):

$$R = \begin{bmatrix} 1 - 2(y^2+z^2) & 2(xy - wz) & 2(xz + wy) \\ 2(xy + wz) & 1 - 2(x^2+z^2) & 2(yz - wx) \\ 2(xz - wy) & 2(yz + wx) & 1 - 2(x^2+y^2) \end{bmatrix}$$

Rule of thumb: fewer than about 3 vectors → sandwich formula; more → build $R$.

**Additional Notes:**
- **Double cover.** $q$ and $-q$ describe the *same* rotation ($\theta/2 \to \theta/2 + \pi$ flips both the $\sin$ and $\cos$ terms, and the sandwich cancels the sign). So the map from quaternions to rotations is 2-to-1. Consequence for interpolation: always check whether $q_1 \cdot q_2 < 0$ and negate one, or you'll take the $350^\circ$ path instead of the $10^\circ$ one.
- **Normalize periodically.** Repeated multiplication accumulates floating-point error and drifts off the unit sphere, which introduces scaling. Cheap fix: divide by $\lVert q \rVert$ every so often — far cheaper than re-orthonormalizing a matrix.



**Resources**
- [Quaternions](https://www.3dgep.com/understanding-quaternions/)
- [C++ Implementation Quaternions](https://github.com/JeanPhilippeKernel/RendererEngine/blob/develop/ZEngine/ZEngine/Core/Maths/Quaternion.h)
