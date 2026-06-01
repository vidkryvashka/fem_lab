# 3D Finite Element Method (FEM) Solver & Visualizer

![large ./docs/example.gif](./docs/example.gif)

5th year of Ivan Franko LNU

A software suite for calculating the stress-strain state of 3D bodies using the Finite Element Method (FEM). The project utilizes the **Raylib** graphics library for real-time rendering of the mesh, deformations, and stress heatmaps.

---

## Mathematical Model and Algorithms

### 1. Discretization and Isoparametric Approximation

The domain is divided into $nel$ finite elements. To approximate displacements within an element, isoparametric shape functions $\phi_i(\alpha, \beta, \gamma)$ are used in the local coordinate system $\alpha, \beta, \gamma \in [-1, 1]$.

The displacement $u(x, y, z)$ at any point within the element is defined as:


$$u(x, y, z) = \sum_{i=1}^{20} \phi_i(\alpha, \beta, \gamma) u_i$$


where $u_i$ are the displacements at the nodes of the finite element.

Since the element is isoparametric, the global coordinates are also expressed through the shape functions:


$$x = \sum_{i=1}^{20} \phi_i(\alpha, \beta, \gamma) x_i, \quad y = \sum_{i=1}^{20} \phi_i(\alpha, \beta, \gamma) y_i, \quad z = \sum_{i=1}^{20} \phi_i(\alpha, \beta, \gamma) z_i$$

### 2. Element Stiffness Matrix (MGE)

The local stiffness matrix of size 60x60 is calculated using the Jacobian matrix $[DJ]$ for the transition from global to local coordinates:


$$[DJ] = \begin{bmatrix} \frac{\partial x}{\partial \alpha} & \frac{\partial y}{\partial \alpha} & \frac{\partial z}{\partial \alpha} \\ \frac{\partial x}{\partial \beta} & \frac{\partial y}{\partial \beta} & \frac{\partial z}{\partial \beta} \\ \frac{\partial x}{\partial \gamma} & \frac{\partial y}{\partial \gamma} & \frac{\partial z}{\partial \gamma} \end{bmatrix}$$

To calculate the stiffness matrix $K^e$, 3D Gaussian integration is applied (3 points per axis, 27 points in total):


$$K^e = \sum_{m=1}^3 \sum_{n=1}^3 \sum_{k=1}^3 w_m w_n w_k B^T D B \det(J_{mnk})$$


where $w$ are the Gaussian weight coefficients, $D$ is the elasticity matrix (depending on Young's modulus $E$ and Poisson's ratio $\nu$), and $B$ is the matrix of shape function derivatives with respect to global coordinates.

The Lamé parameters are calculated as:


$$\lambda = \frac{E \cdot \nu}{(1+\nu)(1-2\nu)}, \quad \mu = \frac{E}{2(1+\nu)}$$

### 3. Load Vector (FE)

The application of pressure $P$ to an element face is modeled by a surface integral. 2D shape functions $\psi_i(\eta, \tau)$ are used for the 8 nodes of the face:


$$F^e = \iint_S N^T P \, dS$$

The calculation is performed using 2D Gaussian quadratures (3x3 = 9 points):


$$F^e = \sum_{m=1}^3 \sum_{n=1}^3 w_m w_n P (J_{2D}) \psi_i(\eta, \tau)$$


where the normal and area are accounted for via the cross product of the tangent vectors on the surface $\left(\frac{\partial \mathbf{r}}{\partial \tau} \times \frac{\partial \mathbf{r}}{\partial \eta}\right)$.

### 4. Boundary Conditions (ZU)

To account for rigid fixation (displacements $u_x=0, u_y=0, u_z=0$), the **penalty method** is used. A large number (penalty) is added to the main diagonal of the global stiffness matrix $[MG]$ at the corresponding nodes, and 0 is written to the load vector:


$$MG[i][i] = 10^{16}, \quad F[i] = 0$$

### 5. Solving the Global SLAE

The assembled global stiffness matrix $[MG]$ and load vector $F$ form the system:


$$[MG] \{U\} = \{F\}$$


Since the matrix $[MG]$ is symmetric and positive-definite, the iterative **Conjugate Gradient Method** is used to find the displacement vector $\{U\}$, providing fast convergence for sparse systems.

### 6. Stress Calculation (Von Mises)

After finding the displacements, the components of the strain tensor are calculated:


$$\epsilon_{xx} = \frac{\partial u_x}{\partial x}, \quad \epsilon_{yy} = \frac{\partial u_y}{\partial y}, \quad \gamma_{xy} = \frac{\partial u_x}{\partial y} + \frac{\partial u_y}{\partial x}$$

The stress tensor according to Hooke's Law:


$$\sigma_{xx} = \lambda(\epsilon_{xx} + \epsilon_{yy} + \epsilon_{zz}) + 2\mu\epsilon_{xx}$$

$$\tau_{xy} = \mu \gamma_{xy}$$

The equivalent stress according to the Von Mises yield criterion is calculated to visualize the heatmap:


$$\sigma_{v} = \sqrt{\frac{1}{2} \left[ (\sigma_{xx}-\sigma_{yy})^2 + (\sigma_{yy}-\sigma_{zz})^2 + (\sigma_{zz}-\sigma_{xx})^2 + 6(\tau_{xy}^2 + \tau_{xz}^2 + \tau_{yz}^2) \right]}$$

---

## Build & Run

To compile the project, the [Raylib](https://www.raylib.com/) library and a standard C compiler (gcc/clang) are required.

### macOS / Linux (not tested)

```bash
# Compilation and run
make clean run

```

## Controls

* **Right-Click on a face:** Add/Remove a boundary condition.
* With "SET FIXATION" active: The face turns blue (ZU - fixed).
* With "SET PRESSURE" active: The face turns orange (ZP - under pressure).


* **Left-Click (outside UI):** Rotate camera (Orbit).
* **Mouse Wheel:** Zoom (In/Out).
* **"RUN FEM ANALYSIS" Button:** Triggers the matrix assembly and SLAE solving.
* **UI Panel (Left):** Adjust the number of mesh blocks, Young's modulus, Poisson's ratio, and pressure magnitude.
