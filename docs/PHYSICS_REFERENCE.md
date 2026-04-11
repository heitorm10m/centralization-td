
# PHYSICS_REFERENCE.md

## Source and scope

This document is a technical extraction of the paper **“Modeling of a detailed bow spring centralizer description in stiff-string torque and drag calculation”** by Dao et al., *Geoenergy Science and Engineering* 222 (2023) 211457. It is written as an implementation reference for a coding agent. It is grounded only in the paper material and the equation images provided in this conversation. Where the available extracts do not expose a value or a full numbered equation, that gap is marked explicitly.

---

## SECTION 1 — COMPLETE VARIABLE GLOSSARY

### 1.1 Bow-spring model symbols

| Symbol | Full name | Units | Physical meaning | First appears in |
|---|---|---:|---|---|
| \(n\) | number of blades | dimensionless | total number of blades in one bow-spring centralizer | Eq. (1) |
| \(\vec F_i\) | blade contact force vector | force | reaction force of blade \(i\) against the borehole | Eq. (1) |
| \(\vec F_{\text{restoring}}\) | restoring force vector | force | reaction of the centralizer on the casing/pipe; vector sum of all blade contact forces | Eq. (1) |
| \(\phi\) | clearance | length | radial gap between centralizer internal diameter and pipe external diameter | Eq. (2) |
| \(D_{\text{int}}\) | centralizer internal diameter | length | inner diameter of centralizer ring assembly | Eq. (2) |
| \(D_{\text{pipe}}\) | pipe external diameter | length | OD of casing/pipe on which the centralizer is mounted | Eq. (2) |
| \(\vec U_r\) | pipe radial displacement vector | length | lateral displacement of pipe center | Eq. (3) |
| \(\vec U_{r,\text{cent}}\) | centralizer radial displacement vector | length | effective radial displacement transmitted to centralizer after clearance is consumed | Eq. (3) |
| \(\delta_i\) | blade deflection | length | compression/deflection of blade \(i\) | Eq. (4)–(5) |
| \(\vartheta_i\) | blade deformed radius | length | distance from blade extremity to centralizer center after deformation | Fig. 6 / text around Eq. (5) |
| \(D_{\max}\) | maximum centralizer diameter | length | free maximum diameter of bow-spring centralizer | Eq. (5) |
| \(D_{\min}\) | minimum centralizer diameter | length | fully compressed minimum diameter of centralizer | text after Eq. (5) |
| \(D_{\text{hole}}\) | hole diameter | length | borehole diameter in the current section | Eq. (5) |
| \(k_{\text{blade}}\) | blade rigidity | force / length\(^p\) | coefficient of blade force law | Eq. (4), Eq. (9) |
| \(p\) | stiffness power coefficient | dimensionless | nonlinearity exponent in blade force law | Eq. (4), Eq. (9) |
| \(\alpha_i\) | blade working angle | rad or deg | angle of blade \(i\) relative to centralizer radial displacement direction | Eq. (5) |
| \(F_{fa}\) | axial friction force | force | axial drag related to blade–borehole friction | Eq. (6) |
| \(\mu_{\text{hole}}\) | hole friction coefficient | dimensionless | friction coefficient for blade–hole contact; depends on open hole or cased hole | Eq. (6) |
| \(F_{fr}\) | tangential friction force | force | friction between pipe and centralizer rings during rotation | Eq. (7) |
| \(\mu_{\text{steel-steel}}\) | steel–steel friction coefficient | dimensionless | friction coefficient for pipe–ring contact | Eq. (7) |
| \(M_{fr}\) | friction torque | moment | torque applied to casing due to tangential friction at rings | Eq. (8) |
| \(F_{\text{restoring,test}}\) | measured restoring force | force | manufacturer test restoring force | Eq. (9) |
| \(F_{\text{restoring,model}}\) | model restoring force | force | restoring force predicted by model | Eq. (9) |
| \(\mu_{\text{test}}\) | estimated test friction coefficient | dimensionless | friction coefficient back-computed from running force test | Eq. (10) |
| \(F_{\text{running,test}}\) | measured running force | force | manufacturer test running force | Eq. (10) |

The paper states that each blade is modeled as identical and independent, the restoring force is the **vector sum** of all blade forces, the axial friction is tied to the **scalar/algebraic sum of blade-force magnitudes**, and the tangential friction is tied to the **norm of the restoring-force resultant**.

### 1.2 FE formulation and contact symbols

| Symbol | Full name | Units | Physical meaning | First appears in |
|---|---|---:|---|---|
| \(R_i\) | inner radius | length | element inner radius | Sec. 3 |
| \(R_o\) | outer radius | length | element outer radius | Sec. 3 |
| \(A\) | cross-sectional area | length\(^2\) | annular area of hollow cylindrical element | Sec. 3 |
| \(E\) | Young modulus | pressure | elastic modulus | Sec. 3 |
| \(\nu\) | Poisson ratio | dimensionless | Poisson’s ratio | Sec. 3 |
| \(\rho\) | string mass density | mass/length\(^3\) | density of string material | Sec. 3 |
| \(\rho_f\) | fluid density | mass/length\(^3\) | density of drilling fluid | Sec. 3 |
| \(\vec x_k\) | well trajectory point | length, 3D vector | point on wellbore neutral line | Sec. 3 |
| \(\vec t_k\) | tangent vector | dimensionless, 3D vector | local trajectory tangent | Sec. 3 |
| \(R_w\) | wellbore radius | length | borehole radius | Sec. 3 |
| \(V_a\) | axial speed | length/time | tripping speed / ROP | Sec. 3, Eq. (19) |
| \(\Omega\) | rotation speed | angle/time | pipe angular speed | Sec. 3, Eq. (19) |
| \(\vec g\) | gravity vector | length/time\(^2\), 3D vector | gravitational acceleration | Sec. 3 |
| \(\vec e_X,\vec e_Y,\vec e_Z\) | global basis vectors | dimensionless, 3D vectors | global frame; \(\vec e_Z\) aligned with gravity | Eq. (11) text |
| \(\vec e_u,\vec e_v,\vec e_w\) | local nodal basis vectors | dimensionless, 3D vectors | local node frame; \(\vec e_w=\vec t\) | Eq. (11) |
| \(t_Z\) | vertical component of tangent | dimensionless | \(t_Z=\vec t\cdot\vec e_Z\) | Eq. (11) |
| \(n_{\text{nodes}}\) | number of nodes | dimensionless | total FE nodes | Sec. 3.1 |
| \(\mathbf K\) | stiffness matrix | force/displacement | global structural stiffness | Eq. (12) |
| \(\vec X\) | generalized displacement vector | mixed | all nodal displacements and rotations | Eq. (12) |
| \(\vec F_o\) | constant force vector | mixed | gravity, buoyancy, curvature-induced forces, etc. | Eq. (12) |
| \(\vec F_c(\vec X)\) | contact force vector | mixed | state-dependent contact forces | Eq. (12) |
| \(\vec u_i\) | translational displacement vector | length, 3D vector | node displacement \((u_i,v_i,w_i)\) | Sec. 3.1 |
| \(\vec \omega_i\) | rotational vector | rad, 3D vector | node rotation \((\theta_{ix},\theta_{iy},\theta_{iz})\) | Sec. 3.1 |
| \(\vec f_i\) | force vector | force, 3D vector | nodal force \((f_{ix},f_{iy},f_{iz})\) | Sec. 3.1 |
| \(\vec m_i\) | moment vector | moment, 3D vector | nodal moment \((m_{ix},m_{iy},m_{iz})\) | Sec. 3.1 |
| \(L\) | element length | length | beam element length | Eq. (13) |
| \(\vec q\) | distributed gravity-buoyancy force | force/length, 3D vector | \((\rho-\rho_f)A\vec g\) | Sec. 3.2 |
| \(\vec f_{g,1},\vec f_{g,2}\) | equivalent nodal forces | force, 3D vectors | body-force nodal forces | Eq. (13) |
| \(\vec m_{g,1},\vec m_{g,2}\) | equivalent nodal moments | moment, 3D vectors | body-force nodal moments | Eq. (13) |
| \(r\) | radial displacement magnitude | length | \(r=\sqrt{u^2+v^2}\) | Eq. (15) text |
| \(\Delta r\) | body contact penetration | length | relative radial displacement \(r+R_o-R_w\) | Eq. (15) text |
| \(\vec e_r\) | radial unit vector | dimensionless, 2D/3D embedded | direction of \(\vec r\) | Eq. (15) text |
| \(\vec e_\theta\) | orthoradial unit vector | dimensionless, 3D vector | \(\vec e_z\times \vec e_r\) | Eq. (15) text |
| \(\vec e_z\) | local axial direction | dimensionless, 3D vector | axial direction used in contact decomposition | Eq. (15) text |
| \(\vec f_c\) | body contact force vector | force, 3D vector | decomposed into radial, tangential, axial parts | Eq. (15) |
| \(f_{cr}\) | body normal contact force | force | radial contact component | Eq. (16) |
| \(k_c\) | contact stiffness | force/length | effective wellbore/contact stiffness | Eq. (16), Eq. (24) |
| \(H(\cdot)\) | Heaviside function | dimensionless | discontinuous contact activation | Eq. (16) |
| \(h(\cdot)\) | smooth Heaviside surrogate | dimensionless | \(C^1\) smoothing of contact activation | Eq. (17) |
| \(\lambda\) | smoothing parameter | length | tolerance-to-penetration parameter such that \(h(\lambda)=1/2\) | Eq. (17) |
| \(f_{c\theta}\) | tangential body friction | force | orthoradial friction component | Eq. (18) |
| \(f_{cz}\) | axial body friction | force | axial friction component | Eq. (18) |
| \(\mu_\theta\) | tangential friction coefficient component | dimensionless | velocity-weighted tangential part of friction | Eq. (19) |
| \(\mu_z\) | axial friction coefficient component | dimensionless | velocity-weighted axial part of friction | Eq. (19) |
| \(V_g^\varepsilon\) | numerical guard velocity | length/time | avoids division by zero in friction split | Eq. (19) |
| \(V_g\) | slip speed magnitude | length/time | combined slip speed | Eq. (19) |
| \(V_{g\theta}\) | tangential slip speed | length/time | \(R\Omega\) | Eq. (19) |
| \(V_{gz}\) | axial slip speed | length/time | \(V_a\) | Eq. (19) |
| \(\mu(V_g)\) | total friction coefficient | dimensionless | velocity-dependent friction law | Eq. (20) |
| \(\mu_s,\mu_d\) | static and dynamic friction coefficients | dimensionless | endpoints of friction law | Eq. (20) |
| \(V_g^0\) | reference slip velocity | length/time | decay scale in exponential transition | Eq. (20) |
| \(V_g^{\lim}\) | arctan smoothing velocity | length/time | removes discontinuity at zero slip | Eq. (20) |
| \(R_e\) | moment arm radius | length | radius used in contact moment computation | Eq. (21), Eq. (29) |

These symbols, frames, and contact quantities are defined in the FE formulation and in the body-contact and bow-contact subsections. The model uses two-node beam elements, six DOFs per node, explicit local frames, and regularized contact/friction laws.

### 1.3 Bow-spring FE contact symbols

| Symbol | Full name | Units | Physical meaning | First appears in |
|---|---|---:|---|---|
| \(N_b\) | number of blades in FE bow-spring model | dimensionless | blades per centralizer in detailed FE contact model | Eq. (22) text |
| \(\alpha_{b0}\) | initial bow-spring angular orientation | rad or deg | global orientation of one reference blade w.r.t. \(\vec e_u\) | Eq. (22) text |
| \(\alpha_{b,i}\) | blade orientation angle | rad or deg | angle of blade \(i\) | Eq. (22) text |
| \(\vec e_{br,i}\) | blade radial direction | dimensionless, 2D vector | blade \(i\) radial direction in local plane | Eq. (22) text |
| \(\vec e_{b\theta,i}\) | blade orthoradial direction | dimensionless, 2D vector | orthogonal in-plane direction of blade \(i\) | Eq. (28) |
| \(\vec r_{b,i}\) | blade position vector | length, 2D vector | position of blade \(i\) extremity relative to well section center | Eq. (22) |
| \(r_{b,i}\) | blade radius magnitude | length | \(r_{b,i}=\|\vec r_{b,i}\|\) | Eq. (22) text |
| \(r_0\) | pipe–ring initial clearance | length | gap between string and centralizer rings | Eq. (22) |
| \(R_b\) | blade free radius | length | radial location of blade extremity in unloaded state | Eq. (22) |
| \(R_{b,\min}\) | minimum blade radius | length | fully compressed blade radius | Eq. (23) text |
| \(\Delta r_{b,i}\) | blade penetration | length | blade \(i\) penetration into borehole | Eq. (23) |
| \(\Delta r_{b,\max}\) | maximum blade penetration before consolidation | length | \(R_b-R_{b,\min}\) | Eq. (23) |
| \(\Delta r_i\) | post-consolidation penetration | length | \(\Delta r_{b,i}-\Delta r_{b,\max}\) | Eq. (23) |
| \(k_b\) | blade rigidity in FE contact law | force/length\(^p\) | same physical role as \(k_{\text{blade}}\) | Eq. (23) |
| \(f_{cbr,i}\) | blade radial contact force | force | radial contact force for blade \(i\) | Eq. (23), Eq. (24) |
| \(\vec f_{cbr,i}\) | blade radial force vector | force, 2D/3D embedded vector | radial vector force for blade \(i\) | text after Eq. (24) |
| \(f_{cbz,i}\) | blade axial friction component | force | axial friction from blade \(i\) | Eq. (25) |
| \(f_{cbz}\) | summed blade axial friction | force | axial friction summed over all blades | Eq. (25) |
| \(\vec f_{cbr}\) | resultant of blade radial forces | force, 2D vector | vector sum of all \(\vec f_{cbr,i}\) | Eq. (26) |
| \(\vec f_{cb\theta}\) | resultant tangential blade friction | force, 2D/3D vector | orthoradial resultant due to pipe–ring friction | Eq. (27)–(28) |
| \(\vec f_{cb,i}\) | generalized bow-contact force per blade | force, 3D vector | local FE force contribution of blade \(i\) | Eq. (29) |
| \(\vec m_{cb,i}\) | bow-contact moment per blade | moment, 3D vector | local FE moment contribution of blade \(i\) | Eq. (29) |
| \(c_{cbu,i}\) | x-like bow coefficient | dimensionless | \(\cos\alpha_{b,i}-\mu_\theta\sin\alpha_{b,i}\) | Eq. (29) |
| \(c_{cbv,i}\) | y-like bow coefficient | dimensionless | \(\sin\alpha_{b,i}+\mu_\theta\cos\alpha_{b,i}\) | Eq. (29) |
| \(f'_{cr}\) | derivative of body radial contact force | force/length | derivative used in Jacobian | Eq. (35) |
| \(h'(x)\) | derivative of smooth Heaviside | 1/length | derivative of \(h(x)\) | Eq. (35) |
| \(f'_{cbr,i}\) | derivative of bow radial contact force | force/length | derivative used in bow Jacobian | Eq. (36) |
| \(\mathcal R(\vec X)\) | nonlinear residual | mixed | static equilibrium residual for Newton–Raphson | Sec. 3.4 |

The bow-spring FE model computes blade forces individually, assembles axial friction by summation along \(\vec e_w\), assembles tangential friction from the resultant blade force, and then converts these contributions into local nodal force and moment vectors. The Eq. (29) transcription contains one OCR typo, corrected in Section 12.

---

## SECTION 2 — PHYSICAL ASSUMPTIONS

**ASSUMPTION 1:** all blades of a centralizer are identical.  
**CONSEQUENCE:** one constitutive law \((k_{\text{blade}},p)\) is reused for every blade.  
**RISK IF VIOLATED:** heterogeneous blades cannot be represented without extending the model.

**ASSUMPTION 2:** each blade acts independently from the others.  
**CONSEQUENCE:** blade–blade coupling is ignored; contact of one blade does not modify the constitutive state of another except through global geometry.  
**RISK IF VIOLATED:** any ring-coupled or shell-like redistribution between blades is missed.

**ASSUMPTION 3:** a blade may be either in contact or out of contact with the borehole depending on lateral displacement and orientation.  
**CONSEQUENCE:** contact activation is unilateral and state-dependent.  
**RISK IF VIOLATED:** restoring force onset and loss-of-contact behavior become wrong.

**ASSUMPTION 4:** the pipe may have clearance relative to the centralizer rings.  
**CONSEQUENCE:** radial pipe motion does not immediately activate the centralizer; \(\phi\) or \(r_0\) must be consumed first.  
**RISK IF VIOLATED:** restoring force begins too early and the model overpredicts stiffness at small displacement.

**ASSUMPTION 5:** the centralizer is axially fixed to the pipe.  
**CONSEQUENCE:** no relative axial sliding between pipe and centralizer along the pipe axis.  
**RISK IF VIOLATED:** axial drag partition is wrong.

**ASSUMPTION 6:** the pipe is free to rotate inside the centralizer rings, but the rings do not rotate relative to the well.  
**CONSEQUENCE:** tangential friction is pipe–ring steel–steel friction; axial friction is blade–hole friction.  
**RISK IF VIOLATED:** torque and friction surfaces are confused.

**ASSUMPTION 7:** blade force follows a power law \(F_i=k_{\text{blade}}\delta_i^p\).  
**CONSEQUENCE:** linear behavior only for \(p=1\); otherwise nonlinear stiffness.  
**RISK IF VIOLATED:** restoring-force calibration cannot match manufacturer data.

**ASSUMPTION 8:** when a blade reaches the minimum-diameter regime, it is treated as infinitely rigid relative to the remaining compression, and borehole/contact stiffness \(k_c\) governs additional penetration.  
**CONSEQUENCE:** the constitutive law is two-regime.  
**RISK IF VIOLATED:** fully compressed centralizers remain artificially soft.

**ASSUMPTION 9:** FE displacements and rotations are referred to the well neutral line and assumed small.  
**CONSEQUENCE:** the formulation is a small-displacement beam model around a curved reference trajectory.  
**RISK IF VIOLATED:** large-displacement geometric nonlinearity is outside model scope.

**ASSUMPTION 10:** beam elements are initially straight even though the well reference line is curved.  
**CONSEQUENCE:** an initial curvature-induced stress/loading must be introduced.  
**RISK IF VIOLATED:** the reference state inside the curved well is inconsistent.

**ASSUMPTION 11:** body contact with the borehole is unilateral and regularized with a smooth Heaviside-like function.  
**CONSEQUENCE:** contact force is exactly zero for negative penetration and smooth for positive penetration.  
**RISK IF VIOLATED:** Newton–Raphson robustness degrades near contact onset.

**ASSUMPTION 12:** the split of friction into tangential and axial components is controlled by the slip-velocity components.  
**CONSEQUENCE:** \(\mu_\theta\) and \(\mu_z\) are not independent inputs but projections of \(\mu(V_g)\).  
**RISK IF VIOLATED:** incorrect force direction under combined rotation and translation.

**ASSUMPTION 13:** for bow-springs in the horizontal section, gravity can cause the top blade to lose contact.  
**CONSEQUENCE:** not all blades remain active; the active contact set changes with trajectory.  
**RISK IF VIOLATED:** force distribution around the circumference is wrong.

---

## SECTION 3 — ALL EQUATIONS IN IMPLEMENTATION ORDER

### 3.1 Frames and base kinematics

**EQ [11]:**
\[
\vec e_w=\vec t,\qquad
\vec e_u=\frac{-\vec e_Z+t_Z\vec t}{\sqrt{1-t_Z^2}},\qquad
\vec e_v=\vec t\times\vec e_u,\qquad
t_Z=\vec t\cdot\vec e_Z
\]
**COMPUTES:** local nodal frame.  
**INPUTS REQUIRED:** \(\vec t,\vec e_Z\).  
**VALID WHEN:** \(t_Z^2\neq 1\).  
**REGIME:** geometric frame construction.  
**NOTES:** singular in vertical sections; paper states a combination of \(\vec e_X,\vec e_Y\) is used there to ensure frame continuity.

### 3.2 Constant body loads

**EQ [13]:**
\[
\vec q=(\rho-\rho_f)A\vec g
\]
\[
\vec f_{g,1}=\vec f_{g,2}=\frac{L}{2}\vec q,\qquad
\vec m_{g,1}=-\vec m_{g,2}=\frac{L^2}{12}\,\vec t_1\times\vec q
\]
**COMPUTES:** distributed gravity-buoyancy force and equivalent nodal loads.  
**INPUTS REQUIRED:** \(\rho,\rho_f,A,\vec g,L,\vec t_1\).  
**VALID WHEN:** beam element body load representation.  
**REGIME:** constant force assembly.

**EQ [14]:** element-to-node assembly of \(\vec F_g\).  
**COMPUTES:** global body-force vector from elemental contributions.  
**INPUTS REQUIRED:** elemental \(\vec F_{g,1},\vec F_{g,2}\).  
**VALID WHEN:** FE assembly.  
**REGIME:** standard FE assembly.  
**NOTES:** the paper states the interior-node force is \(\vec F^2_{g,k-1}+\vec F^1_{g,k}\), with special cases at the first and last nodes.

### 3.3 Theoretical bow-spring geometry and force

**EQ [2]:**
\[
\phi=\frac{D_{\text{int}}-D_{\text{pipe}}}{2}
\]
**COMPUTES:** clearance between pipe and centralizer ring.  
**INPUTS REQUIRED:** \(D_{\text{int}},D_{\text{pipe}}\).  
**VALID WHEN:** \(D_{\text{int}}\ge D_{\text{pipe}}\) as a geometric clearance definition.  
**REGIME:** local centralizer geometry.

**EQ [3]:**
\[
\vec U_{r,\text{cent}}=\vec U_r-\phi\frac{\vec U_r}{\|\vec U_r\|}
\qquad\text{when }\|\vec U_r\|>\phi
\]
**COMPUTES:** effective radial displacement seen by the centralizer.  
**INPUTS REQUIRED:** \(\vec U_r,\phi\).  
**VALID WHEN:** pipe motion exceeds clearance.  
**REGIME:** local centralizer geometry.  
**NOTES:** for \(\|\vec U_r\|\le\phi\), the centralizer is not yet engaged.

**EQ [5]:**
\[
\delta_i=\max\!\left[
\frac{D_{\max}}{2}
-
\left(
\sqrt{\left(\frac{D_{\text{hole}}}{2}\right)^2-\left(U_{r,\text{cent}}\sin\alpha_i\right)^2}
-U_{r,\text{cent}}\cos\alpha_i
\right),
0
\right]
\]
**COMPUTES:** deflection of blade \(i\).  
**INPUTS REQUIRED:** \(D_{\max},D_{\text{hole}},U_{r,\text{cent}},\alpha_i\).  
**VALID WHEN:** geometric blade compression model.  
**REGIME:** theoretical bow-spring contact.  
**NOTES:** zero deflection implies loss of contact.

**EQ [4]:**
\[
F_i=k_{\text{blade}}\delta_i^p
\]
**COMPUTES:** scalar force magnitude on blade \(i\).  
**INPUTS REQUIRED:** \(k_{\text{blade}},\delta_i,p\).  
**VALID WHEN:** normal operating regime, between \(D_{\max}\) and \(D_{\min}\).  
**REGIME:** blade constitutive law.  
**NOTES:** linear if \(p=1\).

**EQ [1]:**
\[
\vec F_{\text{restoring}}=\sum_{i=1}^{n}\vec F_i
\]
**COMPUTES:** restoring-force vector.  
**INPUTS REQUIRED:** all blade-force vectors.  
**VALID WHEN:** blades may be active or inactive individually.  
**REGIME:** force assembly.  
**NOTES:** **vector sum**, not scalar sum.

**EQ [6]:**
\[
F_{fa}=\mu_{\text{hole}}\sum_{i=1}^{n}\|\vec F_i\|
\]
**COMPUTES:** axial friction force.  
**INPUTS REQUIRED:** blade-force magnitudes, \(\mu_{\text{hole}}\).  
**VALID WHEN:** bow-spring blades slide against hole.  
**REGIME:** axial drag.  
**NOTES:** **scalar sum of magnitudes**.

**EQ [7]:**
\[
F_{fr}=\mu_{\text{steel-steel}}\left\|\sum_{i=1}^{n}\vec F_i\right\|
\]
**COMPUTES:** tangential friction due to pipe rotation in centralizer rings.  
**INPUTS REQUIRED:** restoring-force resultant, \(\mu_{\text{steel-steel}}\).  
**VALID WHEN:** pipe rotates inside centralizer rings.  
**REGIME:** tangential friction.  
**NOTES:** magnitude of the restoring-force resultant.

**EQ [8]:**
\[
M_{fr}=\frac{D_{\text{pipe}}}{2}F_{fr}
\]
**COMPUTES:** friction torque on casing.  
**INPUTS REQUIRED:** \(D_{\text{pipe}},F_{fr}\).  
**VALID WHEN:** tangential friction exists at rings.  
**REGIME:** torque calculation.

**EQ [9]:**
\[
k_{\text{blade}}=
\frac{F_{\text{restoring,test}}}
{F_{\text{restoring,model}}(k_{\text{blade}}=1,p)}
\]
**COMPUTES:** blade rigidity from first API data point.  
**INPUTS REQUIRED:** measured restoring force, model restoring force with unit rigidity, chosen \(p\).  
**VALID WHEN:** calibration using 67% standoff point.  
**REGIME:** calibration step 1.

**EQ [10]:**
\[
\mu_{\text{test}}=
\frac{F_{\text{running,test}}}{\sum_{i=1}^{n}\|\vec F_i\|}
\]
**COMPUTES:** estimated friction coefficient in test condition.  
**INPUTS REQUIRED:** running-force test result, blade forces under test configuration.  
**VALID WHEN:** blade rigidity and \(p\) are known.  
**REGIME:** calibration/post-processing.

### 3.4 FE equilibrium and body contact

**EQ [12]:**
\[
\mathbf K\vec X=\vec F_o+\vec F_c(\vec X)
\]
**COMPUTES:** FE static equilibrium statement.  
**INPUTS REQUIRED:** stiffness matrix, constant loads, state-dependent contact loads.  
**VALID WHEN:** small-displacement stiff-string FE model.  
**REGIME:** global equilibrium.

**EQ [15]:**
\[
r=\sqrt{u^2+v^2},\qquad
\Delta r=r+R_o-R_w,\qquad
\vec f_c=f_{cr}\vec e_r+f_{c\theta}\vec e_\theta+f_{cz}\vec e_z
\]
**COMPUTES:** body-contact penetration and contact-force decomposition.  
**INPUTS REQUIRED:** \(u,v,R_o,R_w\).  
**VALID WHEN:** contact at a string section.  
**REGIME:** body contact.

**EQ [16]:**
\[
f_{cr}=-k_c\Delta r\,H(\Delta r)
\]
**COMPUTES:** ideal normal contact force.  
**INPUTS REQUIRED:** \(k_c,\Delta r\).  
**VALID WHEN:** discontinuous unilateral contact law.  
**REGIME:** body normal contact.  
**NOTES:** negative sign makes the force restorative/opposing penetration.

**EQ [17]:**
\[
f_{cr}=-k_c\Delta r\,h(\Delta r),
\qquad
h(x)=\left(1-\frac{1}{1+(x/\lambda)^2}\right)H(x)
\]
**COMPUTES:** smoothed normal contact force.  
**INPUTS REQUIRED:** \(k_c,\Delta r,\lambda\).  
**VALID WHEN:** numerical solution with smooth contact activation.  
**REGIME:** body normal contact, smoothed.  
**NOTES:** \(h(\lambda)=1/2\); \(h(x)=0\) for \(x<0\).

**EQ [18]:**
\[
f_{c\theta}=\mu_\theta f_{cr},\qquad
f_{cz}=\mu_z f_{cr}
\]
**COMPUTES:** tangential and axial friction components.  
**INPUTS REQUIRED:** \(f_{cr},\mu_\theta,\mu_z\).  
**VALID WHEN:** combined contact and slip.  
**REGIME:** body friction.

**EQ [19]:**
\[
\mu_\theta=\frac{V_{g\theta}}{V_g+V_g^\varepsilon}\mu(V_g),\qquad
\mu_z=\frac{V_{gz}}{V_g+V_g^\varepsilon}\mu(V_g)
\]
\[
V_g=\sqrt{V_{g\theta}^2+V_{gz}^2},\qquad
V_{gz}=V_a,\qquad
V_{g\theta}=R\Omega
\]
**COMPUTES:** directional friction coefficients.  
**INPUTS REQUIRED:** slip-velocity components, guard velocity, total friction law.  
**VALID WHEN:** simultaneous translation and rotation.  
**REGIME:** body friction split.  
**NOTES:** guard velocity avoids division by zero.

**EQ [20]:**
\[
\mu(V_g)=\frac{2}{\pi}\arctan\!\left(\frac{V_g}{V_g^{\lim}}\right)
\left(
\mu_d+(\mu_s-\mu_d)\exp\!\left(-\frac{V_g}{V_g^{0}}\right)
\right)
\]
**COMPUTES:** total velocity-dependent friction coefficient.  
**INPUTS REQUIRED:** \(V_g,\mu_s,\mu_d,V_g^0,V_g^{\lim}\).  
**VALID WHEN:** smoothed static-to-dynamic transition.  
**REGIME:** body friction law.

**EQ [21]:**
\[
\vec f_c
=
f_{cr}(\Delta r)\,
{}^{\mathrm t}
\left(
\frac{u-\mu_\theta v}{r},
\frac{v+\mu_\theta u}{r},
\mu_z
\right)
\]
\[
\vec m_c
=
f_{cr}(\Delta r)\,R_e\,
{}^{\mathrm t}
\left(
\mu_z\frac{v}{r},
-\mu_z\frac{u}{r},
\mu_\theta
\right)
\]
**COMPUTES:** local body-contact force and moment vectors.  
**INPUTS REQUIRED:** \(f_{cr},u,v,r,\mu_\theta,\mu_z,R_e\).  
**VALID WHEN:** local-frame representation of contact.  
**REGIME:** body contact to FE nodal force conversion.

### 3.5 FE bow-spring contact

**EQ [22]:**
\[
\alpha_{b,i}=\alpha_{b0}+(i-1)\frac{2\pi}{N_b},\qquad
\vec e_{br,i}=
{}^{\mathrm t}(\cos\alpha_{b,i},\sin\alpha_{b,i})
\]
\[
\vec r_{b,i}=(r-r_0)\vec e_r+R_b\vec e_{br,i}
\]
**COMPUTES:** blade directions and blade-position vectors.  
**INPUTS REQUIRED:** \(N_b,\alpha_{b0},r,r_0,R_b,\vec e_r\).  
**VALID WHEN:** local plane of the cross-section.  
**REGIME:** bow-contact geometry.

**EQ [23]:**
\[
\Delta r_{b,i}=r_{b,i}-R_w,\qquad
\Delta r_i=\Delta r_{b,i}-\Delta r_{b,\max},\qquad
\Delta r_{b,\max}=R_b-R_{b,\min}
\]
\[
f_{cbr,i}=
-k_b\Delta r_{b,i}^{\,p}H(\Delta r_{b,i})
-\left(k_c\Delta r_i-k_b\Delta r_{b,i}^{\,p}+k_b\Delta r_{b,\max}^{\,p}\right)H(\Delta r_i)
\]
**COMPUTES:** two-regime bow radial contact force with discontinuous activation.  
**INPUTS REQUIRED:** \(k_b,p,\Delta r_{b,i},\Delta r_i,\Delta r_{b,\max},k_c\).  
**VALID WHEN:** bow FE contact.  
**REGIME:** un-smoothed two-regime bow law.

**EQ [24]:**
\[
f_{cbr,i}=
-k_b\Delta r_{b,i}^{\,p}h(\Delta r_{b,i})
-\left(k_c\Delta r_i-k_b\Delta r_{b,i}^{\,p}+k_b\Delta r_{b,\max}^{\,p}\right)h(\Delta r_i)
\]
**COMPUTES:** smoothed two-regime bow radial contact force.  
**INPUTS REQUIRED:** same as Eq. (23), replacing \(H\) by \(h\).  
**VALID WHEN:** Newton–Raphson implementation.  
**REGIME:** smoothed bow FE contact.

**EQ [25]:**
\[
f_{cbz,i}=\mu_z f_{cbr,i},
\qquad
f_{cbz}=\mu_z\sum_{i=1}^{N_b}f_{cbr,i}
\]
**COMPUTES:** axial friction from bow contacts.  
**INPUTS REQUIRED:** \(f_{cbr,i},\mu_z\).  
**VALID WHEN:** bow–hole axial friction.  
**REGIME:** bow axial friction.

**EQ [26]:**
\[
\vec f_{cbr}=\sum_{i=1}^{N_b} f_{cbr,i}\vec e_{br,i}
\]
**COMPUTES:** resultant radial contact force of all blades.  
**INPUTS REQUIRED:** blade radial forces and directions.  
**VALID WHEN:** bow resultant assembly.  
**REGIME:** bow force assembly.  
**NOTES:** vector sum.

**EQ [27]:**
\[
\vec f_{cb\theta}=\mu_\theta \vec e_w\times \vec f_{cbr}
=
\sum_{i=1}^{N_b}\mu_\theta f_{cbr,i}\vec e_{b\theta,i}
\]
**COMPUTES:** resultant tangential friction force of bow-spring in FE.  
**INPUTS REQUIRED:** \(\mu_\theta,\vec e_w,\vec f_{cbr}\).  
**VALID WHEN:** pipe rotates inside rings.  
**REGIME:** bow tangential friction.

**EQ [28]:**
\[
\vec e_{b\theta,i}=
{}^{\mathrm t}(-\sin\alpha_{b,i},\cos\alpha_{b,i}),
\qquad
\vec f_{cb\theta,i}=\mu_\theta f_{cbr,i}\vec e_{b\theta,i},
\qquad
\vec f_{cb\theta}=\sum_{i=1}^{N_b}\vec f_{cb\theta,i}
\]
**COMPUTES:** blade-by-blade tangential friction assembly.  
**INPUTS REQUIRED:** \(\alpha_{b,i},\mu_\theta,f_{cbr,i}\).  
**VALID WHEN:** equivalent representation of Eq. (27).  
**REGIME:** bow tangential friction.

**EQ [29]:**
\[
\vec f_{cb,i}
=
f_{cbr,i}(r)\,
{}^{\mathrm t}
\left(
c_{cbu,i},
c_{cbv,i},
\mu_z
\right)
\]
\[
\vec m_{cb,i}
=
R_e\,f_{cbr,i}(r)\,
{}^{\mathrm t}
\left(
\mu_z\frac{v}{r},
-\mu_z\frac{u}{r},
\mu_\theta\left(c_{cbv,i}\frac{u}{r}-c_{cbu,i}\frac{v}{r}\right)
\right)
\]
\[
c_{cbu,i}=\cos\alpha_{b,i}-\mu_\theta\sin\alpha_{b,i},
\qquad
c_{cbv,i}=\sin\alpha_{b,i}+\mu_\theta\cos\alpha_{b,i}
\]
**COMPUTES:** local force and moment vectors for blade \(i\).  
**INPUTS REQUIRED:** \(f_{cbr,i},\alpha_{b,i},u,v,r,\mu_\theta,\mu_z,R_e\).  
**VALID WHEN:** local FE assembly of bow contact.  
**REGIME:** bow contact to FE nodal force conversion.  
**NOTES:** the transcription with \(f_{cbv,i}\) in the second component is a typo; the correct form is \(c_{cbv,i}\). This correction is explicitly adopted here.

### 3.6 Newton–Raphson

**Unnumbered residual definition in Sec. 3.4:**
\[
\mathcal R(\vec X)=\mathbf K\vec X-\vec F_o-\vec F_c(\vec X)
\]
**COMPUTES:** nonlinear residual.  
**INPUTS REQUIRED:** \(\mathbf K,\vec X,\vec F_o,\vec F_c(\vec X)\).  
**REGIME:** nonlinear equilibrium solve.

**Unnumbered Newton update in Sec. 3.4:**
\[
\vec X_{k+1}
=
\vec X_k
-
\left(\nabla \mathcal R(\vec X_k)\right)^{-1}
\mathcal R(\vec X_k)
\]
**COMPUTES:** iterative state update.  
**INPUTS REQUIRED:** current residual and Jacobian.  
**REGIME:** Newton–Raphson solver.

**EQ [35] visible part:**
\[
\frac{\partial m_{cz}}{\partial u}
=
\mu_\theta R_e\, f'_{cr}(\Delta r)\,\frac{u}{r},
\qquad
\frac{\partial m_{cz}}{\partial v}
=
\mu_\theta R_e\, f'_{cr}(\Delta r)\,\frac{v}{r}
\]
\[
f'_{cr}(x)=-k_c\left(h(x)+x\,h'(x)\right),
\qquad
h'(x)=
\frac{2x/\lambda^2}{\left(1+(x/\lambda)^2\right)^2}\,H(x)
\]
**COMPUTES:** visible Jacobian terms for body contact and derivative definitions.  
**INPUTS REQUIRED:** \(k_c,h,h',x,\mu_\theta,R_e,u,v,r\).  
**VALID WHEN:** smoothed contact active or differentiable at current point.  
**REGIME:** body-contact Jacobian.  
**NOTES:** only the visible part of Eq. (35) was available.

**EQ [36]:**
\[
\frac{\partial \Delta r_{b,i}}{\partial u}
=
\frac{(r^3-r_0v^2)\,u_{b,i}+r_0uv\,v_{b,i}}{r_{b,i}\,r^3},
\qquad
\frac{\partial \Delta r_{b,i}}{\partial v}
=
\frac{r_0uv\,u_{b,i}+(r^3-r_0u^2)\,v_{b,i}}{r_{b,i}\,r^3}
\]
\[
\frac{\partial f_{cbx,i}}{\partial u}
=
f'_{cbr,i}(r)\,c_{cbu,i}\,
\frac{\partial \Delta r_{b,i}}{\partial u},
\qquad
\frac{\partial f_{cbx,i}}{\partial v}
=
f'_{cbr,i}(r)\,c_{cbu,i}\,
\frac{\partial \Delta r_{b,i}}{\partial v}
\]
\[
\frac{\partial f_{cby,i}}{\partial u}
=
f'_{cbr,i}(r)\,c_{cbv,i}\,
\frac{\partial \Delta r_{b,i}}{\partial u},
\qquad
\frac{\partial f_{cby,i}}{\partial v}
=
f'_{cbr,i}(r)\,c_{cbv,i}\,
\frac{\partial \Delta r_{b,i}}{\partial v}
\]
\[
\frac{\partial f_{cbz,i}}{\partial u}
=
\mu_z\,f'_{cbr,i}(r)\,\frac{u}{r},
\qquad
\frac{\partial f_{cbz,i}}{\partial v}
=
\mu_z\,f'_{cbr,i}(r)\,\frac{v}{r}
\]
\[
\frac{\partial m_{cbz,i}}{\partial u}
=
\mu_\theta R_e
\left(
f'_{cbr,i}(r)\,\frac{1}{r}\,
\frac{\partial \Delta r_{b,i}}{\partial u}\,
(c_{cbv,i}u-c_{cbu,i}v)
+
f_{cbr,i}(r)\,\frac{v}{r^3}\,(c_{cbv,i}v+c_{cbu,i}u)
\right)
\]
\[
\frac{\partial m_{cbz,i}}{\partial v}
=
\mu_\theta R_e
\left(
f'_{cbr,i}(r)\,\frac{1}{r}\,
\frac{\partial \Delta r_{b,i}}{\partial v}\,
(c_{cbv,i}u-c_{cbu,i}v)
-
f_{cbr,i}(r)\,\frac{u}{r^3}\,(c_{cbv,i}v+c_{cbu,i}u)
\right)
\]
\[
f'_{cbr,i}(r)
=
-k_b\Delta r_{b,i}^{\,p}\left(h'(\Delta r_{b,i})-h'(\Delta r_i)\right)
-k_b\,p\,\Delta r_{b,i}^{\,p-1}\left(h(\Delta r_{b,i})-h(\Delta r_i)\right)
-k_c\left(h(\Delta r_i)+\Delta r_i h'(\Delta r_i)\right)
-k_b\Delta r_{b,\max}^{\,p}h'(\Delta r_i)
\]
**COMPUTES:** visible Jacobian terms for bow-spring contact.  
**INPUTS REQUIRED:** bow geometry derivatives, contact derivatives, \(u,v,r\), coefficients \(c_{cbu,i},c_{cbv,i}\), \(R_e,\mu_\theta,\mu_z\).  
**VALID WHEN:** smoothed bow-contact regime.  
**REGIME:** bow-contact Jacobian.

---

## SECTION 4 — COORDINATE FRAMES AND TRANSFORMATIONS

### 4.1 Global frame

The global frame is \((\vec e_X,\vec e_Y,\vec e_Z)\), where \(\vec e_Z=\vec g/\|\vec g\|\). Thus, the global vertical is aligned with gravity, not arbitrarily chosen.

### 4.2 Local nodal frame

The local nodal frame is \((\vec e_u,\vec e_v,\vec e_w)\). The tangent vector defines \(\vec e_w=\vec t\). When \(\vec t\) is not collinear with \(\vec e_Z\), the remaining basis vectors are obtained from Eq. (11). The paper interprets \(\vec e_u\) as a high-side direction and \(\vec e_v\) as a right-side direction. In vertical sections, where \(t_Z^2=1\), the standard formula is singular, and the paper states that a combination of \(\vec e_X\) and \(\vec e_Y\) is used to maintain continuous frames.

### 4.3 Body-contact frame

In a local cross-section, \(\vec r={}^\mathrm t(u,v)\), \(r=\|\vec r\|\), \(\vec e_r=\vec r/\|\vec r\|\), and \(\vec e_\theta=\vec e_z\times \vec e_r\). The contact force is then decomposed as \(f_{cr}\vec e_r+f_{c\theta}\vec e_\theta+f_{cz}\vec e_z\). This is the basis for Eqs. (15)–(21).

### 4.4 Blade frame

For blade \(i\), the blade radial direction is
\[
\vec e_{br,i}={}^\mathrm t(\cos\alpha_{b,i},\sin\alpha_{b,i}),
\]
and the blade orthoradial direction is
\[
\vec e_{b\theta,i}={}^\mathrm t(-\sin\alpha_{b,i},\cos\alpha_{b,i}).
\]
These define the radial and orthoradial components of the bow-spring contact forces in Eqs. (26)–(28).

### 4.5 Transformations

The body-contact force in the local frame is given explicitly by Eq. (21). The bow-contact force in the local frame is given explicitly by Eq. (29), where \(c_{cbu,i}\) and \(c_{cbv,i}\) encode the combination of blade orientation and tangential friction coefficient. The associated moments use \(\vec m=R_e \vec e_r\times \vec f\).

---

## SECTION 5 — BOW-SPRING MODEL

### 5.1 Geometry

The centralizer geometry includes \(D_{\max},D_{\min},D_{\text{int}},D_{\text{pipe}},D_{\text{hole}}\), blade angle \(\alpha_i\), and clearance \(\phi\). The paper explicitly introduces clearance through Eq. (2), then defines the effective centralizer displacement by Eq. (3), making the activation of the centralizer conditional on the pipe first consuming the ring-to-pipe gap. In the FE bow-contact model, the analogous gap is \(r_0\), the initial pipe-to-ring clearance.

### 5.2 Blade deflection

For the theoretical bow model, blade deflection is defined by Eq. (5), which measures the difference between the free maximum radius and the deformed blade radius. The max-with-zero structure means a blade can lose contact entirely. The results section states that slope changes in restoring-force curves occur when one or several top blades lose contact with the hole. In the horizontal section, the paper reports that the top blade can lose contact, leaving only four active blade forces at a centralizer.

### 5.3 Blade force law

The local blade constitutive law is \(F_i=k_{\text{blade}}\delta_i^p\). The model is linear when \(p=1\). It remains valid only in the normal operating range between \(D_{\max}\) and \(D_{\min}\). Once the blade reaches the minimum-diameter state, the paper treats the blade as effectively infinitely rigid and switches the additional resistance to the contact stiffness \(k_c\). In the FE implementation, that transition is embedded explicitly in Eqs. (23)–(24).

### 5.4 Force assembly

The paper distinguishes three assemblies. First, restoring force is a **vector sum**:
\[
\vec F_{\text{restoring}}=\sum_i \vec F_i.
\]
Second, axial friction is a **scalar sum of magnitudes**:
\[
F_{fa}=\mu_{\text{hole}}\sum_i \|\vec F_i\|.
\]
Third, tangential friction is based on the **resultant vector norm**:
\[
F_{fr}=\mu_{\text{steel-steel}}\left\|\sum_i \vec F_i\right\|.
\]
The physical reason is explicit in the paper: axial friction acts at the blade–hole interface and all those friction contributions act along the same axial direction, while tangential friction is generated at the pipe–ring interface and is driven by the resultant normal reaction transmitted to the pipe.

### 5.5 Friction torque

The tangential friction torque on the casing is
\[
M_{fr}=\frac{D_{\text{pipe}}}{2}F_{fr}.
\]
The contact surface responsible is the pipe–centralizer ring contact, not the blade–hole contact. The paper states that steel–steel friction is always used here because the rotating contact is between the casing/pipe and the rings.

---

## SECTION 6 — FINITE ELEMENT FORMULATION

### 6.1 Element and DOF definition

The string is discretized into a series of two-node beam elements. Each node has six DOFs: translations \(u,v,w\) and rotations \(\theta_x,\theta_y,\theta_z\). The paper identifies \(u,v\) as lateral displacements, \(w\) as axial displacement, \(\theta_x,\theta_y\) as bending rotations, and \(\theta_z\) as torsion.

### 6.2 Stiffness matrix

The global stiffness matrix is \(\mathbf K\), based on beam elements following Lalanne and Ferraris (1998), as cited by the paper. The paper does not reproduce the full beam element matrix in the visible extracts; it references the beam formulation and states that \(\mathbf K\) is assembled globally.

### 6.3 Equilibrium equation

The static equilibrium equation is
\[
\mathbf K\vec X=\vec F_o+\vec F_c(\vec X).
\]
The linear part is \(\mathbf K\vec X\) plus the constant loads in \(\vec F_o\). The nonlinear part is entirely in \(\vec F_c(\vec X)\), due to contact and friction dependence on the current state.

### 6.4 Initial curvature stress

Because the well neutral line is curved but the FE elements are initially straight, the paper states that curvature-induced initial stress must be included. It proposes a step-by-step construction: deform one element at each step, keep previous elements fixed, and move following elements as a rigid body. The exact algebraic expression of this procedure was not visible in the available extracts.

### 6.5 Gravity and buoyancy

Gravity and buoyancy enter through
\[
\vec q=(\rho-\rho_f)A\vec g.
\]
Equivalent nodal forces and moments are then computed by Eq. (13), including both translational forces and couples due to the body-force distribution. This means the FE model transfers body forces into both nodal forces and nodal moments.

---

## SECTION 7 — CONTACT FORCES IN FE MODEL

### 7.1 Body contact detection

The penetration criterion is
\[
\Delta r=r+R_o-R_w.
\]
Contact is active when \(\Delta r>0\). The ideal activation uses \(H(\Delta r)\), but the implementation uses the smooth function \(h(\Delta r)\) from Eq. (17). The paper states explicitly that this regularization is introduced for numerical reasons, is exactly zero for negative penetration, and satisfies \(h(\lambda)=1/2\).

### 7.2 Normal contact force

The body normal contact force is
\[
f_{cr}=-k_c\Delta r\,h(\Delta r).
\]
The sign is restorative: positive penetration generates a force opposing that penetration. Physically, \(k_c\) is the contact stiffness that represents the effective radial resistance of the borehole/contact interface. In the fully compressed bow regime, the same \(k_c\) is reused to govern the post-consolidation branch.

### 7.3 Friction forces on body

Body friction is decomposed into tangential and axial components via Eq. (18), then split by velocity direction via Eq. (19). The total friction law in Eq. (20) blends static and dynamic friction using an exponential decay and an \(\arctan\) smoothing factor. The paper fixes \(V_g^\varepsilon=10^{-7}\,\text{m/s}\) and \(V_g^{\lim}=1\,\text{mm/s}\). The first avoids division by zero; the second removes the discontinuity of the friction law at zero slip.

### 7.4 Bow-spring contact forces in FE

The FE bow-contact geometry is built blade by blade using \(\alpha_{b,i}\), \(\vec e_{br,i}\), \(R_b\), \(r_0\), and \(\vec r_{b,i}\). The force law in Eqs. (23)–(24) has two regimes: a blade-law regime controlled by \(k_b\) and \(p\), and a post-minimum-diameter regime controlled by \(k_c\). Axial friction is summed directly across blades along the same axial direction, while tangential friction is built from the resultant of all blade radial reactions and rotated into an orthoradial direction. Eq. (29) then maps each blade’s contribution into local FE force and moment components through \(c_{cbu,i}\) and \(c_{cbv,i}\).

---

## SECTION 8 — NEWTON-RAPHSON SOLVER

### 8.1 Formulation

The residual is
\[
\mathcal R(\vec X)=\mathbf K\vec X-\vec F_o-\vec F_c(\vec X),
\]
and the Newton update is
\[
\vec X_{k+1}
=
\vec X_k
-
\left(\nabla\mathcal R(\vec X_k)\right)^{-1}\mathcal R(\vec X_k).
\]
The Jacobian is therefore
\[
\nabla\mathcal R(\vec X)=\mathbf K-\nabla\vec F_c(\vec X).
\]
The paper states that the contact-force Jacobian is block-diagonal by node under the adopted definitions, so only diagonal nodal blocks are nonzero.

### 8.2 Jacobian of body contact forces

Only part of Eq. (35) was available in the provided extracts. The visible exact expressions are
\[
\frac{\partial m_{cz}}{\partial u}
=
\mu_\theta R_e f'_{cr}(\Delta r)\frac{u}{r},
\qquad
\frac{\partial m_{cz}}{\partial v}
=
\mu_\theta R_e f'_{cr}(\Delta r)\frac{v}{r},
\]
with
\[
f'_{cr}(x)=-k_c\left(h(x)+x h'(x)\right),
\qquad
h'(x)=\frac{2x/\lambda^2}{(1+(x/\lambda)^2)^2}H(x).
\]
The paper also states, in the solver discussion, that only the derivatives with respect to the local active displacement variables are nonzero in the nodal contact block; unrelated node-to-node derivatives are zero.

### 8.3 Jacobian of bow-spring contact forces

Eq. (36) was visible and includes the chain-rule derivatives of \(\Delta r_{b,i}\), the force-component derivatives \(\partial f_{cbx,i}/\partial u\), \(\partial f_{cby,i}/\partial v\), the axial derivatives \(\partial f_{cbz,i}/\partial u,\partial f_{cbz,i}/\partial v\), the moment derivatives \(\partial m_{cbz,i}/\partial u,\partial m_{cbz,i}/\partial v\), and the full derivative \(f'_{cbr,i}(r)\) of the two-regime smoothed bow law. This is the key analytic Jacobian required for a robust Newton implementation of the detailed bow-spring model.

❓ **NOT SPECIFIED IN PAPER (visible extracts):** no explicit convergence tolerance, residual norm threshold, or line-search strategy was visible in the extracts available here.

---

## SECTION 9 — CALIBRATION ALGORITHM

**STEP 1 — choose test configuration.**  
Use the manufacturer test data. The paper states that only the open-hole-size test provides both the restoring force at 67% standoff and the standoff at the API-recommended restoring force, so that test is used for calibration. If the test configuration is unspecified in the technical sheet, configuration (a) of API Spec 10D is used by default. If the blade angle is unspecified, \(\alpha=0\) is used by default.

**STEP 2 — set an initial stiffness power.**  
The calibration diagram starts with \(p_2=1.0\) and then sets \(p_1:=p_2\).

**STEP 3 — compute blade rigidity from the first API point.**  
Use the 67% standoff restoring-force test point and Eq. (9):
\[
k_{\text{blade}}=
\frac{F_{\text{restoring,test}}}
{F_{\text{restoring,model}}(k_{\text{blade}}=1,p)}.
\]
This fixes \(k_{\text{blade}}\) for the current trial value of \(p\).

**STEP 4 — evaluate the second API point.**  
With the current \((k_{\text{blade}},p)\), compute the model restoring force at the second API datum, namely the API-recommended restoring-force condition corresponding to the reported standoff. Compare the model value to the target test value. The calibration diagram checks whether \(|F_{\text{res,API1}}-F_{\text{res,API}}|<\varepsilon\).

**STEP 5 — compute numerical derivative with respect to \(p\).**  
The diagram perturbs \(p\) by \(dp\), recomputes \(k_{\text{blade,2}}\) and the predicted restoring force, then estimates the numerical derivative:
\[
VF_{\text{res,API}}=\frac{F_{\text{res,API2}}-F_{\text{res,API1}}}{dp}.
\]

**STEP 6 — update \(p\).**  
The next stiffness power is
\[
p_2:=p_1+\frac{F_{\text{res,API}}-F_{\text{res,API1}}}{VF_{\text{res,API}}}.
\]
Iterate until the criterion is met.

**STEP 7 — estimate test friction coefficient.**  
Once \(k_{\text{blade}}\) and \(p\) are available, compute the blade forces under the running-force test configuration and estimate \(\mu_{\text{test}}\) using Eq. (10).

**Default assumptions when data are missing.**  
The paper explicitly uses: API test configuration (a) if the configuration is not specified; \(\alpha=0\) if blade angle is not specified; and, for the example calibration, the initial centralizer maximum diameter \(D_{\max}=7.44\) in when the post-restriction \(D_{\max}\) is not reported. It then studies sensitivity by varying \(D_{\max}\) from 7.30 in to 7.44 in.

---

## SECTION 10 — NUMERICAL CONSTANTS (HARDCODED VALUES)

| Constant | Value | Units | Source in paper | Physical justification |
|---|---:|---:|---|---|
| \(V_g^\varepsilon\) | \(10^{-7}\) | m/s | Eq. (19) text | avoid division by zero in friction split |
| \(V_g^{\lim}\) | \(1\) | mm/s | Eq. (20) text | smooth friction law at zero slip |
| initial \(p_2\) in calibration flowchart | \(1.0\) | dimensionless | Fig. 8 | starting guess for stiffness exponent |
| default blade angle when unspecified | \(0\) | deg | Sec. 4.1 discussion | default working angle |
| default API configuration when unspecified | configuration (a) | — | Sec. 2 text | default restoring-force test setup |
| mud weight | \(1.2\) | SG | Table 4 | case-study parameter |
| rotation speed | \(60\) | rpm | Table 4 | case-study parameter |
| tripping speed | \(200\) | m/h | Table 4 | case-study parameter |
| cased-hole friction | \(0.15\) | dimensionless | Table 4 | case-study parameter |
| open-hole friction | \(0.20\) | dimensionless | Table 4 | case-study parameter |

The paper also uses specific calibrated values in the application case: \(D_{\max}=7.35\) in, \(p=1.17509\), \(k_{\text{blade}}=154024.29\ \text{kgf}/\text{m}^p\), and \(\mu_{\text{test}}=0.05424\) for the 6.5 in restriction-size running-force test. These are not general hardcoded constants, but they are fixed reference values for the paper’s demonstration case.

---

## SECTION 11 — TEST CASES FROM PAPER

### TEST CASE 1 — Centralizer calibration

**Inputs recovered from Table 2:**

- Restriction size: 6.5 in = 165.1 mm  
- Running force at restriction size: 203.94 kgf = 583.3 lbf  
- Restoring force at 67% standoff for restriction test: n/a  
- Standoff at API-recommended restoring force for restriction test: n/a  

- Open-hole size: 7.25 in = 184.15 mm  
- Running force at open-hole size: 61.18 kgf = 135.8 lbf  
- Restoring force at 67% standoff: 10.17 kN = 2287.3 lbf  
- Standoff at API-recommended restoring force: 93%

**Calibration conditions and outcomes reported:**

- Only the open-hole-size test can be used for the two-point calibration because it contains both API calibration datapoints.  
- If post-restriction \(D_{\max}\) is not specified, the paper first uses the initial \(D_{\max}=7.44\) in.  
- Sensitivity study varies \(D_{\max}\) from 7.30 in to 7.44 in.  
- At \(D_{\max}=7.30\) in, the model is almost linear with \(p=1.00855\).  
- For the case study, the paper adopts \(D_{\max}=7.35\) in, \(p=1.17509\), \(k_{\text{blade}}=154024.29\ \text{kgf}/\text{m}^p\), and estimated \(\mu_{\text{test}}=0.05424\).

❓ **NOT SPECIFIED IN PAPER (available extracts):** the full contents of Table 1 were not visible in the accessible excerpts, so the exact general characteristics listed there could not be reproduced verbatim here.

### TEST CASE 2 — FE model validation against Menand et al. (2006)

**String configuration from Table 3:**

- Casing shoe: length 0.5 m, OD \(4\frac34\) in, ID \(2\frac14\) in, total length 0.5 m, mass 34.8 kg, total mass 34.8 kg, no tooljoint OD.  
- Casing \(\times 236\): length 2938.2 m, OD \(4\frac12\) in, ID 3.92 in, total length 2938.7 m, mass 59029.12 kg, total mass 59063.92 kg, OD tooljoint \(4\frac{31}{32}\) in.  
- Drill-pipe: length 9.5 m, OD \(3\frac12\) in, ID \(2\frac58\) in, total length 2948.2 m, mass 229.15 kg, total mass 59293.07 kg, OD tooljoint 5 in.

**Well / simulation parameters from Table 4 and text:**

- Mud weight 1.2 SG  
- Rotation speed 60 rpm  
- Tripping speed 200 m/h  
- Cased-hole friction 0.15  
- Open-hole friction 0.20  
- String-bottom MD = 4000 m  
- Well trajectory and geometry as in Figs. 13 and 14.

**Expected outcome:** very good coherence between the new FE model and Menand et al. (2006) for tension, torque, lateral displacements, and normal contact forces; only small negligible differences remain due to different resolution methods and the 3D tortuous trajectory.

### TEST CASE 3 — Torque-and-drag case with bow-spring centralizers

**Added centralizer parameters and setup:**

- Same global well and string parameters as Test Case 2.  
- Centralizer type from Sec. 4.1.  
- Use calibrated parameters for \(D_{\max}=7.35\) in: \(p=1.17509\), \(k_{\text{blade}}=154024.29\ \text{kgf}/\text{m}^p\).  
- Estimated \(\mu_{\text{test}}\) for the 6.5 in restriction-size test: 0.05424.  
- All centralizers oriented with angle zero relative to local \(\vec e_u\).  
- Placement densities: one centralizer every 3, then 5, then 11 casing elements of 12.45 m.  
- Well includes a 500 m section of 6.5 in hole before the 7.25 in open hole.

**Expected behavior:**

- In the 6.5 in section, all blades are highly compressed and blade forces are large.  
- In the horizontal 7.25 in section, the string lies on the borehole and the top blade may lose contact, leaving four active blade forces.  
- Theoretical recalculation from FE lateral displacement should match FE restoring force, axial friction, and friction torque well.  
- Centralizer axial friction from 1000 m to 1500 m MD is about 550 kgf, much larger than the restriction-test running force 203.94 kgf because the case-study cased-hole friction coefficient is 0.15, whereas the estimated restriction-test friction factor is only 0.05424.

### TEST CASE 4 — Standoff and surface tension results

**Expected standoff outcomes:**

- With bow-spring centralizers, more than 90% standoff can be ensured in the horizontal open-hole section from 2000 m to 4000 m.  
- Without bow-spring centralizers, standoff is less than 20% everywhere in the same comparison.  
- The resulting standoff at centralizers is close to the 93% standoff from the technical sheet.

**Expected surface-tension evolution:**

- The first centralizer enters the 6.5 in hole at about 1036 m MD.  
- The first centralizer exits the 6.5 in hole at about 1536 m MD.  
- From 1000 m to 1200 m, centralizers enter every 37 m.  
- From 1200 m to 1500 m, centralizers enter every 62 m.  
- From 1500 m onward, the dent period is about 137 m, matching the spacing of the last centralizers on the string.

---

## SECTION 12 — CRITICAL WARNINGS FOR IMPLEMENTATION

**WARNING 1:** confusing restoring-force assembly with axial-friction assembly.  
**CORRECT BEHAVIOR:** restoring force is a vector sum; axial friction is a scalar sum of blade-force magnitudes.  
**COMMON MISTAKE:** summing all blade forces as scalars in both places.  
**HOW TO TEST:** arrange symmetric blade forces around the circumference. Restoring force should cancel to near zero, while axial friction remains positive.

**WARNING 2:** using steel–steel friction for axial blade drag.  
**CORRECT BEHAVIOR:** axial friction uses hole/contact coefficient \(\mu_{\text{hole}}\); tangential friction at rings uses steel–steel coefficient.  
**COMMON MISTAKE:** one single friction coefficient everywhere.  
**HOW TO TEST:** reproduce the paper’s explanation for the large difference between 550 kgf case-study axial friction and 203.94 kgf test running force.

**WARNING 3:** activating centralizer stiffness before clearance is consumed.  
**CORRECT BEHAVIOR:** use Eq. (3) or \(r_0\)-based geometry so restoring force is zero for sufficiently small radial displacement.  
**COMMON MISTAKE:** applying blade compression from \(U_r=0\).  
**HOW TO TEST:** small lateral displacement should yield zero restoring force until the clearance threshold is crossed.

**WARNING 4:** ignoring the two-regime nature of the bow law.  
**CORRECT BEHAVIOR:** once the blade reaches the minimum-diameter state, additional resistance uses \(k_c\), not only the power law.  
**COMMON MISTAKE:** continuing \(k_b\Delta r^p\) indefinitely.  
**HOW TO TEST:** compress a blade beyond \(\Delta r_{b,\max}\); stiffness should jump to the consolidated/contact regime.

**WARNING 5:** treating all blades as permanently active.  
**CORRECT BEHAVIOR:** a blade may lose contact; inactive blades have zero deflection/force.  
**COMMON MISTAKE:** forcing every blade to remain compressed in the horizontal section.  
**HOW TO TEST:** in a gravity-dominated horizontal section, verify loss of contact of the top blade and four active blade forces, as described by the paper.

**WARNING 6:** using the singular frame formula in vertical sections.  
**CORRECT BEHAVIOR:** replace the generic Eq. (11) construction with a continuity-preserving \(\vec e_X,\vec e_Y\)-based choice when \(t_Z^2=1\).  
**COMMON MISTAKE:** direct evaluation leading to division by zero or frame flipping.  
**HOW TO TEST:** run a purely vertical trajectory segment and check frame continuity.

**WARNING 7:** assuming the technical-sheet \(D_{\max}\) after restriction is always the initial \(D_{\max}\).  
**CORRECT BEHAVIOR:** treat post-restriction \(D_{\max}\) as potentially changed; the paper explicitly studies 7.30–7.44 in.  
**COMMON MISTAKE:** calibrating with an incorrect free diameter and then trusting the fitted \(p\).  
**HOW TO TEST:** sensitivity sweep in \(D_{\max}\); verify that \(p\) decreases sharply from 7.35 in to 7.30 in and becomes nearly linear at 7.30 in.

**WARNING 8:** carrying the OCR typo from Eq. (29) into code.  
**CORRECT BEHAVIOR:** the second component of the force vector is \(c_{cbv,i}\), not \(f_{cbv,i}\).  
**COMMON MISTAKE:** implementing
\[
\vec f_{cb,i}=f_{cbr,i}(r)\,{}^\mathrm t(c_{cbu,i},f_{cbv,i},\mu_z).
\]
**HOW TO TEST:** compare Eq. (29) to the coefficient definitions immediately below it; only \(c_{cbv,i}\) is dimensionally and notationally consistent.

---

## AFTER COMPLETING THE EXTRACTION

### ⚠️ VERIFY

**⚠️ VERIFY — Eq. 29, second component of force vector**

Paper as transcribed by OCR:
\[
\vec f_{cb,i}=f_{cbr,i}(r)\,{}^\mathrm t(c_{cbu,i},f_{cbv,i},\mu_z)
\]

Correct form, consistent with the coefficient definitions below Eq. (29):
\[
\vec f_{cb,i}=f_{cbr,i}(r)\,{}^\mathrm t(c_{cbu,i},c_{cbv,i},\mu_z)
\]

where
\[
c_{cbu,i}=\cos(\alpha_{b,i})-\mu_\theta\sin(\alpha_{b,i}),
\qquad
c_{cbv,i}=\sin(\alpha_{b,i})+\mu_\theta\cos(\alpha_{b,i})
\]

The \(f_{cbv,i}\) in the OCR transcription is a typo for \(c_{cbv,i}\). This correction is adopted throughout this reference.

**⚠️ VERIFY — Eq. (35) completeness**  
Only the visible part of Eq. (35) was available in the provided material. The derivative definitions \(f'_{cr}(x)\) and \(h'(x)\), plus the visible \(\partial m_{cz}/\partial u\) and \(\partial m_{cz}/\partial v\), were transcribed exactly; other terms of Eq. (35) should be checked against the original paper PDF before code freeze.

### ❓ NOT SPECIFIED IN PAPER / NOT RECOVERABLE FROM AVAILABLE EXTRACTS

- Full Table 1 numerical contents were not visible in the accessible extracts.  
- An explicit Newton convergence tolerance was not visible in the accessible extracts.  
- A numerical value of \(\lambda\) was not visible in the accessible extracts; only its role and the condition \(h(\lambda)=1/2\) were visible.

### DEPENDENCIES

**DEPENDENCY:** Eq. (3) requires Eq. (2), because centralizer radial displacement is defined only after clearance is computed.

**DEPENDENCY:** Eq. (5) requires Eq. (3), because blade deflection is based on the effective centralizer displacement, not directly on raw pipe displacement.

**DEPENDENCY:** Eq. (4) requires Eq. (5), because blade force uses blade deflection as input.

**DEPENDENCY:** Eq. (6), Eq. (7), and Eq. (8) require Eq. (1), because both axial drag and tangential friction/torque depend on the assembled blade-force state.

**DEPENDENCY:** Eq. (17) requires Eq. (16), because it is the smoothed replacement of the ideal unilateral contact law.

**DEPENDENCY:** Eq. (18) requires Eq. (19) and Eq. (20), because \(f_{c\theta}\) and \(f_{cz}\) depend on directional friction coefficients derived from the total friction law.

**DEPENDENCY:** Eq. (21) requires Eq. (17)–(20), because local body-contact force and moment components use the smoothed normal force and projected friction coefficients.

**DEPENDENCY:** Eq. (24) requires Eq. (22)–(23), because bow-contact force depends on blade position, blade penetration, and consolidated-regime penetration.

**DEPENDENCY:** Eq. (29) requires Eq. (24), Eq. (27)–(28), and the coefficient definitions, because the FE local bow force/moment vector is built from the already computed radial force magnitude and friction-orientation coefficients.

**DEPENDENCY:** Eq. (36) requires Eq. (24) and Eq. (29), because the bow-contact Jacobian is the derivative of the smoothed two-regime bow law and its mapped local force/moment components.

---

This reference is implementation-ready for the visible equations and data. The only items that should still be checked against the original PDF before final code hardening are the complete Eq. (35) body-contact Jacobian terms and the full Table 1 values.
