# Kalman Filter Exercise — 1D Position Tracking with a Constant-Velocity Model

This exercise simulates a 1D robot moving along a line and tracks its **position** using a **Kalman Filter (KF)** whose state is:

\[
\mathbf{x}_t =
\begin{bmatrix}
x_t \\
v_t
\end{bmatrix}
\]

Where:
- \(x_t\): position (m)  
- \(v_t\): velocity (m/s)

You will:
1) generate noisy position measurements,  
2) run a KF with state \([x, v]\),  
3) visualize the **prediction (prior)**, **measurement likelihood**, and **correction (posterior)** as Gaussians.

---

## 1) Simulation Setup (Ground Truth + Measurements)

### True motion (unknown acceleration disturbance)
The “true” robot is *approximately* constant-velocity, but with random acceleration disturbances:

- Acceleration disturbance:
\[
a_t \sim \mathcal{N}(0,\sigma_a^2)
\]

- Velocity update:
\[
v_t = v_{t-1} + a_t\,\Delta t
\]

- Position update (discrete-time kinematics):
\[
x_t = x_{t-1} + v_{t-1}\,\Delta t + \tfrac{1}{2} a_t\,\Delta t^2
\]

In the code:
- `accel_std = 0.15` sets \(\sigma_a\).  
- `dt = 1.0` is \(\Delta t\).

### Measurement model (position only)
The sensor measures position only:

\[
z_t = x_t + n_t, \quad n_t \sim \mathcal{N}(0, R)
\]

In the code:
- `R = 0.7**2` means \(\sigma_z = 0.7\) meters.

---

## 2) Kalman Filter Model (What the KF Assumes)

The KF **does not** model acceleration explicitly. It assumes a constant-velocity model:

### State transition
\[
\mathbf{x}_t = A\mathbf{x}_{t-1} + \mathbf{w}_t
\]

with
\[
A =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
\]

and process noise
\[
\mathbf{w}_t \sim \mathcal{N}(\mathbf{0}, Q)
\]

In the code, a simple (teaching-friendly) choice is used:
\[
Q =
\begin{bmatrix}
\sigma_x^2 & 0 \\
0 & \sigma_v^2
\end{bmatrix}
\]
where `sigma_x = 0.20`, `sigma_v = 0.25`.

> Note: A more “physics-based” constant-acceleration noise model can build a correlated Q, but the diagonal Q is enough to demonstrate KF behavior.

### Measurement matrix
\[
z_t = H\mathbf{x}_t + n_t, \quad n_t \sim \mathcal{N}(0, R)
\]

with
\[
H = \begin{bmatrix}1 & 0\end{bmatrix}
\]

Because we measure position only.

---

## 3) KF Algorithm (Predict → Update)

The belief is Gaussian:
\[
bel(\mathbf{x}_t) = \mathcal{N}(\mu_t, P_t)
\]

### A) Predict (prior)
\[
\mu_t^- = A\mu_{t-1}
\]
\[
P_t^- = A P_{t-1} A^T + Q
\]

### B) Update (posterior)
Innovation (measurement residual):
\[
\mathbf{y}_t = z_t - H\mu_t^-
\]

Innovation covariance:
\[
S_t = H P_t^- H^T + R
\]

Kalman gain:
\[
K_t = P_t^- H^T S_t^{-1}
\]

Posterior mean and covariance:
\[
\mu_t = \mu_t^- + K_t\,\mathbf{y}_t
\]
\[
P_t = (I - K_t H)P_t^-
\]

---

## 4) Why the Code Plots “3 Gaussians” at One Time Step

At a chosen time step `t_star`, the script plots **1D PDFs over position x**, even though the KF state is 2D \([x, v]\).

### 1) Prediction marginal over position
The KF prior is Gaussian in \([x,v]\). Its **position marginal** is:

\[
x_t \sim \mathcal{N}(\mu_{x,t}^-,\; P_{xx,t}^-)
\]

In code:
- `mu_x_pred = mu_pred[0]`
- `var_x_pred = P_pred[0,0]`

### 2) Measurement likelihood over x
The measurement distribution is centered at the measured value \(z_t\):

\[
p(z_t \mid x) \propto \mathcal{N}(x;\; z_t,\; R)
\]

In code:
- `mu_x_meas = z_t`
- `var_x_meas = R`

### 3) Posterior marginal over position
After update, the position marginal is:

\[
x_t \sim \mathcal{N}(\mu_{x,t},\; P_{xx,t})
\]

In code:
- `mu_x_post = mu_post[0]`
- `var_x_post = P_post[0,0]`

So the plot shows:
- **Prediction (prior)**: what you believe before seeing \(z_t\)  
- **Measurement likelihood**: what the sensor says about \(x\)  
- **Correction (posterior)**: combination of both (KF result)

---

## 5) Interpreting the Results

- If **R is large** (noisy sensor), the KF trusts the model more → posterior stays closer to prediction.
- If **Q is large** (model is uncertain), the KF trusts measurements more → posterior moves toward measurement.
- The posterior variance \(P\) usually **shrinks after update** (you learn from measurement).

---

## 6) Tasks / Questions (Exercise)

### A) Read and explain
1. What does `H = [[1, 0]]` mean physically?
2. Why is the KF state 2D \([x, v]\) but the measurement is 1D \(z\)?
3. In your own words: what do **predict** and **update** do?

### B) Parameter experiments (run and observe)
Change **one** parameter at a time and rerun:

1. Increase measurement noise:
   - Set `R = 1.5**2`
   - What happens to the Kalman gain and the estimate curve?

2. Increase process noise:
   - Set `Q = diag([0.6**2, 0.7**2])`
   - Does the estimate follow measurements more closely? Why?

3. Change initialization:
   - Try `mu = [5, 2]` and `P = diag([5**2, 5**2])`
   - How many steps until it “recovers”?

### C) Explain the “3 Gaussians” plot
At `t_star`:
1. Which curve is narrowest and why?
2. Why is the posterior typically **between** the prediction and the measurement?

### D) (Optional) Improve the model Q (acceleration-based)
Replace diagonal Q with the constant-acceleration discrete noise model:

\[
Q = \sigma_a^2
\begin{bmatrix}
\tfrac{\Delta t^4}{4} & \tfrac{\Delta t^3}{2} \\
\tfrac{\Delta t^3}{2} & \Delta t^2
\end{bmatrix}
\]

Try `sigma_a = 0.15` and compare.

---

## 7) What to Submit
- A short write-up (5–10 lines) answering tasks A + C
- One screenshot of the final trajectory plot (True vs Measurements vs KF)
- One screenshot of the “3 Gaussians” plot at `t_star`
- A brief note on one parameter experiment from task B

---

### Note on the helper `gaussian_pdf`
The function:

\[
\mathcal{N}(x;\mu,\sigma^2) = \frac{1}{\sqrt{2\pi\sigma^2}}\exp\left(-\frac{1}{2}\frac{(x-\mu)^2}{\sigma^2}\right)
\]

is used only to draw the PDFs of the three distributions on the x-axis.
