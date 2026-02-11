# Kalman Filter (1D Position Tracking)

In this exercise, we implement a simple **Kalman Filter (KF)** to estimate the 1D position of a robot moving in a straight line.  

⚠️ **Note on visualization:**  
Although the system is **1D** (state = position along a line), we show the results in a **2D time-series plot**:  
- **x-axis** = time steps  
- **y-axis** = position values (true, noisy, predicted, estimated)  

This makes it easier to compare how the filter evolves over time.

---

## 📘 Step 1: Formulas

We assume the state is **position** $x_t$.  
Measurements are noisy observations $z_t$.  

### 📘 Prediction Step

$$
x_t^- = x_{t-1}
$$

$$
P_t^- = P_{t-1} + Q
$$

---

### 📘 Update (Correction) Step

**Kalman Gain:**

$$
K_t = \frac{P_t^-}{P_t^- + R}
$$

**State Update:**

$$
x_t = x_t^- + K_t (z_t - x_t^-)
$$

**Covariance Update:**

$$
P_t = (1 - K_t) P_t^-
$$

---

## 📝 Step 2: Generate Synthetic Data

We simulate a robot moving from 0 → 10 in 50 steps.  
Measurements are noisy with Gaussian noise.

```python
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(0)
T = 50
true_pos = np.linspace(0, 10, T)
measurements = true_pos + np.random.normal(0, 0.5, T)

print("First 5 true positions:", true_pos[:5])
print("First 5 noisy measurements:", measurements[:5])

plt.plot(true_pos, label="True Position")
plt.plot(measurements, "o", alpha=0.5, label="Measurements")
plt.legend()
plt.title("True vs Noisy Measurements (1D state shown as 2D plot)")
plt.show()
```

---

## 📝 Step 3: Initialize the Filter

We define arrays for the estimated state `x_est`, predicted state `x_pred_all`, and their uncertainty `P`.  
We also set noise values `Q` (process) and `R` (measurement).

```python
x_est = np.zeros(T)
x_pred_all = np.zeros(T)
P = np.zeros(T)
x_est[0] = 0      # initial guess
P[0] = 1          # initial uncertainty
Q = 0.01          # process noise
R = 0.25          # measurement noise

print("Initial guess x0 =", x_est[0])
print("Initial uncertainty P0 =", P[0])
```

---

## 📝 Step 4: Recursive Kalman Loop

We apply **predict** then **update** at each time step.  
We also store the predictions before the update for visualization.

```python
for t in range(1, T):
    # Predict
    x_pred = x_est[t-1]
    P_pred = P[t-1] + Q

    # Store prediction
    x_pred_all[t] = x_pred

    # Update
    K = P_pred / (P_pred + R)
    x_est[t] = x_pred + K * (measurements[t] - x_pred)
    P[t] = (1 - K) * P_pred

    # Print first few iterations for clarity
    if t < 5:
        print(f"Step {t}: Prediction = {x_pred:.2f}, Measurement = {measurements[t]:.2f}, Update = {x_est[t]:.2f}, K = {K:.2f}")
```

### 🔎 Example Output for Step 1
```
Step 1: Prediction = 0.00, Measurement = 0.40, Update = 0.32, K = 0.80
```

- **Prediction = 0.00** → The predicted position (from initial guess).  
- **Measurement = 0.40** → The noisy sensor reading at step 1.  
- **Update = 0.32** → The corrected estimate, between prediction and measurement.  
- **K = 0.80** → The Kalman Gain, showing the filter trusts the measurement more (since uncertainty was high initially).  

---

## 📝 Step 5: Plot Results

We compare true position, noisy measurements, **predictions**, and KF **estimates**.

```python
plt.plot(true_pos, label="True Position")
plt.plot(measurements, "o", alpha=0.5, label="Measurements")
plt.plot(x_pred_all, "--", label="Predictions (before update)")
plt.plot(x_est, label="KF Estimate (after update)")
plt.legend()
plt.title("Kalman Filter Result (with Predictions)")
plt.show()
```

---

## 🎯 Exercises

1. Change the measurement noise **R**.  
   - What happens when R is very large? Very small?  

2. Add a **constant velocity model** (instead of just position).  
   - How does the KF change?  

3. Try increasing **process noise Q**.  
   - How does the estimate respond?  

4. Compare the **prediction curve** vs **update curve**.  
   - What do you notice about their relationship to the measurements?  

---
