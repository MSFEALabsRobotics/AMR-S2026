<img width="467" height="304" alt="image" src="https://github.com/user-attachments/assets/46d7bd2d-2f23-4c15-ae39-d298059469d9" />

# Problem

```
Exercise: 1D Kalman Filter (Robotics) + Plot 3 Gaussians
Scenario

A robot moves along a straight line (1D).
You estimate its position 
𝑥
x using:

Odometry (motion model) with process noise 
𝑄
Q

A position sensor (e.g., GPS/UWB/vision landmark) with measurement noise 
𝑅
R

At each time step:

Predict belief using motion

Correct belief using measurement

Plot prediction Gaussian, measurement Gaussian, and posterior Gaussian
```

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
