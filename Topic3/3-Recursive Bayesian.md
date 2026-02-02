# Recursive Bayesian Estimation Example (Door Open/Closed)

<img width="571" height="436" alt="image" src="https://github.com/user-attachments/assets/90fe58d4-4623-4ec7-b71b-ea15dc3527d8" />

---

<img width="373" height="188" alt="image" src="https://github.com/user-attachments/assets/097f09c4-d2fa-4eaf-bf95-feea9d895bb1" />

---

<img width="383" height="284" alt="image" src="https://github.com/user-attachments/assets/e2382001-107a-461c-8bde-a02d5de5d760" />

---

<img width="388" height="124" alt="image" src="https://github.com/user-attachments/assets/0668acaf-0f6b-4121-8fdd-9ead21801690" />

---

This script demonstrates a **recursive Bayesian state estimation** example, where a robot tries to estimate whether a door is **open** or **closed** based on noisy sensor measurements.

---

## Code Explanation (Line by Line)

```python
import numpy as np
import matplotlib.pyplot as plt
```
- Import **NumPy** for random numbers & arrays, and **Matplotlib** for plotting.

```python
rng = np.random.default_rng(7)
print(rng)
```
- Create a random number generator with seed `7` (results reproducible).
- Printing shows the generator object.

```python
T = 30
alpha = 0.1
p_z_open_given_open = 0.8
p_z_open_given_closed = 0.2
```
- `T = 30`: number of time steps.
- `alpha = 0.1`: probability the **door flips state**.
- `p_z_open_given_open = 0.8`: sensor says “open” correctly 80% of the time.
- `p_z_open_given_closed = 0.2`: false alarm, sensor says “open” 20% of the time when door is closed.

```python
x_true = np.zeros(T, dtype=int)
z_obs = np.zeros(T, dtype=int)  # 1=open reading
x_true[0] = 1  # start open
```
- `x_true`: true door state (1=open, 0=closed).
- `z_obs`: sensor observations (1=open reading).
- Start with door **open**.

```python
for t in range(1, T):
    if rng.random() < alpha:      # with probability alpha
        x_true[t] = 1 - x_true[t-1]  # flip state
    else:
        x_true[t] = x_true[t-1]      # keep same state
```
- Simulates the true door state:
  - With 10% chance, the door flips.
  - Otherwise, it stays the same.

```python
for t in range(T):
    if x_true[t] == 1:
        z_obs[t] = rng.random() < p_z_open_given_open
    else:
        z_obs[t] = rng.random() < p_z_open_given_closed
```
- Simulates **sensor readings**:
  - If door is open → sensor says open 80% of the time.
  - If door is closed → sensor says open 20% of the time.

```python
bel = np.zeros(T)  # belief P(open)
bel[0] = 0.5
```
- `bel`: robot’s belief (probability door is open).
- Start with **50% uncertainty**.

```python
for t in range(1, T):
    # Predict
    prior_open = (1-alpha)*bel[t-1] + alpha*(1-bel[t-1])
```
- **Prediction step**: compute prior belief before sensor update.
- With probability `1-alpha`, door stays in same state.
- With probability `alpha`, door flips.

```python
    # Update
    if z_obs[t] == 1:
        num = p_z_open_given_open * prior_open
        den = p_z_open_given_open*prior_open + p_z_open_given_closed*(1-prior_open)
    else:
        p_z_closed_given_open = 1 - p_z_open_given_open
        p_z_closed_given_closed = 1 - p_z_open_given_closed
        num = p_z_closed_given_open * prior_open
        den = p_z_closed_given_open*prior_open + p_z_closed_given_closed*(1-prior_open)
    bel[t] = num / den
```
- **Update step** (Bayes rule):
  - If sensor says **open**, update using likelihoods.
  - If sensor says **closed**, use complementary probabilities.
  - Normalize with denominator.

```python
fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
```
- Create 3 stacked plots.

```python
axs[0].step(range(T), x_true, where="mid")
axs[0].set_ylabel("True state\n(1=open,0=closed)")
axs[0].set_ylim(-0.2, 1.2)
```
- Plot **true state**.

```python
axs[1].step(range(T), z_obs, where="mid", color="orange")
axs[1].set_ylabel("Observation\n(1=open,0=closed)")
axs[1].set_ylim(-0.2, 1.2)
```
- Plot **sensor readings**.

```python
axs[2].plot(range(T), bel, marker="o")
axs[2].set_ylabel("Belief P(open)")
axs[2].set_xlabel("Time step")
axs[2].set_ylim(-0.05, 1.05)
```
- Plot **belief evolution**.

```python
plt.tight_layout()
plt.show()
```
- Display plots.

---


