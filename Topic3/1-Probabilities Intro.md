# Exercise 1 – Coin Flip Simulation — Law of Large Numbers

## Problem Statement
Simulate repeated coin flips and show how the estimated probability of heads converges to the true probability (Law of Large Numbers).

## Explanation
1. We generate random numbers to simulate coin flips (1=heads, 0=tails).
2. We compute the running average of heads as the number of flips increases.
3. We plot the running average against the true probability to visualize convergence.

- **Why:** This illustrates how empirical frequencies converge to theoretical probabilities when the number of trials grows.

## Python Code
```python
import numpy as np
import matplotlib.pyplot as plt

# number of flips
N = 2000
p = 0.5   # probability of heads

# STEP 1: make N random numbers between 0 and 1
numbers = np.random.rand(N)

# STEP 2: convert random numbers into coin flips
flips = []
for num in numbers:
    if num < p:
        flips.append(1)
    else:
        flips.append(0)

flips = np.array(flips)

# STEP 3: compute running average 
running_mean = []
total_heads = 0
for i in range(N):
    total_heads += flips[i]
    average = total_heads / (i + 1)
    running_mean.append(average)

running_mean = np.array(running_mean)

# STEP 4: plot the result
plt.plot(running_mean, label="Estimate")
plt.axhline(p, color="red", linestyle="--", label="True probability")
plt.xlabel("Number of flips")
plt.ylabel("Probability of heads")
plt.title("Coin Flip Simulation (Beginner Version)")
plt.legend()
plt.show()
```

## Reflection
- The running average stabilizes around 0.5 as the number of flips grows.
- This demonstrates the Law of Large Numbers in practice.
