# Exercise 2 – Normal Distribution — Sampling and Probability Density Function

## Problem Statement
Draw samples from a Gaussian distribution and compare the histogram with the theoretical probability density function (PDF).

## Explanation
1. Generate random samples from a normal distribution with mean 0 and standard deviation 1.
2. Plot a histogram of the samples (empirical distribution).
3. Overlay the theoretical Gaussian curve.
4. Observe that as the number of samples grows, the histogram matches the theory.

- **Why:** This illustrates how sampling from a distribution approximates its PDF.


<img width="478" height="82" alt="image" src="https://github.com/user-attachments/assets/705581f1-2fca-4959-8197-5369d81a1f44" />


## Python Code
```python
import numpy as np
import matplotlib.pyplot as plt

# STEP 1: generate random numbers from a normal distribution
x = np.random.normal(0, 1, 5000)
#density=True, data normalized, Area of histograms is 1
#alpha = 0.6  transparency

# STEP 2: histogram of the samples
plt.hist(x, bins=40, density=True, alpha=0.6, label="Histogram (samples)")


# STEP 3: theoretical PDF
xs = np.linspace(-4, 4, 400) #linspace Creates an array of evenly spaced numbers
pdf = (1/np.sqrt(2*np.pi)) * np.exp(-xs**2 / 2)
plt.plot(xs, pdf, 'r', label="PDF (theory)")

# STEP 4: labels and show
plt.title("Normal Distribution (Beginner Version)")
plt.xlabel("x")
plt.ylabel("Density")
plt.legend()
plt.show()
```

## Reflection
- The histogram approximates the bell curve of the normal distribution.
- With larger sample size, the approximation improves.
