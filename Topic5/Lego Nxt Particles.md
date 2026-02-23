<img width="389" height="129" alt="image" src="https://github.com/user-attachments/assets/97ee4879-2898-417d-b756-fc865932549f" />


<img width="906" height="391" alt="image" src="https://github.com/user-attachments/assets/1cd1a939-6f39-4071-a981-ea24691df4ed" />


###

<img width="554" height="400" alt="image" src="https://github.com/user-attachments/assets/fbbb45ea-3853-4578-83c0-7b9a8d73d0fd" />

```python

def h(x):

    """
    x in mm
    h(x) in tiles (one tile is 40cm)

    """
    if 0 <= x < 200:
        return 1.5
    elif 200 <= x < 300:
        return 3.0
    elif 300 <= x <= 400:
        return 2.0
    elif 400 <= x <= 500:
        return 1.5
    else:
        raise ValueError("x is out of range [0, 400]")

```


<img width="448" height="341" alt="image" src="https://github.com/user-attachments/assets/548d6af9-c50b-4b93-b2ff-d59dad2f2f44" />

# Plotting

```python

import numpy as np
import matplotlib.pyplot as plt

def h(x):
    if 0 <= x < 200:
        return 1.5
    elif 200 <= x < 300:
        return 3.0
    elif 300 <= x <= 400:
        return 2.0
    else:
        return np.nan  # out of range

# sample x values
xs = np.linspace(0, 400, 1000)
ys = [h(x) for x in xs]

plt.plot(xs, ys)
plt.xlabel("x")
plt.ylabel("h(x)")
plt.title("Piecewise measurement map h(x)")
plt.grid(True)
plt.show()

```

# Particle Filter Representation
## Use M particles (e.g., 500–2000). Each particle x_t^[m] is a hypothesis of position.
## Maintain weights w_t^[m] that sum to 1.
## PF steps each time t:
## Predict (Motion update): sample a new position for each particle using the control u_t and process noise.
## Correct (Measurement update): evaluate a likelihood for z_t given each particle’s tile; multiply weights and normalize.
## Resample: apply low‑variance resampling (systematic resampling) to focus particles on likely regions.
## Estimate: compute an estimate of position (e.g., weighted mean) and spread (e.g., weighted variance).
## Plot: plot in real time
