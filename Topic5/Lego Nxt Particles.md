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
        
# Particle Filter Representation
## Use M particles (e.g., 500–2000). Each particle x_t^[m] is a hypothesis of position.
## Maintain weights w_t^[m] that sum to 1.
## PF steps each time t:
## Predict (Motion update): sample a new position for each particle using the control u_t and process noise.
## Correct (Measurement update): evaluate a likelihood for z_t given each particle’s tile; multiply weights and normalize.
## Resample: apply low‑variance resampling (systematic resampling) to focus particles on likely regions.
## Estimate: compute an estimate of position (e.g., weighted mean) and spread (e.g., weighted variance).
## Plot: plot in real time
