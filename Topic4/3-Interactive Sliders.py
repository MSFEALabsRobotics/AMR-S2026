import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons, Button

def gaussian_pdf(x, mu, var):
    var = max(var, 1e-12)
    return (1.0 / np.sqrt(2*np.pi*var)) * np.exp(-0.5 * (x - mu)**2 / var)

# -----------------------
# Fixed simulation (truth + measurements)
# -----------------------
np.random.seed(1)
T = 25
dt = 1.0

x_true = np.zeros(T)
v_true = np.zeros(T)
v_true[0] = 1.0

accel_std = 0.15
for t in range(1, T):
    a = np.random.normal(0, accel_std)
    v_true[t] = v_true[t-1] + a*dt
    x_true[t] = x_true[t-1] + v_true[t-1]*dt + 0.5*a*dt*dt

# Measurement noise (we will scale R via slider, but keep a base noise realization)
base_meas_std = 0.7
meas_noise_unit = np.random.normal(0, 1.0, size=T)  # fixed noise draw

# KF constant matrices
A = np.array([[1, dt],
              [0, 1 ]], dtype=float)
H = np.array([[1, 0]], dtype=float)

# Initial belief
mu0 = np.array([0.0, 0.0])
P0  = np.diag([1.5**2, 1.0**2])

x_axis = np.linspace(-5, 35, 900)

def run_kf(Qx, Qv, R, t_star):
    """
    Runs KF for all timesteps and returns:
    - z (measurements)
    - mu_hist, P_hist
    - at t_star: mu_pred, P_pred, mu_post, P_post
    """
    # Build measurement sequence with chosen R, using fixed noise draw
    meas_std = np.sqrt(max(R, 1e-12))
    z = x_true + meas_noise_unit * meas_std

    Q = np.array([[Qx, 0],
                  [0,  Qv]], dtype=float)

    mu = mu0.copy()
    P  = P0.copy()

    mu_hist = []
    P_hist  = []

    snap = None

    for t in range(T):
        # Predict
        mu_pred = A @ mu
        P_pred  = A @ P @ A.T + Q

        # Update
        z_t = z[t]
        S = H @ P_pred @ H.T + R
        K = (P_pred @ H.T) / S
        y = z_t - (H @ mu_pred)
        mu_post = mu_pred + (K.flatten() * float(y))
        P_post  = (np.eye(2) - K @ H) @ P_pred

        mu, P = mu_post, P_post
        mu_hist.append(mu.copy())
        P_hist.append(P.copy())

        if t == t_star:
            snap = (z_t, mu_pred.copy(), P_pred.copy(), mu_post.copy(), P_post.copy())

    return z, np.array(mu_hist), np.array(P_hist), snap

# -----------------------
# Figure layout
# -----------------------
fig = plt.figure(figsize=(11, 6))
ax_pdf = fig.add_axes([0.08, 0.35, 0.55, 0.6])
ax_traj = fig.add_axes([0.70, 0.35, 0.27, 0.6])

# Sliders axes
ax_R  = fig.add_axes([0.10, 0.23, 0.50, 0.03])
ax_Qx = fig.add_axes([0.10, 0.18, 0.50, 0.03])
ax_Qv = fig.add_axes([0.10, 0.13, 0.50, 0.03])
ax_t  = fig.add_axes([0.10, 0.08, 0.50, 0.03])

# Checkbox + reset
ax_chk = fig.add_axes([0.70, 0.18, 0.27, 0.12])
ax_rst = fig.add_axes([0.70, 0.08, 0.12, 0.06])

# Create widgets
sR  = Slider(ax_R,  "R (meas var)", 0.05**2, 2.0**2, valinit=0.7**2)
sQx = Slider(ax_Qx, "Qx (proc var)", 0.01**2, 1.0**2, valinit=0.20**2)
sQv = Slider(ax_Qv, "Qv (proc var)", 0.01**2, 1.5**2, valinit=0.25**2)
st  = Slider(ax_t,  "t* (step)", 0, T-1, valinit=10, valstep=1)

chk = CheckButtons(ax_chk, ["Show measurement pdf"], [True])
btn = Button(ax_rst, "Reset")

# Initial draw
line_pred, = ax_pdf.plot([], [], label="Prediction (prior on x)")
line_meas, = ax_pdf.plot([], [], label="Measurement likelihood p(z|x)")
line_post, = ax_pdf.plot([], [], label="Correction (posterior on x)")
v_true_line = ax_pdf.axvline(0, linestyle="--", label="True x (ref)")

traj_true, = ax_traj.plot([], [], label="True x")
traj_meas, = ax_traj.plot([], [], "o", alpha=0.35, label="z")
traj_est,  = ax_traj.plot([], [], label="KF x̂")

ax_pdf.set_xlabel("x (m)")
ax_pdf.set_ylabel("pdf")
ax_pdf.legend(loc="upper right")
ax_pdf.set_title("3 Gaussians on x at timestep t*")

ax_traj.set_xlabel("t")
ax_traj.set_ylabel("x (m)")
ax_traj.legend(loc="upper left")
ax_traj.set_title("Trajectory")

ax_traj.set_xlim(0, T-1)

show_meas = True

def update(_=None):
    global show_meas

    R  = sR.val
    Qx = sQx.val
    Qv = sQv.val
    t_star = int(st.val)

    z, mu_hist, P_hist, snap = run_kf(Qx, Qv, R, t_star)
    z_t, mu_pred, P_pred, mu_post, P_post = snap

    mu_x_pred, var_x_pred = mu_pred[0], P_pred[0, 0]
    mu_x_post, var_x_post = mu_post[0], P_post[0, 0]

    pred_pdf = gaussian_pdf(x_axis, mu_x_pred, var_x_pred)
    post_pdf = gaussian_pdf(x_axis, mu_x_post, var_x_post)

    line_pred.set_data(x_axis, pred_pdf)
    line_post.set_data(x_axis, post_pdf)

    if show_meas:
        meas_pdf = gaussian_pdf(x_axis, z_t, R)
        line_meas.set_data(x_axis, meas_pdf)
        line_meas.set_visible(True)
    else:
        line_meas.set_visible(False)

    v_true_line.set_xdata([x_true[t_star]])

    # Autoscale pdf y-limits
    ymax = max(pred_pdf.max(), post_pdf.max(), (gaussian_pdf(x_axis, z_t, R).max() if show_meas else 0))
    ax_pdf.set_xlim(x_axis.min(), x_axis.max())
    ax_pdf.set_ylim(0, ymax * 1.15 + 1e-9)

    # Trajectory plot
    t_arr = np.arange(T)
    traj_true.set_data(t_arr, x_true)
    traj_meas.set_data(t_arr, z)
    traj_est.set_data(t_arr, mu_hist[:, 0])

    ax_traj.set_ylim(min(x_true.min(), z.min(), mu_hist[:,0].min()) - 2,
                     max(x_true.max(), z.max(), mu_hist[:,0].max()) + 2)

    ax_pdf.set_title(
        f"t*={t_star} | z={z_t:.2f} | "
        f"pred: μ={mu_x_pred:.2f}, σ={np.sqrt(var_x_pred):.2f}  "
        f"post: μ={mu_x_post:.2f}, σ={np.sqrt(var_x_post):.2f}"
    )

    fig.canvas.draw_idle()

def on_check(label):
    global show_meas
    show_meas = not show_meas
    update()

def on_reset(event):
    sR.reset()
    sQx.reset()
    sQv.reset()
    st.reset()

chk.on_clicked(on_check)
btn.on_clicked(on_reset)

sR.on_changed(update)
sQx.on_changed(update)
sQv.on_changed(update)
st.on_changed(update)

update()
plt.show()
