"""
=================
Conditional ProMP
=================

Probabilistic Movement Primitives (ProMPs) define distributions over
trajectories that can be conditioned on viapoints. In this example, we
plot the resulting posterior distribution after conditioning on varying
start positions.
"""
import os
import pandas as pd
from movement_primitives.promp import ProMP
from movement_primitives.dmp import DMP
from movement_primitives.data import generate_1d_trajectory_distribution
import matplotlib.pyplot as plt
import numpy as np
print(__doc__)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "trajectory")
n_case = sum(os.path.isdir(os.path.join(DATA_DIR, name))
             for name in os.listdir(DATA_DIR))
print("load trajectories in directories of case_0 to case_"+str(n_case))

trajs = [[pd.read_csv(DATA_DIR+"/case_" +
                      str(k)+"/"+str(i+1) + ".csv") for i in range(100)] for k in range(n_case)]


mode = 3
T = [trajs[mode][i]["t"] for i in range(100)]
Y = [trajs[mode][i]["px"] for i in range(100)]
dy = trajs[mode][0]["vx"]
ddy = trajs[mode][0]["ax"]

n_demos = 100

# T, Y = generate_1d_trajectory_distribution(n_demos, n_steps)
y_conditional_cov = np.array([0.000025])
promp = ProMP(n_dims=1, n_weights_per_dim=10)
promp.imitate(T, Y)
Y_mean = promp.mean_trajectory(T[0])
Y_conf = 1.96 * np.sqrt(promp.var_trajectory(T[0]))

plt.figure(figsize=(10, 5))

ax1 = plt.subplot(121)
ax1.set_title("Training set and ProMP")
ax1.fill_between(T[0], (Y_mean - Y_conf).ravel(),
                 (Y_mean + Y_conf).ravel(), color="r", alpha=0.3)
ax1.plot(T[0], Y_mean, c="r", lw=2, label="ProMP")
# ax1.plot(T[:, 0], Y[:, :, 0].T, c="k", alpha=0.1)
# ax1.set_xlim((-0.05, 1.05))
# ax1.set_ylim((-2.5, 3))
ax1.legend(loc="best")

ax2 = plt.subplot(122)
ax2.set_title("Conditional ProMPs")

for color, y_cond in zip("rgbcmyk", np.linspace(-0.3, 0.3, 7)):
    cpromp = promp.condition_position(
        np.array([y_cond]), y_cov=y_conditional_cov, t=0.0, t_max=1.0)
    Y_cmean = cpromp.mean_trajectory(T[0])
    Y_cconf = 1.96 * np.sqrt(cpromp.var_trajectory(T[0]))

    ax2.scatter([0], [y_cond], marker="*", s=100,
                c=color, label="$y_0 = %.2f$" % y_cond)
    ax2.fill_between(T[0], (Y_cmean - Y_cconf).ravel(),
                     (Y_cmean + Y_cconf).ravel(), color=color, alpha=0.3)
    ax2.plot(T[0], Y_cmean, c=color, lw=2)
    ax2.set_xlim((-0.05, 1.05))
    # ax2.set_ylim((-2.5, 3))
    ax2.legend(loc="best")

plt.tight_layout()
plt.show()
