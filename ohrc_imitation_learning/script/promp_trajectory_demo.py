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


promps = []
for mode in range(n_case):

    n_demos = 100
    T = [np.array(trajs[mode][i]["t"].values) for i in range(100)]
    px = np.array([trajs[mode][i]["px"].values for i in range(100)])
    py = [trajs[mode][i]["py"].values for i in range(100)]
    pz = [trajs[mode][i]["pz"].values for i in range(100)]

    Y = [np.array([[px[i][j], py[i][j], pz[i][j]]
                   for j in range(px[i].size)]) for i in range(n_demos)]

    dy = trajs[mode][0]["vx"]
    ddy = trajs[mode][0]["ax"]

    # T, Y = generate_1d_trajectory_distribution(n_demos, n_steps)
    y_conditional_cov = np.array([0.0000025, 0.0000025, 0.0000025])
    promp = ProMP(n_dims=3, n_weights_per_dim=10)
    promp.imitate(T, Y)
    promps.append(promp)


y_cond = [0.1, 0.2, 0.3]
Y_cmeans = []
dY_cmeans = []

t_max = 5.0
time = np.arange(0.0, t_max, 0.01)
for promp in promps:
    cpromp = promp.condition_position(
        np.array(y_cond), y_cov=y_conditional_cov, t=0.0, t_max=t_max)

    Y_cmean = cpromp.mean_trajectory(time)
    dY_cmean = cpromp.mean_velocities(time)
    Y_cmeans.append(Y_cmean)
    dY_cmeans.append(dY_cmean)

plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble'] = [r'\usepackage{sansmath}', r'\sansmath']
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = 'Helvetica'
plt.rcParams["font.size"] = 18


test = [2, 4, 3]
plt.figure()
ax = plt.subplot(311)
for i in test:
    ax.plot(time, Y_cmeans[i][:, 0], label="ProMP")
ax.grid(True)
ax.set_ylabel("x")
ax.set_xticklabels([])


ax = plt.subplot(312)
for i in test:
    ax.plot(time, Y_cmeans[i][:, 1], label="ProMP")
ax.grid(True)
ax.set_ylabel("y")
ax.set_xticklabels([])

ax = plt.subplot(313)
for i in test:
    ax.plot(time, Y_cmeans[i][:, 2], label="ProMP")
ax.grid(True)
ax.set_ylabel("z")
ax.set_xlabel("time [s]")

plt.savefig("promp.pdf", bbox_inches='tight')

plt.figure()
ax = plt.axes(projection='3d')
for i in test:
    ax.plot3D(Y_cmeans[i][:, 0], Y_cmeans[i][:, 1],
              Y_cmeans[i][:, 2], label="ProMP")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
# ax.legend(["c1", "c2", "c3"])
ax.legend(["model-1", "model-2", "model-3"],
          loc="upper center", ncol=3)
ax.axis('equal')
plt.savefig("promp_3d.pdf", bbox_inches='tight')

plt.show()
