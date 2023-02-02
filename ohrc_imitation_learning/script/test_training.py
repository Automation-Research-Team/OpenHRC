from dummy_trajectory_generation import *
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


# SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# DATA_DIR = os.path.join(SCRIPT_DIR, "trajectory")
# n_case = sum(os.path.isdir(os.path.join(DATA_DIR, name))
#              for name in os.listdir(DATA_DIR))
# print("load trajectories in directories of case_0 to case_"+str(n_case))

# trajs = [[pd.read_csv(DATA_DIR+"/case_" +
#                       str(k)+"/"+str(i+1) + ".csv") for i in range(100)] for k in range(n_case)]

# trajs = []
# n_case = 5
# fig2 = plt.figure()
# ax_ = [fig2.add_subplot(1, n_case, i+1, projection="3d")
#        for i in range(n_case)]
# for k in range(n_case):
#     fig = plt.figure()
#     plt.title("case_" + str(k))
#     ax = [fig.add_subplot(3, 3, i+1) for i in range(9)]

#     trajs.append([pd.read_csv(FILE_DIR+"/trajectory/case_" +
#                               str(k)+"/"+str(i+1) + ".csv") for i in range(100)])
#     for traj in trajs[k]:
#         ax_[k].plot(traj["px"], traj["py"], traj["pz"])

#         for j in range(1, len(traj.keys())):
#             ax[j-1].plot(traj["t"], traj[traj.keys()[j]])
#             ax[j-1].set_xlabel("t")
#             ax[j-1].set_ylabel(traj.keys()[j])

#     ax_[k].set_xlabel("px")
#     ax_[k].set_ylabel("py")
#     ax_[k].set_zlabel("pz")


N = 10


# t = trajs[0][0]["t"]
# y = trajs[0][0]["px"]
# dy = trajs[0][0]["vx"]
# ddy = trajs[0][0]["ax"]
# N_t = len(y)


traj = getMinJerkTraj(0.1, 1.0, 0.0, 1.0)
t = np.array([x[0] for x in traj])
y = np.array([x[1] for x in traj])
dy = np.array([x[2] for x in traj])
ddy = np.array([x[3] for x in traj])
N_t = len(y)

fig = plt.figure()
plt.plot(t, y)


tau = 2.0

alpha_x = 8.0
beta_z = 0.4
alpha_z = 4.0*beta_z

x = np.exp(-alpha_x/tau*t)
fig = plt.figure()
plt.plot(t, x)

c = [np.exp(- alpha_x * (i)/(N-1)) for i in range(N)]
h = [1./((c[i+1]-c[i])**2) for i in range(N-1)]
h.append(h[-1])


def phi(i, xi):
    return np.exp(-h[i]*(xi - c[i])**2)


# for
Phi = [[phi(i, x[k])/sum([phi(j, x[k]) for j in range(N)]) * x[k]
        for i in range(N)] for k in range(N_t)]

Phi_ = np.array(Phi)
phi_ = [Phi_[j, :].T for j in range(N_t)]

sum_phi = []
for k in range(N_t):
    aa = [phi(j, x[k]) for j in range(N)]
    sum_phi.append(sum(aa))


fd = [tau**2*ddy[i] - alpha_z*(beta_z*(y[-1]-y[i]) - tau*dy[i])
      for i in range(N_t)]

lam = 0.8
P = np.eye(N)
w = np.zeros(N)
for i in range(N_t):
    P = 1.0/lam * \
        (P - (P * phi_[i]@phi_[i].T@P) /
         (lam + phi_[i].T@P @ phi_[i]))
    w = w+(fd[i] - phi_[i].T@w)*P@phi_[i]

print(w)
# # w = w + (fd[])


# s =

plt.show()
