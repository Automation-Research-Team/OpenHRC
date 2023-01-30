import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os


FILE_DIR = os.path.dirname(os.path.abspath(__file__))

trajs = []
n_case = 5
fig2 = plt.figure()
ax_ = [fig2.add_subplot(1, n_case, i+1, projection="3d")
       for i in range(n_case)]
for k in range(n_case):
    fig = plt.figure()
    plt.title("case_" + str(k))
    ax = [fig.add_subplot(3, 3, i+1) for i in range(9)]

    trajs.append([pd.read_csv(FILE_DIR+"/trajectory/case_" +
                              str(k)+"/"+str(i+1) + ".csv") for i in range(100)])
    for traj in trajs[k]:
        ax_[k].plot(traj["px"], traj["py"], traj["pz"])

        for j in range(1, len(traj.keys())):
            ax[j-1].plot(traj["t"], traj[traj.keys()[j]])
            ax[j-1].set_xlabel("t")
            ax[j-1].set_ylabel(traj.keys()[j])

    ax_[k].set_xlabel("px")
    ax_[k].set_ylabel("py")
    ax_[k].set_zlabel("pz")


N = 10


t = trajs[0][0]["t"]
y = trajs[0][0]["px"]
dy = trajs[0][0]["vx"]
ddy = trajs[0][0]["ax"]
N_t = len(y)

dt = 0.1
tau = 1.0

alpha_x = 0.3
beta_z = 0.1
alpha_z = 4.0*beta_z

x = np.exp(alpha_x/tau*t)

c = [np.exp(- alpha_x * (i-1.0)/(N-1.0)) for i in range(N)]
h = [1./((c[i+1]-c[i])**2) for i in range(N-1)]
h.append(h[-1])


def phi(i, xi):
    return np.exp(-h[i]*(xi - c[i])**2)


print(phi(0, x[0])/sum([phi(j, x[0]) for j in range(N)]))
Phi = [[phi(i, x[k])/sum([phi(j, x[k]) for j in range(N)]) * x[k]
        for i in range(N)] for k in range(N_t)]

Phi_ = np.array(Phi)
phi_ = [Phi_[j, :].T for j in range(N_t)]


fd = [tau**2*ddy[i] - alpha_z*(beta_z*(0.0-y[i]) - tau*dy[i])
      for i in range(N_t)]

lam = 0.1
P = np.eye(N)
w = np.zeros(N)
# for i in range(N_t):
#     P = 1.0/lam * (P - (P * phi_[i]@phi_[i].T@P)/(lam + phi_[i].T@P@phi_[i]))
#     w = w+(fd[i] - phi_[i].T@w)@P@phi_[i]
#     print(i)
# # w = w + (fd[])


s =

plt.show()
