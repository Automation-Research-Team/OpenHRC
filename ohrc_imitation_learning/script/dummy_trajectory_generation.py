import numpy as np


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ArtistAnimation

import pandas as pd

import os

import cvxpy

FILE_DIR = os.path.dirname(os.path.abspath(__file__))


def getMinJerkTraj_QP(dt, tf, x0, xf):
    t = np.arange(0.0, tf, dt)

    a = np.zeros((3, 3))
    a[0, 1] = 1.0
    a[1, 2] = 1.0

    a_ = np.kron(np.eye(3), a)

    b = np.zeros((3, 1))
    b[2] = 1.0
    b_ = np.kron(np.eye(3), b)

    A = np.eye(9)+dt*a_
    B = dt*b_
    N = int(tf/dt)
    Q = np.zeros((9, 9))
    R = np.eye(3)
    P = np.eye(9)*100.0
    x0_ = x0[["px", "vx", "ax", "py", "vy", "ay",  "pz", "vz", "az"]].values
    xf_ = xf[["px", "vx", "ax", "py", "vy", "ay",  "pz", "vz", "az"]].values

    x, u = use_modeling_tool(A, B, N, Q, R, P, x0_.T, xf_.T)

    # t = t[:, np.newaxis]
    xt = np.concatenate(
        [
            t[:, np.newaxis], x.transpose()
        ], axis=1
    )

    df = pd.DataFrame(xt)
    df.columns = ["t", "px", "vx", "ax",
                  "py", "vy", "ay",  "pz", "vz", "az"]
    return df

    # # print((x))


def use_modeling_tool(A, B, N, Q, R, P, x0, xf, umax=None, umin=None, xmin=None, xmax=None):
    """
    solve MPC with modeling tool for test
    """
    (nx, nu) = B.shape

    # mpc calculation
    x = cvxpy.Variable((nx, N + 1))
    u = cvxpy.Variable((nu, N))

    costlist = 0.0
    constrlist = []

    k = cvxpy.Variable(N, boolean=True)

    for t in range(N):
        costlist += 0.5 * cvxpy.quad_form(x[:, t], Q)
        costlist += 0.5 * cvxpy.quad_form(u[:, t], R)
        costlist += k[t]**2

        constrlist += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

        if xmin is not None:
            constrlist += [x[:, t] >= xmin[:, 0]]
        if xmax is not None:
            constrlist += [x[:, t] <= xmax[:, 0]]

    costlist += 0.5 * cvxpy.quad_form(x[:, N], P)  # terminal cost
    if xmin is not None:
        constrlist += [x[:, N] >= xmin[:, 0]]
    if xmax is not None:
        constrlist += [x[:, N] <= xmax[:, 0]]

    if umax is not None:
        constrlist += [u <= umax]  # input constraints
    if umin is not None:
        constrlist += [u >= umin]  # input constraints

    constrlist += [x[:, 0] == x0[:, 0]]  # inital state constraints
    # constrlist += [x[:, -1] == xf[:, 0]]  # final state constraints

    zh = 0.2
    dh = 0.05
    M = 10.0
    # for t in range(N):
    # pass
    # constrlist += [k[0] == 0]
    # constrlist += [-(x[6, t] - zh) <= M*k[t]]
    # constrlist += [-(cvxpy.quad_form(x[:, t], np.diag(np.array([1,
    #                  0, 0, 0, 1, 0, 0, 0, 0]))) - dh) >= -M*(1 - k[t])]

    prob = cvxpy.Problem(cvxpy.Minimize(costlist), constrlist)

    prob.solve(verbose=True)

    return x.value, u.value


def getMinJerkTraj(dt, tf, x0, xf):

    t = np.arange(0.0, tf, dt)

    p0 = np.squeeze(x0[["px", "py", "pz"]].values)
    pf = np.squeeze(xf[["px", "py", "pz"]].values)

    s = t/tf
    s2 = s * s
    s3 = s2*s
    s4 = s3*s
    s5 = s4*s

    pt = np.empty([3, len(s)])
    vt = np.empty([3, len(s)])
    at = np.empty([3, len(s)])
    for i in range(3):
        pt[i] = p0[i] + (pf[i] - p0[i]) * (6.0 * s5 - 15.0*s4 + 10.0*s3)
        vt[i] = (pf[i] - p0[i]) * (30.0 * s4 - 60.0*s3 + 30.0*s2)/tf
        at[i] = (pf[i] - p0[i]) * (120.0 * s3 - 180.0*s2 + 60.0*s)/(tf*tf)

    # t = t[:, np.newaxis]
    xt = np.concatenate(
        [
            t[:, np.newaxis], np.transpose(
                pt), np.transpose(vt), np.transpose(at)
        ], axis=1
    )

    df = pd.DataFrame(xt)
    df.columns = ["t", "px", "py", "pz", "vx", "vy",
                  "vz", "ax", "ay", "az"]
    return df


def getInitState(n):
    x0 = np.concatenate(
        [
            (np.random.rand(n, 1) - 0.5)*0.5,  # px -> [-0.25, 0.25)
            np.zeros((n, 1)),  # vx
            np.zeros((n, 1)),  # ax
            (np.random.rand(n, 1) - 0.5)*0.5,  # py -> [-0.25, 0.25)
            np.zeros((n, 1)),  # vy
            np.zeros((n, 1)),  # ay
            (np.random.rand(n, 1) - 0.5)*0.1+0.3,  # pz -> [0.25, 0.35)
            np.zeros((n, 1)),  # vz
            np.zeros((n, 1)),  # az
        ], axis=1
    )

    df = pd.DataFrame(x0)
    df.columns = ["px", "vx", "ax",
                  "py", "vy", "ay",  "pz", "vz", "az"]
    return df


def getFinalState(n):
    xf = np.concatenate(
        [
            np.zeros((n, 1)),  # px
            np.zeros((n, 1)),  # vx
            np.zeros((n, 1)),  # ax
            np.zeros((n, 1)),  # py
            np.zeros((n, 1)),  # vy
            np.zeros((n, 1)),  # ay
            np.zeros((n, 1)),  # pz
            np.zeros((n, 1)),  # vz
            np.zeros((n, 1)),  # az
        ], axis=1
    )

    df = pd.DataFrame(xf)
    df.columns = ["px", "vx", "ax",
                  "py", "vy", "ay",  "pz", "vz", "az"]
    return df


def standardize(x):
    return (x - x.min()) / (x.max() - x.min())


if __name__ == "__main__":
    # t = np.arange(0, 10, 0.1)
    # nt = np.size(t)

    # xf = np.array([0.0, 0.0, 0.0])

    # p = np.empty((3, 0))
    # v = np.empty((1, 0))

    tf = 1.0
    dt = 0.1

    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2, projection="3d")
    for i in range(100):
        n = 1
        x0 = getInitState(n)
        xf = getFinalState(n)

        tf = (np.random.rand(1) - 0.5)*4.0 + 10.0  # 8.0 ~ 12.0
        xt = getMinJerkTraj(dt, tf, x0, xf)

        xt.to_csv(FILE_DIR+"/trajectory/"+str(i+1) +
                  ".csv", index=None)

        ax.plot(xt["px"], xt["py"], xt["pz"])

        A = np.zeros((3, 3))
        A[0, 1] = 1.0
        A[1, 2] = 1.0

        B = np.zeros((3, 1))
        B[2] = 1.0

        xt_qp = getMinJerkTraj_QP(dt, tf, x0, xf)

        ax2.plot(xt_qp["px"], xt_qp["py"], xt_qp["pz"])

        # print("x:" + str(np.mean((xt["px"]-xt_qp["px"].values))))
        # print("y:" + str(np.mean((xt["py"]-xt_qp["py"].values))))
        # print("z:" + str(np.mean((xt["pz"]-xt_qp["pz"].values))))
        # ax.plot(x[0, :], x[1, :], x[2, :])

    plt.show()

    # N = 10000
    # for i in range(N):
    #     trj = getTrajectory(t, getInitPos(1), xf)
    #     p = np.append(p, trj[0], axis=1)
    #     v = np.append(v, trj[1])

    # R = 0.1
    # grid_max = np.array([0.5, 0.5, 1.0])

    # grid_min = np.array([-0.5, -0.5, 0.0])
    # grid_idx = [
    #     np.floor((p[0, :] - (-grid_min[0])) / R).astype(int),
    #     np.floor((p[1, :] - (-grid_min[1])) / R).astype(int),
    #     np.floor((p[2, :] - (-grid_min[2])) / R).astype(int),
    # ]

    # ix = np.arange(grid_min[0], grid_max[0], R)
    # iy = np.arange(grid_min[1], grid_max[1], R)
    # iz = np.arange(grid_min[2], grid_max[2], R)
    # grid = np.meshgrid(ix, iy, iz)

    # grid_value = np.zeros((np.size(ix), np.size(iy), np.size(iz)))
    # # l = 0.3
    # # grid_value = 1.0 - np.exp(
    # #     -l * np.linalg.norm(np.meshgrid(ix - xf[0], iy - xf[1], iz - xf[2]), axis=0)
    # # )  #
    # grid_value_int = np.copy(grid_value)
    # grid_count = np.zeros((np.size(ix), np.size(iy), np.size(iz))) + 1
    # grid_appeared = np.zeros((np.size(ix), np.size(iy), np.size(iz)))

    # # update grid map
    # for i in range(0, len(v)):
    #     if v[i] > grid_value[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]]:
    #         # if (
    #         #     grid_count[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]] == 0
    #         # ):  # init with v[i]
    #         #     grid_value[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]] = v[i]
    #         # else:  # average
    #         grid_value[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]] = (
    #             grid_value[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]]
    #             * grid_count[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]]
    #             + v[i]
    #         ) / (grid_count[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]] + 1.0)
    #         grid_count[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]] += 1.0
    #     grid_appeared[grid_idx[0][i], grid_idx[1][i], grid_idx[2][i]] += 1.0

    # # standalize and mask unfined points
    # grid_value = standardize(grid_value)
    # grid_value = np.ma.masked_where(grid_appeared == 0, grid_value)

    # grid_value_int = standardize(grid_value_int)
    # grid_value_int = np.ma.masked_where(grid_appeared == 0, grid_value_int)

    # # evaluation
    # Ns = 20
    # ps = np.empty((3, 0))
    # vs = np.empty((1, 0))
    # for i in range(Ns):
    #     trj = getTrajectory(t, getInitPos(), xf)
    #     ps = np.append(ps, trj[0], axis=1)
    #     vs = np.append(vs, trj[1])

    # grids_idx = [
    #     np.floor((ps[0, :] - (-grid_min[0])) / R).astype(int),
    #     np.floor((ps[1, :] - (-grid_min[1])) / R).astype(int),
    #     np.floor((ps[2, :] - (-grid_min[2])) / R).astype(int),
    # ]

    # grids_value = []
    # for i in range(0, len(vs)):
    #     grids_value = np.append(
    #         grids_value, grid_value[grids_idx[0][i],
    #                                 grids_idx[1][i], grids_idx[2][i]]
    #     )

    # # plot
    # fig = plt.figure()
    # cmap = "jet"

    # nSubPlot = 3
    # ax = []
    # for i in range(nSubPlot):
    #     ax.append(fig.add_subplot(1, nSubPlot, i + 1, projection="3d"))
    #     ax[i].set_xlabel("x")
    #     ax[i].set_ylabel("y")
    #     ax[i].set_zlabel("z")

    # for i in range(0, 20):
    #     ax[0].plot(
    #         p[0, nt * i: nt * (i + 1)],
    #         p[1, nt * i: nt * (i + 1)],
    #         p[2, nt * i: nt * (i + 1)],
    #     )
    # # ax[0].scatter(grid[0], grid[1], grid[2], s=50.0, c=grid_value_int, cmap=cmap)

    # ax[1].scatter(grid[0], grid[1], grid[2], s=50.0, c=grid_value, cmap=cmap)

    # ax[2].scatter(ps[0, :], ps[1, :], ps[2, :], c=grids_value, cmap=cmap)

    # plt.show()
