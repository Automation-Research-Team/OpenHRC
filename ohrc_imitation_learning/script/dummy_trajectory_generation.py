import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import cvxpy
from multiprocessing import Pool


FILE_DIR = os.path.dirname(os.path.abspath(__file__))


def getMinJerkTraj_QP(dt, tf, x0, xf, constraint=None):
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
    P = np.eye(9)*1.0e6
    x0_ = x0[["px", "vx", "ax", "py", "vy", "ay",  "pz", "vz", "az"]].values
    xf_ = xf[["px", "vx", "ax", "py", "vy", "ay",  "pz", "vz", "az"]].values

    x, u = use_modeling_tool(A, B, N, Q, R, P, x0_.T, xf_.T, constraint)

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


def use_modeling_tool(A, B, N, Q, R, P, x0, xf, constraint=None, umax=None, umin=None, xmin=None, xmax=None):
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
        costlist += 1.0e-6*k[t]

        constrlist += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t]]

        if xmin is not None:
            constrlist += [x[:, t] >= xmin[:, 0]]
        if xmax is not None:
            constrlist += [x[:, t] <= xmax[:, 0]]

    costlist += 0.5 * cvxpy.quad_form(x[:, 0]-x0[:, 0], P)  # terminal cost
    costlist += 0.5 * cvxpy.quad_form(x[:, -1]-xf[:, 0], P)  # terminal cost
    if xmin is not None:
        constrlist += [x[:, N] >= xmin[:, 0]]
    if xmax is not None:
        constrlist += [x[:, N] <= xmax[:, 0]]

    if umax is not None:
        constrlist += [u <= umax]  # input constraints
    if umin is not None:
        constrlist += [u >= umin]  # input constraints

    # constrlist += [x[:, 0] == x0[:, 0]]  # inital state constraints
    # constrlist += [x[:, -1] == xf[:, 0]]  # final state constraints

    if constraint is not None:
        zh = constraint["zh"]  # 0.1
        dh = constraint["dh"]  # 0.05
        M = 1.0e2
        # if constraint["reverse"]:
        # a = np.diag(np.array([1,
        #   0, 0, 0, 0, 0, 1, 0, 0]))
        # else:
        a = np.diag(np.array([1,
                              0, 0, 1, 0, 0, 0, 0, 0]))
        # Big-M method
        for t in range(N):
            constrlist += [-(x[6, t] - zh) <= M*k[t]]
            constrlist += [-(cvxpy.quad_form(x[:, t], a) -
                             dh**2) >= -M*(1 - k[t])]

    prob = cvxpy.Problem(cvxpy.Minimize(costlist), constrlist)

    prob.solve(verbose=False)

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


def sample_func(args):
    i, seed = args
    np.random.seed(seed)

    print("trajectory #: " + str(i))
    dt = 0.1
    n = 1
    x0 = getInitState(n)
    xf = getFinalState(n)

    xt = []
    constraints = []

    tf = (np.random.rand(1) - 0.5)*2.0 + 3.0  # 2.0 ~ 4.0
    xt.append(getMinJerkTraj(dt, tf, x0, xf))
    constraints.append(None)

    xt.append(getMinJerkTraj_QP(dt, tf, x0, xf, constraint=None))
    constraints.append(None)

    zh = [0.1, 0.2]
    dh = [0.03]

    for z in zh:
        for d in dh:
            constraint = dict()
            constraint["zh"] = z
            constraint["dh"] = d

            if z == 0.1:
                constraint["reverse"] = True
            else:
                constraint["reverse"] = False
            xt.append(getMinJerkTraj_QP(dt, tf, x0, xf, constraint))
            constraints.append(constraint)

    return xt, constraints


def main():
    N = 100

    # sample_func((N, 0))
    with Pool(processes=os.cpu_count()) as p:
        results = p.map(
            func=sample_func, iterable=zip(range(N), np.random.randint(0, 2 ** 32 - 1, N)))

    n_case = len(results[0][0])
    fig = plt.figure()
    ax = [fig.add_subplot(1, n_case, i+1, projection="3d")
          for i in range(n_case)]

    t = np.arange(0., 2*np.pi, 0.01)

    for i, a in enumerate(results):
        result, c = a[0], a[1]
        for j, xt in enumerate(result):
            ax[j].plot(xt["px"], xt["py"], xt["pz"])

            if i == N-1:
                if c[j] is not None:
                    ax[j].plot(c[j]["dh"]*np.cos(t), c[j]["dh"] *
                               np.sin(t), c[j]["zh"]*np.ones(t.size))

                    ax[j].axis('equal')
            directory = FILE_DIR+"/trajectory/case_"+str(j)
            if not os.path.exists(directory):
                os.makedirs(directory)
            xt.to_csv(directory+"/"+str(i+1) + ".csv", index=None)
    plt.show()


if __name__ == "__main__":
    main()
