"""
=================
Conditional ProMP
=================

Probabilistic Movement Primitives (ProMPs) define distributions over
trajectories that can be conditioned on viapoints. In this example, we
plot the resulting posterior distribution after conditioning on varying
start positions.
"""
from ohrc_msgs.msg import State
import rospy
import sys
import os
import pandas as pd
from movement_primitives.promp import ProMP
from movement_primitives.dmp import DMP
from movement_primitives.data import generate_1d_trajectory_distribution
import matplotlib.pyplot as plt
import numpy as np

from moveit_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
from ohrc_msgs.srv import GetTrajectories, GetTrajectoriesResponse


def load_data():
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = os.path.join(SCRIPT_DIR, "trajectory")
    n_case = sum(os.path.isdir(os.path.join(DATA_DIR, name))
                 for name in os.listdir(DATA_DIR))
    trajs = [[pd.read_csv(DATA_DIR+"/case_" +
                          str(k)+"/"+str(i+1) + ".csv") for i in range(100)] for k in range(n_case)]

    return trajs, n_case


def select_dataset(trajs, mode=3):
    n_demos = 100
    T = [np.array(trajs[mode][i]["t"].values) for i in range(n_demos)]
    px = np.array(
        [trajs[mode][i]["px"].values for i in range(n_demos)], dtype=object)
    py = [trajs[mode][i]["py"].values for i in range(n_demos)]
    pz = [trajs[mode][i]["pz"].values for i in range(n_demos)]

    Y = [np.array([[px[i][j], py[i][j], pz[i][j]]
                   for j in range(px[i].size)]) for i in range(n_demos)]

    dy = trajs[mode][0]["vx"]
    ddy = trajs[mode][0]["ax"]

    return T, Y, dy, ddy


def main(mode=3):
    rospy.init_node('promp_trajectory_service')  # ノードの生成

    trajs, n_case = load_data()

    rospy.loginfo(
        "The selected trajectory case [0-" + str(n_case) + "] is " + str(mode) + ".")

    T, Y, dy, ddy = select_dataset(trajs, mode)

    promp = ProMP(n_dims=3, n_weights_per_dim=10)
    promp.imitate(T, Y)

    y_conditional_cov = np.array([0.0000025, 0.0000025, 0.0000025])
    t_max = 12.0

    def generate_trjs(req):
        targetPoses = req.targetPoses
        trjs = []

        for pose in targetPoses.poses:
            y_cond = [-pose.position.x, -pose.position.y, -pose.position.z]
            cpromp = promp.condition_position(
                np.array(y_cond), y_cov=y_conditional_cov, t=0.0, t_max=t_max)

            time = np.arange(0.0, t_max, 0.01)
            Y_cmean = cpromp.mean_trajectory(time)
            dY_cmean = cpromp.mean_velocities(time)

            trj = CartesianTrajectory()
            trj.header.frame_id = "world"
            trj.header.stamp = rospy.Time.now()
            for i in range(len(time)):
                point = CartesianTrajectoryPoint()
                point.point.pose.position.x = Y_cmean[i, 0] - y_cond[0]
                point.point.pose.position.y = Y_cmean[i, 1] - y_cond[1]
                point.point.pose.position.z = Y_cmean[i, 2] - y_cond[2]
                point.point.velocity.linear.x = dY_cmean[i, 0]/t_max
                point.point.velocity.linear.y = dY_cmean[i, 1]/t_max
                point.point.velocity.linear.z = dY_cmean[i, 2]/t_max
                point.time_from_start = rospy.Duration.from_sec(time[i]*t_max)
                trj.points.append(point)
            trjs.append(trj)

        rospy.loginfo("Generated trajectories.")
        return GetTrajectoriesResponse(trjs)

    s = rospy.Service('/trajectory_generation_service',
                      GetTrajectories, generate_trjs)

    rospy.loginfo("Ready to generate trajectories.")
    rospy.spin()


if __name__ == '__main__':
    args = sys.argv

    if len(args) == 2:
        main(int(args[1]))
    else:
        main()
