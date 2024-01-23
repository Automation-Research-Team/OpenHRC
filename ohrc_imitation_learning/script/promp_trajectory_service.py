#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import pandas as pd
from movement_primitives.promp import ProMP

import numpy as np

from moveit_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
from ohrc_msgs.srv import GetTrajectories, GetTrajectoriesResponse


def load_data():
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    DATA_DIR = os.path.join(SCRIPT_DIR, "trajectory")
    n_case = sum(os.path.isdir(os.path.join(DATA_DIR, name))
                 for name in os.listdir(DATA_DIR))
    trajs = [[pd.read_csv(DATA_DIR+"/case_" +
                          str(k)+"/"+str(i) + ".csv") for i in range(17)] for k in range(2)]

    return trajs, n_case


def select_dataset(trajs, mode=3):
    n_demos = 17
    T = [np.array(trajs[mode][i]["t"].values) for i in range(n_demos)]
    px = np.array(
        [trajs[mode][i]["px"].values for i in range(n_demos)], dtype=object)
    py = [trajs[mode][i]["py"].values for i in range(n_demos)]
    pz = [trajs[mode][i]["pz"].values for i in range(n_demos)]

    Y = [np.array([[px[i][j], py[i][j], pz[i][j]]
                   for j in range(px[i].size)]) for i in range(n_demos)]

    # dy = trajs[mode][0]["vx"]
    # ddy = trajs[mode][0]["ax"]

    return T, Y


def main(mode=1):
    rospy.init_node('promp_trajectory_service')  # ノードの生成
    pub_trj = rospy.Publisher('trj', CartesianTrajectory, queue_size=1)

    trajs, n_case = load_data()

    rospy.loginfo(
        "The selected trajectory case [0-" + str(n_case) + "] is " + str(mode) + ".")

    T, Y = select_dataset(trajs, mode)

    promp = ProMP(n_dims=3, n_weights_per_dim=10)
    promp.imitate(T, Y)

    y_conditional_cov = np.array([0.000025, 0.000025, 0.000025])

    dt = 0.01

    global targetPoses_prev, trjs_prev
    targetPoses_prev = None
    trjs_prev = None

    def generate_trjs(req):
        t_max = 2.5  # + (np.random.rand() - 0.5) / 0.5*0.5  # 2.0 ~ 3.0
        targetPoses = req.targetPoses
        restPose = req.restPose

        global targetPoses_prev, trjs_prev
        if targetPoses_prev is None:
            targetPoses_prev = targetPoses
            for pose_prev in targetPoses_prev.poses:
                pose_prev.position.x = 0.0
                pose_prev.position.y = 0.0
                pose_prev.position.z = 0.0

        trjs = []

        for i, (pose, pose_prev) in enumerate(zip(targetPoses.poses,  targetPoses_prev.poses)):

            trj = CartesianTrajectory()
            trj.header.frame_id = "world"
            trj.header.stamp = rospy.Time.now()

            if trjs_prev is not None and np.linalg.norm((np.array([pose.position.x, pose.position.y, pose.position.z]) - np.array([pose_prev.position.x, pose_prev.position.y, pose_prev.position.z]))) < 0.01:
                trj = trjs_prev[i]
                rospy.loginfo("Skipped trajectory generation.")
            else:

                targetPoses_prev.poses[i] = pose

                y_cond = [-pose.position.x, -pose.position.y, -pose.position.z]
                cpromp = promp.condition_position(
                    np.array(y_cond), y_cov=y_conditional_cov, t=0.0, t_max=t_max)

                time = np.arange(0.0, t_max, dt)
                Y_cmean = cpromp.mean_trajectory(time)
                dY_cmean = cpromp.mean_velocities(time)

                point = CartesianTrajectoryPoint()
                point.point.pose.position.x = 0.0
                point.point.pose.position.y = 0.0
                point.point.pose.position.z = 0.0
                point.point.velocity.linear.x = 0.0
                point.point.velocity.linear.y = 0.0
                point.point.velocity.linear.z = 0.0
                point.time_from_start = rospy.Duration.from_sec(0.0)
                trj.points.append(point)

                for i in range(len(time)):
                    point = CartesianTrajectoryPoint()
                    point.point.pose.position.x = Y_cmean[i, 0] - y_cond[0]
                    point.point.pose.position.y = Y_cmean[i, 1] - y_cond[1]
                    point.point.pose.position.z = Y_cmean[i, 2] - y_cond[2]
                    point.point.velocity.linear.x = dY_cmean[i, 0]/t_max
                    point.point.velocity.linear.y = dY_cmean[i, 1]/t_max
                    point.point.velocity.linear.z = dY_cmean[i, 2]/t_max
                    point.time_from_start = rospy.Duration.from_sec(
                        time[i]*t_max+dt)
                    trj.points.append(point)

                point = CartesianTrajectoryPoint()
                point.point.pose.position.x = -y_cond[0]
                point.point.pose.position.y = -y_cond[1]
                point.point.pose.position.z = -y_cond[2]
                point.point.velocity.linear.x = 0.0
                point.point.velocity.linear.y = 0.0
                point.point.velocity.linear.z = 0.0
                point.time_from_start = rospy.Duration.from_sec(
                    time[i]*t_max+dt+dt)
                trj.points.append(point)

            trjs.append(trj)

        # targetPoses_prev = targetPoses
        trjs_prev = trjs

        # pub_trj.publish(trjs[0])

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
