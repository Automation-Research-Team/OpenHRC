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
import os
import pandas as pd
from movement_primitives.promp import ProMP
from movement_primitives.dmp import DMP
from movement_primitives.data import generate_1d_trajectory_distribution
import matplotlib.pyplot as plt
import numpy as np

from moveit_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint

print(__doc__)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "trajectory")
n_case = sum(os.path.isdir(os.path.join(DATA_DIR, name))
             for name in os.listdir(DATA_DIR))
print("load trajectories in directories of case_0 to case_"+str(n_case))

trajs = [[pd.read_csv(DATA_DIR+"/case_" +
                      str(k)+"/"+str(i+1) + ".csv") for i in range(100)] for k in range(n_case)]


mode = 3

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
# Y_mean = promp.mean_trajectory(T[0])
# Y_conf = 1.96 * np.sqrt(promp.var_trajectory(T[0]))

# plt.figure(figsize=(10, 5))

# ax1 = plt.subplot(121)
# ax1.set_title("Training set and ProMP")
# ax1.fill_between(T, (Y_mean - Y_conf).ravel(),
#                  (Y_mean + Y_conf).ravel(), color="r", alpha=0.3)
# ax1.plot(T[0], Y_mean, c="r", lw=2, label="ProMP")
# for i in range(n_demos):
#     ax1.plot(T[i], Y[i].T, c="k", alpha=0.1)
# ax1.set_xlim((-0.05, 1.05))
# # ax1.set_ylim((-3.5, 3.5))
# ax1.legend(loc="best")

# ax2 = plt.subplot(122)
# ax2.set_title("Conditional ProMPs")


rospy.init_node('promp_state_publisher')  # ノードの生成
pub = rospy.Publisher('chatter', State, queue_size=1)
pub_trj = rospy.Publisher('trj', CartesianTrajectory, queue_size=1)
rate = rospy.Rate(1)
# for color, y_cond in zip("rgbcmyk", np.linspace(-0.3, 0.3, 7)):
y_cond = [-0.1, 0., 0.3]

t_max = 12.0
while not rospy.is_shutdown():
    cpromp = promp.condition_position(
        np.array(y_cond), y_cov=y_conditional_cov, t=0.0, t_max=t_max)

    time = np.arange(0.0, 12.0, 0.01)
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

    pub_trj.publish(trj)
    # t += 1.0/10.0
    rate.sleep()

rospy.spin()
