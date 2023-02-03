
from matplotlib import pyplot as plt
import os  # noqa
import sys  # noqa
FILE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(FILE_DIR, '../../3rd_party/dmpbbo'))  # noqa

import dmpbbo.json_for_cpp as json_for_cpp  # noqa
from dmpbbo.dmps.Dmp import Dmp  # noqa
from dmpbbo.dmps.Trajectory import Trajectory  # noqa
from dmpbbo.functionapproximators.FunctionApproximatorRBFN import FunctionApproximatorRBFN  # noqa


def main():
    """ Main function for script. """

    # Train a DMP with a trajectory
    traj = Trajectory.loadcsv(FILE_DIR+"/trajectory/case_0/1.csv")
    function_apps = [FunctionApproximatorRBFN(
        20, 0.7) for _ in range(traj.dim)]
    dmp = Dmp.from_traj(traj, function_apps, dmp_type="KULVICIUS_2012_JOINING")

    # Compute analytical solution
    xs, xds, _, _ = dmp.analytical_solution(traj.ts)
    traj_reproduced = dmp.states_as_trajectory(traj.ts, xs, xds)

    # Numerical integration
    dt = 0.002
    n_time_steps = int(1.3 * traj.duration / dt)
    x, xd = dmp.integrate_start()
    for tt in range(1, n_time_steps):
        x, xd = dmp.integrate_step(dt, x)
        # Convert complete DMP state to end-eff state
        y, yd, ydd = dmp.states_as_pos_vel_acc(x, xd)

    # Save the DMP to a json file that can be read in C++
    filename = "dmp_for_cpp.json"
    json_for_cpp.savejson_for_cpp(os.path.join(FILE_DIR, filename), dmp)
    print(f"Saved {filename} to this script directory.")

    dmp.plot(plot_demonstration=traj)
    dmp.plot_comparison(traj)
    plt.show()


if __name__ == "__main__":
    main()
