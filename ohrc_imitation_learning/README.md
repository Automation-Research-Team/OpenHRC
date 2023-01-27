# ohrc_imitation_learning


## install

Recommend installing in a new env using conda
```sh
$ conda create -n ohrc python=3.10
$ conda activate ohrc
$ conda install numpy matplotlib pandas jsonpickle
$ conda install -c conda-forge cvxpy pyscipopt
```

## generating pseudo data of reaching motion
```sh
$ conda activate ohrc
$ python script/dummy_trajectory_generation.py
```
This python script generates trajectories with several constraints in `script/trajectory`.


## train DMP
