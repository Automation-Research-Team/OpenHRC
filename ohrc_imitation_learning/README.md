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
### References:
> Dynamic Movement Primitives in Robotics: A Tutorial Survey https://arxiv.org/abs/2102.03861

> pydmps https://github.com/studywolf/pydmps

> dmpbbo https://github.com/stulp/dmpbbo

### Implementation
In the current implementation, this package is using 
> Movement Primitives https://github.com/dfki-ric/movement_primitives

which can be installed via pip and provides conditional DMP (ProMPs).
For futher study, I'm planning to implement my DMP codes for fitting HRC more.