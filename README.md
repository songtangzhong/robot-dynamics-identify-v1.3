# robot-dynamics-identify-v1.3

## Description:
This is a common robot dynamics parameters identification library, which can be used on all robots with revolute joint. The identification method is based on the least square method. Some examples are in package "examples".

## Dependence:
The library is based on Eigen(most version-3.3.7) and nlopt(most version-2.6.2). To use the library, the dependences should be installed firstly.

## Usage:
1. Build and install the library.
```
cd ~/
git clone https://github.com/songtangzhong/robot-dynamics-identify-v1.3.git
cd robot-dynamics-identify-v1.3/dynamics_library/build
rm -fr *
cmake ..
make
sudo make install
```

2. Run some examples.

(1) Calculate dynamics matrix.
```
cd robot-dynamics-identify-v1.3/examples/calculate_dynamics/build
rm -rf *
cmake ..
make
./bin/calculate_dynamics
```

(2) Generate excitation trajectory.
```
cd robot-dynamics-identify-v1.3/examples/excitation_trajectory/build
rm -rf *
cmake ..
make
./bin/excitation_trajectory
```
A file called "optimal_x.txt" will be generate in this "build" folder, which contains optimal parameters of fourier trajectory.

(3) Plot the generated excitation trajectory.

    In matlab environment, run "plot_excitation_trajectory.m".
