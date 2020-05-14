# Kidnapped Vehicle Project

Assuming that a robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS 
estimate of its initial location, and lots of (noisy) sensor and control data.
In this project I will implement a 2 dimensional particle filter in C++. The particle filter will be given a map and 
some initial localization information (analogous to what a GPS would provide). 
At each time step The filter will also get observation and control data.

---

## How to compile and run
1. Download the Term 2 Simulator [here](https://github.com/udacity/self-driving-car-sim/releases).
2. Install `uWebSocketIO`: <br>
This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) 
for either Linux or Mac systems. For windows you can use either Docker, VMware, 
or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)<br>
You can execute the `install-ubuntu.sh` to install uWebSocketIO.

3. Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.
    ```shell script
    mkdir build
    cd build
    cmake ..
    make
    ./particle_filter
    ```
Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the 
following in the top directory of the project:
    ```shell script
    ./clean.sh
    ./build.sh
    ./run.sh
    ```
## Input & Output data notes

### Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

## Implementing the Particle Filter
The directory structure of this repository is as follows:

```shell script
${ROOT}
├──build.sh
├──clean.sh
├──CMakeLists.txt
├──README.md
├──run.sh
├──data/
    ├──map_data.txt
├──src/
    ├──helper_functions.h
    ├──main.cpp
    ├──map.h
    ├──particle_filter.cpp
    ├──particle_filter.h
```

## Success Criteria
1. **Accuracy**: The particle filter should localize vehicle position and yaw to within the values specified in the 
parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: The particle filter should complete execution within the time of 100 seconds.