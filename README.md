# Vehicle Primitives

This repository contains a set of motion primitives designed for vehicle navigation and control. These primitives can be used to generate feasible trajectories for various types of vehicles, including cars, trucks, and autonomous robots.

## Requirements

### General

- git
- cmake
- make

### Third party libraries

Make sure to have all the required submodules installed. You can do this by running the following command:

```{shell}
git submodule update --init --recursive
```
  
## Building

To install the project you need to run the following commands:

```{shell}
./build.sh
```

Default will build the project in release mode, if you want to build in debug mode you can run:

```{shell}
./build.sh -debug
```

For further information you can run:

```{shell}
./build.sh -h
```

### Alternative build

If you want to install the project in a different directory you can run:

```{shell}
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" ..
make -j
```

