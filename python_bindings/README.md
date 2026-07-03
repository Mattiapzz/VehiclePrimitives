# Python Bindings for Vehicle Primitives (`vprim_py`)

This directory contains the Python bindings for the `vprim` C++ library. It allows you to use the `Polynomial`, `CubicSpiral`, and `SpeedMatching` primitives directly from Python.

## Installation

To build and install the Python package, you need a C++17 compatible compiler. We provide two recommended ways to set up your environment: using `conda` or using a Python virtual environment (`venv`).

### Option 1: Using Conda (Recommended)

Conda can automatically install a compatible C++ compiler for you, which simplifies the setup process across different platforms (Linux, macOS, Windows).

1.  **Create and activate the conda environment:**
    From this directory (`python_bindings/`), run the following command. It will create an environment named `vprim_py_env` using the provided `environment.yml` file.

    ```bash
    conda env create -f environment.yml
    conda activate vprim_py_env
    ```

2.  **Install the package:**
    Once the environment is active, install the package in editable mode using `pip`.

    ```bash
    pip install -e .
    ```

### Option 2: Using `venv` and a system compiler

If you prefer not to use Conda, you can use a standard Python virtual environment. You must ensure you have a C++17 compiler (e.g., GCC, Clang, or MSVC) installed and available in your system's PATH.

```bash
# Navigate to this directory (python_bindings/)
# It is highly recommended to use a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install the package in editable mode (-e).
pip install -e .
```

This command will compile all the C++ source code and create a Python module that you can import.

## Usage

Once installed, you can import and use the classes in your Python code:

```python
import vprim_py as vprim

# --- Using Polynomial ---
p = vprim.Polynomial([0, -1, 1]) # 1*x^2 - 1
print(f"Polynomial: {p}")
print(f"Root near 0.5 is: {p.closest_root(0.5)}")

# --- Using CubicSpiral ---
spiral = vprim.CubicSpiral()
start = vprim.SpatialState(0, 0, 0, 0)
goal = vprim.SpatialState(10, 3, 0, 0)
assert spiral.build(start, goal)
print(f"Spiral length: {spiral.length}")

# --- Using SpeedMatching ---
sm = vprim.SpeedMatching(weight_time=1.0)
start_lon = vprim.LongitudinalState(s=0, v=10, a=0, j=0)
goal_lon = vprim.LongitudinalState(s=100, v=20, a=0, j=0)

if sm.build(start_lon, goal_lon):
    print(f"Speed profile generated with optimal time T = {sm.T:.2f} s")
    # Evaluate state at t=1.0s
    state_at_1s = sm.eval(1.0)
    print(f"Velocity at t=1s: {state_at_1s.v:.2f} m/s")

## Running the Examples

This package includes detailed examples with plotting. To run them, you will need `matplotlib` and `numpy`. If you used the `environment.yml` file to create your environment, they are already installed.

To see a visual demonstration of the `SpeedMatching` primitive, run the following command from this directory:
```bash
python example_speed_matching.py
```

