"""
Export every example plot in this folder to TikZ/pgfplots, matching the
house style already established on the MATLAB side (see tikz_utils.py's
docstring). Each plot_*.py script already builds its figure(s) as plain
module-level variables (`fig`, or `fig1`/`fig2`) and guards its PNG-saving /
plt.show() side effects behind `if __name__ == "__main__":` -- so importing a
module here runs its plotting code once and exposes the figure object(s)
directly, with no need to refactor those scripts or duplicate their plotting
logic.

Usage (from this directory, in an environment with vprim_py + matplot2tikz,
e.g. vprim_py_env):
    MPLBACKEND=Agg python3 export_all_tikz.py [--force]

Output lands in figures_export/<name>/ (+ figures_export/<name>_exported_tikz.pdf),
mirroring motionsick/matlab-old/TestAndPlot/figures_export/'s layout.
"""

import argparse
import importlib

import matplotlib

matplotlib.use("Agg")  # must be set before importing the plot_*.py modules

from tikz_utils import compile_tikz_folder, export_tikz_figure

ROOT_FOLDER = "figures_export"

# module name -> {figure attribute name in that module: exported tikz name}
MODULES_AND_FIGURES = {
    "plot_mp_lon_mintminj": {
        "fig": "MP_LON_MINTMINJ_states",
    },
    "plot_motion_sickness_primitive": {
        "fig1": "motion_sickness_sweep",
        "fig2": "motion_sickness_vs_unfiltered",
    },
    "plot_mintminj_vs_sickness_wT_sweep": {
        "fig": "mintminj_vs_sickness_wT_sweep",
    },
    "plot_comfort_matching": {
        "fig1": "comfort_matching_sweep",
        "fig2": "comfort_matching_comparison",
    },
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true", help="recompile even if up to date")
    args = parser.parse_args()

    for module_name, fig_map in MODULES_AND_FIGURES.items():
        print(f"=== {module_name} ===")
        module = importlib.import_module(module_name)
        for attr_name, export_name in fig_map.items():
            fig = getattr(module, attr_name)
            export_tikz_figure(fig, export_name, ROOT_FOLDER)
            compile_tikz_folder(ROOT_FOLDER, export_name, force=args.force)


if __name__ == "__main__":
    main()
