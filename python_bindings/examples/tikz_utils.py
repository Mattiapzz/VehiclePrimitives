"""
Matplotlib -> TikZ/pgfplots export utilities for vprim_py example plots.

Mirrors the house style already established on the MATLAB side
(motionsick/matlab-old/TestAndPlot/utils/export2tikz*.m,
motionsick/matlab-old/TestAndPlot/compileAllTikz.sh,
motionsick/tikz_example/tikz_utils.py) so Python- and MATLAB-generated
figures compile into the same paper/report with a consistent look:
externalized data tables, a small `_main.tex` preview wrapper (pgfplots +
times + \\PreviewEnvironment), compiled with pdflatex.

Two entry points, replacing the earlier ad hoc helpers with one configurable
function each:
  - export_tikz_figure(): replaces the three near-duplicate
    save_tikz_figure/2/3 functions in motionsick/tikz_example/tikz_utils.py
    with one function and explicit keyword options.
  - compile_tikz_folder(): a dependency-free Python equivalent of
    compileAllTikz.sh (same incremental-skip-if-unchanged behavior),
    callable per-figure instead of only as a standalone shell script.

Requires matplot2tikz (the maintained successor to tikzplotlib; same save()
API, confirmed via its TikzArgs TypedDict to accept the same keyword
arguments used here). Available in the vprim_py_env conda environment.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import matplot2tikz

# --- Known matplot2tikz 0.5.4 bugs, worked around here rather than waiting
# on an upstream fix (both confirmed by reading the installed package source
# and reproducing the exact pdflatex failure each one causes) ---
#
# Bug 1: the externalized-table writer (_line2d.py's
# `content.append(f"table {{{opts_str}}}{{{posix_filepath}}};\n")`) always
# wraps the (possibly-empty) options string in its OWN spurious `{}` before
# the filename's `{}`:
#   no options: "table {}{file.dat};"          (should be "table {file.dat};")
#   with options: "table {[opt1,opt2] }{file.dat};"  (should be "table[opt1,opt2] {file.dat};")
# -> "\begin{tikzpicture} ... ended by \end{document}" from the malformed
#    brace nesting.
_BROKEN_TABLE_RE = re.compile(r"table \{(.*?)\}\{")

# Bug 2: _axes.py's `_get_ticks()` emits a literal
# "scaled {x,y} ticks=manual:{}{\pgfmathparse{#1}}" axis option whenever an
# axis has ticks but no tick labels (e.g. every upper subplot of a
# `sharex=True` figure, where matplotlib hides the shared tick labels) to
# suppress pgfplots' multiplier annotation. A bare `#1` is only legal inside
# a macro body being \def'd, not loose in an options list -> "Illegal
# parameter number in definition of \pgfmath@expression". Since we don't
# want a multiplier shown anyway (our axes use plain values), the fix is to
# just drop the whole option -- it has no effect beyond suppressing text we
# don't want in the first place.
_BROKEN_SCALED_TICKS_RE = re.compile(r"scaled [xy] ticks=manual:\{\}\{\\pgfmathparse\{#1\}\},?\n?")

# Bug 3: _util.py's `_tex_escape()` does `text.replace("_", r"\_")` on the
# WHOLE label string before splitting it into text-mode/math-mode segments
# (on unescaped `$`). So a legitimate math-mode subscript like matplotlib's
# own `$v_x$` label gets its underscore escaped to `\_` too, which inside
# math mode prints a literal underscore GLYPH instead of subscripting --
# "v_x" instead of "v" with a subscript "x". Confirmed by inspecting a
# generated label: `ylabel={\(\displaystyle v\_x\) [...]}`. Fixed by
# reversing `\_` back to `_` specifically inside `\(\displaystyle ... \)`
# math-mode spans (left alone outside them, where `\_` is the correct escape
# for a literal underscore in plain text).
_MATH_MODE_RE = re.compile(r"\\\(\\displaystyle(.*?)\\\)", re.DOTALL)


def _unescape_underscores_in_math_mode(tex_text: str) -> str:
    def _fix_span(match: re.Match) -> str:
        return r"\(\displaystyle" + match.group(1).replace(r"\_", "_") + r"\)"

    return _MATH_MODE_RE.sub(_fix_span, tex_text)


def _fix_known_matplot2tikz_bugs(tex_text: str) -> str:
    tex_text = _BROKEN_TABLE_RE.sub(r"table \1{", tex_text)
    tex_text = _BROKEN_SCALED_TICKS_RE.sub("", tex_text)
    tex_text = _unescape_underscores_in_math_mode(tex_text)
    return tex_text


_MAIN_TEX_TEMPLATE = r"""\documentclass[{doc_class_opts}]{{article}}
\usepackage[active,tightpage]{{preview}}
\usepackage{{pgfplots}}
\pgfplotsset{{compat=1.18}}
\usepgfplotslibrary{{groupplots}}
\usepackage{{times}}
\PreviewEnvironment{{tikzpicture}}
\setlength\PreviewBorder{{2pt}}
\begin{{document}}
\pgfkeys{{/pgf/number format/.cd,1000 sep={{}}}}
\input{{{tex_name}}}
\end{{document}}
"""

# Standard LaTeX article class paper-size options. 'default' omits the
# option entirely (LaTeX's own default, letterpaper in most distributions).
_VALID_PAPERS = {"default", "a4paper", "a5paper", "b5paper", "letterpaper", "legalpaper", "executivepaper"}

_WIDTH_RE = re.compile(r"^([0-9.]+)\s*(\\\S+)$")


def _infer_axis_height(fig, width: str) -> str:
    """Compute a per-row `axis_height` from the figure's own aspect ratio,
    matching export2tikzWH.m's approach (derive both dimensions from the
    actual matplotlib figure size) but correctly divided across subplot
    rows. Without this, leaving `axis_height=None` lets matplot2tikz infer a
    height per axis independently of how many rows share the figure -- for
    an N-row `sharex=True` stack this gives each row close to the WHOLE
    original figure's height, making the exported figure ~N times too tall.

    Assumes a single-column stack of N rows (`plt.subplots(N, 1, ...)`), the
    only layout used in this project's example scripts -- `len(fig.axes)` is
    used directly as the row count. A general row/col grid would need the
    actual GridSpec shape instead.
    """
    match = _WIDTH_RE.match(width.strip())
    if not match:
        # Can't parse e.g. a bare LaTeX length or percentage; fall back to
        # matplot2tikz's own default behavior.
        return None
    width_coeff, unit = float(match.group(1)), match.group(2)

    fig_w_in, fig_h_in = fig.get_size_inches()
    n_rows = max(len(fig.axes), 1)

    total_height_coeff = width_coeff * (fig_h_in / fig_w_in)
    per_row_height_coeff = total_height_coeff / n_rows
    return f"{per_row_height_coeff:.4f}{unit}"


def export_tikz_figure(
    fig,
    name: str,
    root_folder: str | Path,
    *,
    width: str = r"1.0\columnwidth",
    height: str | None = None,
    externalize_tables: bool = True,
    clean: bool = True,
    decimal_precision: int = 3,
    scaled_y_ticks: bool = False,
    vertical_sep: str | None = "0.2cm",
    paper: str = "letterpaper",
    pt_size: int = 10,
    columns: int = 1,
    extra_doc_class_opts: str = "",
    extra_lines: list[str] | None = None,
    strip_suptitle: bool = True,
) -> Path:
    """Export a matplotlib Figure to <root_folder>/<name>/<name>.tex (+
    externalized data files) plus a <name>_main.tex preview wrapper, matching
    the MATLAB export2tikz.m/export2tikzWH.m house style exactly (same
    documentclass/preview/pgfplots/times preamble).

    width/height follow pgfplots dimension syntax (e.g. '0.8\\columnwidth').
    height=None (default) computes a per-row height from the figure's own
    aspect ratio via _infer_axis_height() -- NOT the same as leaving it to
    matplot2tikz's own default, which infers height per-axis independently
    of row count and makes multi-row `sharex=True` figures far too tall.
    Pass an explicit height string to override.

    vertical_sep applies to `plt.subplots` figures (matplot2tikz emits these
    as a pgfplots groupplot); set to None for single-axis figures.

    paper: one of 'a4paper', 'a5paper', 'b5paper', 'letterpaper' (default),
    'legalpaper', 'executivepaper', or 'default' to omit the class option
    entirely (LaTeX's own default). Controls the _main.tex PREVIEW wrapper's
    page size only -- since the `preview` package crops to the figure's own
    bounding box (\\PreviewBorder), this mostly matters if you drop the
    `preview` cropping and want a literal full page; it does NOT change
    `width`/`height` (the figure's own size in \\columnwidth units), which
    you still set independently. pt_size is the base font size (9/10/11/12pt
    are standard); columns=2 adds 'twocolumn' (matches the MATLAB pipeline's
    export2tikz.m), columns=1 omits it. extra_doc_class_opts appends any
    further raw class options (e.g. 'landscape') for cases these three don't
    cover.

    strip_suptitle: matplot2tikz renders `fig.suptitle()` as a raw, UNESCAPED
    TeX text node -- any LaTeX-special character in it (notably literal `_`,
    which is a math-mode-only subscript operator outside math mode) breaks
    the compile. Matches the MATLAB pipeline's convention too (none of the
    Plot_MP_*.m scripts set an equivalent whole-figure title -- the caption
    lives in the surrounding paper, not the figure). Default True hides the
    figure's suptitle for the duration of this export only (restored
    immediately after), so on-screen/PNG output is unaffected.

    Returns the folder containing the exported files.
    """
    if paper not in _VALID_PAPERS:
        msg = f"paper={paper!r} not recognized; use one of {sorted(_VALID_PAPERS)}"
        raise ValueError(msg)
    opts = [] if paper == "default" else [paper]
    opts.append(f"{pt_size}pt")
    if columns == 2:
        opts.append("twocolumn")
    elif columns != 1:
        msg = f"columns must be 1 or 2, got {columns}"
        raise ValueError(msg)
    if extra_doc_class_opts:
        opts.append(extra_doc_class_opts)
    doc_class_opts = ", ".join(opts)

    if height is None:
        height = _infer_axis_height(fig, width)

    folder = Path(root_folder) / name
    if clean and folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True, exist_ok=True)

    tex_path = folder / f"{name}.tex"

    number_format_lines = [
        "\\pgfkeys{/pgf/number format/.cd,1000 sep={}}\n",
        f"\\pgfplotsset{{scaled y ticks={'true' if scaled_y_ticks else 'false'}}}\n",
        "\\pgfplotsset{y tick label style={/pgf/number format/.cd,"
        f"precision={decimal_precision},}}}}\n",
    ]
    groupstyle = [f"vertical sep = {vertical_sep}"] if vertical_sep else []

    suptitle_artist = getattr(fig, "_suptitle", None) if strip_suptitle else None
    suptitle_was_visible = suptitle_artist.get_visible() if suptitle_artist is not None else None
    if suptitle_artist is not None:
        suptitle_artist.set_visible(False)
    try:
        matplot2tikz.save(
            str(tex_path),
            figure=fig,
            standalone=False,
            axis_width=width,
            axis_height=height,
            externalize_tables=externalize_tables,
            tex_relative_path_to_data=None,
            extra_lines_start=(extra_lines or []) + number_format_lines,
            extra_groupstyle_parameters=groupstyle,
        )
    finally:
        if suptitle_artist is not None:
            suptitle_artist.set_visible(suptitle_was_visible)

    tex_path.write_text(_fix_known_matplot2tikz_bugs(tex_path.read_text()))

    (folder / f"{name}_main.tex").write_text(
        _MAIN_TEX_TEMPLATE.format(doc_class_opts=doc_class_opts, tex_name=f"{name}.tex")
    )
    return folder


def compile_tikz_folder(root_folder: str | Path, name: str, *, force: bool = False) -> bool:
    """Compile <root_folder>/<name>/<name>_main.tex with pdflatex and copy the
    result to <root_folder>/<name>_exported_tikz.pdf -- a dependency-free
    Python equivalent of compileAllTikz.sh, with the same incremental
    skip-if-nothing-changed behavior (unless force=True). Returns True on
    success (including "already up to date").
    """
    root_folder = Path(root_folder)
    folder = root_folder / name
    main_tex = folder / f"{name}_main.tex"
    output_pdf = folder / f"{name}_main.pdf"
    final_pdf = root_folder / f"{name}_exported_tikz.pdf"

    if not main_tex.exists():
        print(f"  [tikz] {name}: {main_tex} not found, skipping")
        return False

    if not force and final_pdf.exists():
        sources = [main_tex, *folder.glob("*.dat"), *folder.glob("*.csv"), *folder.glob("*.txt")]
        if all(s.stat().st_mtime <= final_pdf.stat().st_mtime for s in sources):
            print(f"  [tikz] {name}: up to date, skipping")
            return True

    result = subprocess.run(
        ["pdflatex", "-interaction=nonstopmode", main_tex.name],
        capture_output=True,
        text=True,
        cwd=folder,
    )
    if result.returncode != 0 or not output_pdf.exists():
        print(f"  [tikz] {name}: pdflatex FAILED")
        print(result.stdout[-3000:])
        return False

    shutil.copy(output_pdf, final_pdf)
    print(f"  [tikz] {name}: compiled -> {final_pdf}")
    return True
