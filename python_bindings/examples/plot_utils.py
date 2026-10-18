"""
Small shared matplotlib layout helpers for this folder's example scripts.
"""


def legend_above(ax, *, ncol=2, fontsize=8, y=1.03):
    """Place `ax`'s legend just above its own bounding box (in axes-relative
    coordinates, y=1 is the axes' top edge) instead of inside it.

    In-axes legends (`ax.legend(loc=...)`, anchored inside the box) are sized
    by their content (fontsize * number of entries/columns) independent of
    the axis's own height -- fine for a tall single-axis figure, but for a
    stack of short `sharex=True` subplots (each panel's height is only a
    fraction of the figure's, to keep the overall figure at a sane aspect
    ratio once exported -- see tikz_utils._infer_axis_height) the legend box
    can end up larger than the panel and overlap the data.

    Anchoring above the axes via `bbox_to_anchor` with y>1 sidesteps this:
    the legend is a coordinate offset from the axes box, not constrained to
    fit inside it, so it never overlaps the data regardless of panel height.

    Deliberately NOT a figure-level `fig.legend()`: matplot2tikz does not
    export those at all (only per-axes legends), so a fig.legend() placed
    above the whole subplot stack would show correctly in the PNG preview
    but silently vanish from the tikz/pgfplots export -- confirmed by
    actually exporting one and finding the legend missing from the output.
    Staying a per-axes legend keeps it exportable while still avoiding the
    overlap.

    Caller is still responsible for reserving enough top margin in its own
    `fig.tight_layout(rect=[0, 0, 1, top])` call for the legend (and any
    suptitle) to have room above the axes grid.
    """
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, y), ncol=ncol, fontsize=fontsize)
