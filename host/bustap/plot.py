"""Event plots: every channel, thresholds, the sliced logic and run durations."""

from pathlib import Path

import numpy as np

from .analyze import EventReport
from .capture import Capture

UNITS = {"DC": "bus V", "AC": "bus swing V", "FC": "loop V"}


def plot_event(cap: Capture, rep: EventReport, path, context_s: float = 0.005, max_labels: int = 80) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = [n for n in ("DC", "AC", "FC") if n in cap.channels]
    fig, axes = plt.subplots(len(names), 1, sharex=True, figsize=(14, 2.6 * len(names) + 0.6), squeeze=False)
    t0 = rep.t_start_s
    for ax, name in zip(axes[:, 0], names):
        ch = cap.channels[name]
        i0 = max(0, int((rep.t_start_s - context_s) * ch.fs))
        i1 = min(len(ch.raw), int((rep.t_end_s + context_s) * ch.fs))
        v = cap.bus_volts(name)[i0:i1]
        t_ms = (np.arange(i0, i1) / ch.fs - t0) * 1e3
        ax.plot(t_ms, v, lw=0.7, color="0.25")
        ax.set_ylabel(f"{name}\n{UNITS.get(name, 'V')}")
        ax.grid(alpha=0.3)

        if name == rep.channel and rep.levels and rep.runs is not None:
            lv = rep.levels
            for y in (lv.threshold - lv.hysteresis / 2, lv.threshold + lv.hysteresis / 2):
                ax.axhline(y, color="tab:orange", lw=0.6, ls="--")
            starts_ms = rep.runs.starts / ch.fs * 1e3
            ends_ms = starts_ms + rep.runs.durations / ch.fs * 1e3
            ys = np.where(rep.runs.levels, lv.high, lv.low)
            ax.hlines(ys, starts_ms, ends_ms, color="tab:blue", lw=1.6, alpha=0.8)
            if len(rep.runs) <= max_labels:
                for s, e, y, d in zip(starts_ms, ends_ms, ys, rep.runs.durations_us()):
                    ax.text((s + e) / 2, y, f"{d:.0f}", fontsize=6, ha="center",
                            va="bottom" if y == lv.high else "top", color="tab:blue")

    title = f"{cap.label or 'capture'}  event {rep.index}  [{rep.source}] ch {rep.channel}"
    if rep.model and rep.model.hypotheses:
        title += f"  |  {rep.model.hypotheses[0].encoding}"
    if rep.best:
        title += f"  |  best decode {rep.best.decoder} ({rep.best.errors} errors)"
    axes[0, 0].set_title(title, fontsize=10)
    axes[-1, 0].set_xlabel("ms from event start (labels: run durations in us)")
    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path
