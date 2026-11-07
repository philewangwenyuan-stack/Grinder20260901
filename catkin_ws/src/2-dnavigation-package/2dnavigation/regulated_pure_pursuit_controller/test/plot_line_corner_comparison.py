"""Plot production-RPP closed-loop fixture CSVs; not a hardware trajectory test.

Python 3.8+, numpy, matplotlib. Model assumptions must accompany any exported
figure. The nearest-polyline distance is geometric, not a coverage metric.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def distances(xy, path):
    result = np.full(len(xy), np.inf)
    for a, b in zip(path[:-1], path[1:]):
        d = b-a
        if np.dot(d, d) < 1e-12:
            continue
        f = np.clip((xy-a) @ d / np.dot(d, d), 0, 1)
        result = np.minimum(result, np.linalg.norm(xy-(a+f[:, None]*d), axis=1))
    return result


def rotation_starts(data):
    return np.flatnonzero((data[:, 9] > .5) & np.r_[True, data[:-1, 9] < .5])


def metrics(data, path):
    error = distances(data[:, 1:3], path)
    turns = []
    events = rotation_starts(data)
    for i in range(1, len(path)-1):
        incoming = path[i]-path[i-1]
        outgoing = path[i+1]-path[i]
        if min(np.linalg.norm(incoming), np.linalg.norm(outgoing)) < 1e-6:
            continue
        angle = np.arctan2(incoming[0]*outgoing[1]-incoming[1]*outgoing[0], incoming @ outgoing)
        if abs(angle) <= .45:
            continue
        nearby = [k for k in events if np.linalg.norm(data[k, 1:3]-path[i]) < .45]
        if not nearby:
            continue
        k = min(nearby, key=lambda k: np.linalg.norm(data[k, 1:3]-path[i]))
        remaining = (path[i]-data[k, 1:3]) @ incoming/np.linalg.norm(incoming)
        turns.append({"vertex": i, "remaining_m": float(remaining), "t": float(data[k, 0])})
    return {"duration_s": float(data[-1, 0]), "max_path_error_m": float(error.max()),
            "rms_path_error_m": float(np.sqrt(np.mean(error**2))),
            "rotation_starts": turns}


def plot(path, old, new, output, model="First-order response tau=0.2 s (assumption)"):
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), constrained_layout=True)
    colors = ["#df8230", "#1769ac"]
    for ax in axes[0]:
        ax.plot(path[:, 0], path[:, 1], "k--", lw=1.3, label="Planned path")
        for data, label, color in [(old, "Legacy", colors[0]), (new, "Line approach", colors[1])]:
            ax.plot(data[:, 1], data[:, 2], color=color, lw=1.5, label=label)
            ids = rotation_starts(data)
            ax.scatter(data[ids, 1], data[ids, 2], s=30, marker="o", facecolors="none", edgecolors=color)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.2)
        ax.set_xlabel("Map x (m)"); ax.set_ylabel("Map y (m)")
    axes[0, 0].set_title("Same recorded path | circles: rotation entry")
    axes[0, 0].legend(loc="best")
    if len(path)>2:
        c=path[1]
        axes[0, 1].set_xlim(c[0]-.55,c[0]+.55)
        axes[0, 1].set_ylim(c[1]-.55,c[1]+.55)
        axes[0, 1].set_aspect("equal", adjustable="box")
    axes[0, 1].set_title("First corner detail")
    for data, label, color in [(old,"Legacy",colors[0]),(new,"Line approach",colors[1])]:
        axes[1, 0].plot(data[:, 0], distances(data[:, 1:3],path)*100,color=color,label=label)
        axes[1, 1].plot(data[:, 0],data[:, 7],color=color,label=label)
    axes[1, 0].set_ylabel("Nearest-path distance (cm)")
    axes[1, 1].set_ylabel("Command angular velocity (rad/s)")
    for ax in axes[1]:
        ax.set_xlabel("Simulation time (s)"); ax.grid(alpha=.2);ax.legend()
    fig.suptitle("Production RPP: legacy vs line-based corner approach\n"+model+" | NOT a field measurement\nv=0.03 m/s, max w=0.10 rad/s; 20 Hz control / 10 Hz odom; initial offset=6 cm, yaw=0.03 rad",fontsize=13)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(str(output),dpi=180)
    plt.close(fig)
    report={"model":model,"legacy":metrics(old,path),"line":metrics(new,path)}
    output.with_suffix(".json").write_text(json.dumps(report,indent=2),encoding="utf8")
    return report


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path",required=True)
    parser.add_argument("--legacy",required=True)
    parser.add_argument("--line",required=True)
    parser.add_argument("--output",required=True)
    parser.add_argument("--model-label",default="First-order response tau=0.2 s (assumption)")
    args=parser.parse_args()
    load=lambda name: np.loadtxt(name,delimiter=",",skiprows=1)
    report=plot(load(args.path),load(args.legacy),load(args.line),args.output,args.model_label)
    print(json.dumps(report,indent=2))
