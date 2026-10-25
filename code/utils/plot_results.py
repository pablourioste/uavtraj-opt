"""Plot the CSV files written by the use cases.

Usage (from code/, after running the use cases):
    python utils/plot_results.py                      # output/ -> output/figures/
    python utils/plot_results.py --out ../docs/assets/figures/code
Requires numpy and matplotlib.
"""

import argparse
import csv
import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = pathlib.Path("output")
UF_COLOR, FC_COLOR = "#1f77b4", "#d62728"
plt.rcParams.update({"figure.dpi": 110, "axes.grid": True, "grid.alpha": 0.3, "font.size": 9})


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    header, data = rows[0], np.array(rows[1:], dtype=float)
    return {name: data[:, i] for i, name in enumerate(header)}


def save(fig, out, name):
    fig.tight_layout()
    fig.savefig(out / name, dpi=150)
    plt.close(fig)
    print("  ", out / name)


# ── UC-03 ───────────────────────────────────────────────────────────────────
def plot_simulation(out):
    uf_path, fc_path = ROOT / "uc03" / "uf.csv", ROOT / "uc03" / "fc.csv"
    if not uf_path.exists():
        print("skip UC-03: run data/cases/uc03_simulation.json first")
        return
    uf, fc = load(uf_path), load(fc_path)

    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].plot(uf["y"], uf["x"], color=UF_COLOR)
    ax[0].set(xlabel="East [m]", ylabel="North [m]", title="UF: roll into a 30 deg coordinated turn")
    ax[0].axis("equal")
    ax[1].plot(uf["t"], np.degrees(uf["mu"]), label="bank [deg]")
    ax[1].plot(uf["t"], -uf["z"] + uf["z"][0], label="altitude change [m]")
    ax[1].set(xlabel="t [s]", title="UF: bank and altitude")
    ax[1].legend()
    save(fig, out, "uc03_uf_turn.png")

    fig, ax = plt.subplots(3, 1, figsize=(8, 6.5), sharex=True)
    ax[0].plot(fc["t"], np.degrees(fc["de"]), color=FC_COLOR)
    ax[0].set(ylabel="delta_e [deg]", title="FC: free response to an elevator doublet")
    ax[1].plot(fc["t"], np.degrees(fc["alpha"]), label="alpha")
    ax[1].plot(fc["t"], np.degrees(fc["theta"]), label="theta")
    ax[1].set(ylabel="[deg]")
    ax[1].legend()
    ax[2].plot(fc["t"], -fc["z"], color=FC_COLOR)
    ax[2].set(xlabel="t [s]", ylabel="altitude [m]")
    save(fig, out, "uc03_fc_doublet.png")


# ── UC-04 ───────────────────────────────────────────────────────────────────
def plot_transcription(out):
    path = ROOT / "uc04" / "transcription_check.csv"
    if not path.exists():
        print("skip UC-04: run data/cases/uc04_transcription.json first")
        return
    t = load(path)
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].loglog(t["N"], t["jac_analytic_ms"], "o-", label="analytic (sparse)")
    ax[0].loglog(t["N"], t["jac_fd_ms"], "s-", label="forward differences")
    ax[0].set(xlabel="nodes N", ylabel="time per Jacobian [ms]", title="Defect Jacobian cost")
    ax[0].legend()
    ax[1].semilogy(t["N"], t["defect_cold"], "o-", label="cold start (straight line, zero controls)")
    ax[1].semilogy(t["N"], np.maximum(t["defect_trim"], 1e-17), "s-", label="trimmed straight line")
    ax[1].set(xlabel="nodes N", ylabel="max |defect| (normalised)", title="Seed quality")
    ax[1].legend()
    save(fig, out, "uc04_jacobian_and_seed.png")

    spy = sorted((ROOT / "uc04").glob("sparsity_N*.csv"))
    if spy:
        s = load(spy[0])
        fig, ax = plt.subplots(figsize=(6, 4.2))
        ax.scatter(s["col"], s["row"], s=1, marker="s")
        ax.invert_yaxis()
        ax.set(xlabel="decision variable", ylabel="defect row", title=f"Defect Jacobian sparsity ({spy[0].stem})")
        save(fig, out, "uc04_sparsity.png")


# ── UC-05 ───────────────────────────────────────────────────────────────────
def plot_design_charts(out):
    d = ROOT / "uc05"
    if not (d / "power_curves.csv").exists():
        print("skip UC-05: run data/cases/uc05_design_charts.json first")
        return
    pc, env, tr, tu = (load(d / f) for f in ("power_curves.csv", "envelope.csv", "trim_curves.csv", "turn.csv"))

    fig, ax = plt.subplots(2, 2, figsize=(11, 8))
    a = ax[0, 0]
    for i, h in enumerate(np.unique(pc["h"])):
        m = pc["h"] == h
        c = plt.cm.viridis(i / max(1, len(np.unique(pc["h"])) - 1) * 0.85)
        a.plot(pc["V"][m], pc["P_req"][m], color=c, label=f"required, h = {h:.0f} m")
        a.axhline(pc["P_av"][m][0], color=c, ls="--", lw=1)
    a.set(xlabel="airspeed [m/s]", ylabel="thrust power [W]", title="Power required (solid) vs available (dashed)")
    a.legend(fontsize=7)

    a = ax[0, 1]
    ok = env["V_max"] > 0
    a.fill_betweenx(env["h"][ok], env["V_stall"][ok], env["V_max"][ok], color="#2ca02c", alpha=0.15,
                    label="level flight possible")
    a.plot(env["V_stall"], env["h"], "k-", label="stall")
    a.plot(env["V_max"][ok], env["h"][ok], "-", color="#d62728", label="max level speed (power)")
    a.plot(env["V_best_LD"], env["h"], "--", color="#1f77b4", label="best L/D (max range)")
    a.plot(env["V_min_power"], env["h"], ":", color="#9467bd", label="min power (max endurance)")
    a.axvline(env["V_ne"][0], color="gray", ls="-.", lw=1, label="V_NE")
    a.set(xlabel="airspeed [m/s]", ylabel="altitude [m]", title="Flight envelope")
    a.legend(fontsize=7, loc="upper left")

    a = ax[1, 0]
    a.plot(tr["V"], tr["de_deg"], color=FC_COLOR, label="elevator to trim [deg]")
    a.plot(tr["V"], tr["alpha_deg"], color="#ff7f0e", label="angle of attack [deg]")
    a.axhline(-25, color="gray", ls=":", lw=1, label="elevator limit")
    a.set(xlabel="airspeed [m/s]", ylabel="[deg]", title="6-DoF trim curves (h = 300 m)")
    a2 = a.twinx()
    a2.plot(tr["V"], tr["T_fc"], color="k", lw=1, label="thrust FC [N]")
    a2.plot(tr["V"], tr["T_uf"], "w--", lw=0.8)
    a2.set_ylabel("thrust [N]")
    a2.grid(False)
    lines = a.get_lines() + a2.get_lines()[:1]
    a.legend(lines, [ln.get_label() for ln in lines], fontsize=7, loc="lower right")

    a = ax[1, 1]
    for i, V in enumerate(np.unique(tu["V"])):
        m = tu["V"] == V
        c = plt.cm.plasma(i / max(1, len(np.unique(tu["V"])) - 1) * 0.8)
        feas = tu["feasible"][m] > 0.5
        a.plot(tu["phi_deg"][m][feas], tu["radius"][m][feas], color=c, label=f"V = {V:.0f} m/s")
        a.plot(tu["phi_deg"][m][~feas], tu["radius"][m][~feas], color=c, ls=":", lw=1)
    a.set(yscale="log", xlabel="bank angle [deg]", ylabel="turn radius [m]",
          title="Level turn: radius (dotted = beyond CL, n or power limit)")
    a.legend(fontsize=7)
    save(fig, out, "uc05_design_charts.png")


# ── UC-06 ───────────────────────────────────────────────────────────────────
def plot_scenarios(out):
    d = ROOT / "uc06"
    ufs = sorted(d.glob("SC-*_uf.csv"))
    if not ufs:
        print("skip UC-06: run data/cases/uc06_design_scenarios.json first")
        return
    ids = [p.stem.replace("_uf", "") for p in ufs]
    titles = {"SC-00": "straight, level", "SC-01": "straight, climb +200 m", "SC-02": "180 deg turn, level",
              "SC-03": "climb +200 m and 180 deg turn"}

    # Overview: 3D paths of every scenario, UF vs FC.
    fig = plt.figure(figsize=(13, 4.2))
    for i, sid in enumerate(ids):
        uf, fc = load(d / f"{sid}_uf.csv"), load(d / f"{sid}_fc.csv")
        a = fig.add_subplot(1, len(ids), i + 1, projection="3d")
        a.plot(uf["y"], uf["x"], uf["h"], color=UF_COLOR, label="UF (point mass)")
        a.plot(fc["y"], fc["x"], fc["h"], color=FC_COLOR, ls="--", label="FC (6-DoF)")
        # Same horizontal scale on both axes, so straight legs are not stretched.
        span = max(np.ptp(uf["x"]), np.ptp(uf["y"]), 1.0) / 2 + 50
        cx, cy = (uf["x"].max() + uf["x"].min()) / 2, (uf["y"].max() + uf["y"].min()) / 2
        a.set_xlim(cy - span, cy + span)
        a.set_ylim(cx - span, cx + span)
        a.set_zlim(280, 520)
        a.set_title(f"{sid}: {titles.get(sid, '')}", fontsize=8)
        a.set_xlabel("E [m]", fontsize=7)
        a.set_ylabel("N [m]", fontsize=7)
        a.set_zlabel("h [m]", fontsize=7)
        a.tick_params(labelsize=6)
        if i == 0:
            a.legend(fontsize=7, loc="upper left")
    fig.subplots_adjust(left=0.01, right=0.97, wspace=0.05)
    fig.savefig(out / "uc06_scenarios_3d.png", dpi=150)
    plt.close(fig)
    print("  ", out / "uc06_scenarios_3d.png")

    # One detailed figure per scenario.
    for sid in ids:
        uf, fc = load(d / f"{sid}_uf.csv"), load(d / f"{sid}_fc.csv")
        fig, ax = plt.subplots(2, 3, figsize=(12, 6.5))
        fig.suptitle(f"{sid}: {titles.get(sid, '')} (generic UAV, closed-loop guidance)", fontsize=10)
        a = ax[0, 0]
        a.plot(uf["y"], uf["x"], color=UF_COLOR, label="UF")
        a.plot(fc["y"], fc["x"], color=FC_COLOR, ls="--", label="FC")
        a.set(xlabel="East [m]", ylabel="North [m]", title="ground track")
        a.axis("equal")
        a.legend()
        a = ax[0, 1]
        a.plot(uf["s"], uf["h"], color=UF_COLOR)
        a.plot(fc["s"], fc["h"], color=FC_COLOR, ls="--")
        a.set(xlabel="along-track distance [m]", ylabel="altitude [m]", title="altitude profile")
        a = ax[0, 2]
        a.plot(uf["t"], uf["V"], color=UF_COLOR)
        a.plot(fc["t"], fc["V"], color=FC_COLOR, ls="--")
        a.set(xlabel="t [s]", ylabel="airspeed [m/s]", title="airspeed")
        a = ax[1, 0]
        a.plot(uf["t"], uf["bank_deg"], color=UF_COLOR)
        a.plot(fc["t"], fc["bank_deg"], color=FC_COLOR, ls="--")
        a.set(xlabel="t [s]", ylabel="bank [deg]", title="bank angle")
        a = ax[1, 1]
        a.plot(uf["t"], uf["gamma_deg"], color=UF_COLOR)
        a.plot(fc["t"], fc["gamma_deg"], color=FC_COLOR, ls="--")
        a.set(xlabel="t [s]", ylabel="gamma [deg]", title="flight-path angle")
        a = ax[1, 2]
        a.plot(uf["t"], uf["T"], color=UF_COLOR, label="thrust UF")
        a.plot(fc["t"], fc["T"], color=FC_COLOR, ls="--", label="thrust FC")
        a.set(xlabel="t [s]", ylabel="thrust [N]", title="controls")
        a2 = a.twinx()
        a2.plot(fc["t"], np.degrees(fc["de"]), color="#2ca02c", lw=0.8, label="delta_e FC [deg]")
        a2.plot(fc["t"], np.degrees(fc["da"]), color="#9467bd", lw=0.8, label="delta_a FC [deg]")
        a2.set_ylabel("[deg]")
        a2.grid(False)
        lines = a.get_lines() + a2.get_lines()
        a.legend(lines, [ln.get_label() for ln in lines], fontsize=7)
        save(fig, out, f"uc06_{sid}.png")

    # Speed trade-off: flight time vs fuel for every scenario.
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for sid in ids:
        p = d / f"{sid}_speed_trade.csv"
        if not p.exists():
            continue
        t = load(p)
        ok = t["completed"] > 0.5
        ax[0].plot(t["time"][ok], 1000 * t["fuel"][ok], "o-", ms=3, label=sid)
        i = np.argmin(np.where(ok, t["fuel"], np.inf))
        ax[0].annotate(f"{t['V'][i]:.0f} m/s", (t["time"][i], 1000 * t["fuel"][i]), fontsize=7,
                       textcoords="offset points", xytext=(4, -10))
        ax[1].plot(t["V"][ok], t["fuel"][ok] / t["fuel"][ok].min(), "o-", ms=3, label=sid)
    ax[0].set(xlabel="flight time [s]", ylabel="fuel [g]",
              title="Time vs fuel when sweeping the commanded speed\n(left end = fast, labelled = least fuel)")
    ax[0].legend(fontsize=7)
    ax[1].set(xlabel="commanded airspeed [m/s]", ylabel="fuel / minimum fuel [-]",
              title="Fuel penalty of flying faster")
    ax[1].legend(fontsize=7)
    save(fig, out, "uc06_speed_trade.png")


# ── UC-07 ───────────────────────────────────────────────────────────────────
def _signed_distance(px, py, poly):
    """Distance to a polygon boundary, positive outside (vectorised; poly = [[north, east], ...])."""
    px, py = np.asarray(px, float), np.asarray(py, float)
    d = np.full(px.shape, np.inf)
    inside = np.zeros(px.shape, bool)
    n = len(poly)
    for i in range(n):
        ax_, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        vx, vy = bx - ax_, by - ay
        t = np.clip(((px - ax_) * vx + (py - ay) * vy) / (vx * vx + vy * vy), 0.0, 1.0)
        d = np.minimum(d, np.hypot(px - (ax_ + t * vx), py - (ay + t * vy)))
        cross = ((ay > py) != (by > py)) & (px < (bx - ax_) * (py - ay) / np.where(by - ay == 0, 1e-12, by - ay) + ax_)
        inside ^= cross
    return np.where(inside, -d, d)


def _zones(d, mid):
    z = load(d / f"{mid}_nfz.csv") if (d / f"{mid}_nfz.csv").stat().st_size > 60 else None
    if z is None:
        return []
    out = []
    for k in np.unique(z["zone"]):
        m = z["zone"] == k
        out.append({"poly": np.column_stack([z["x"][m], z["y"][m]]), "margin": z["margin"][m][0],
                    "active": z["active"][m][0] > 0.5, "h": (z["h_min"][m][0], z["h_max"][m][0])})
    return out


def _clearance(zones, x, y):
    act = [z for z in zones if z["active"]]
    if not act:
        return None
    return np.min([_signed_distance(x, y, z["poly"]) - z["margin"] for z in act], axis=0)


def _draw_zones(a, zones):
    for z in zones:
        poly = np.vstack([z["poly"], z["poly"][:1]])
        if z["active"]:
            a.fill(poly[:, 1], poly[:, 0], color="#d62728", alpha=0.25, lw=0)
            a.plot(poly[:, 1], poly[:, 0], color="#d62728", lw=1)
            # Exact margin contour: signed distance = margin.
            xs = np.linspace(poly[:, 0].min() - 2 * z["margin"], poly[:, 0].max() + 2 * z["margin"], 160)
            ys = np.linspace(poly[:, 1].min() - 2 * z["margin"], poly[:, 1].max() + 2 * z["margin"], 160)
            X, Y = np.meshgrid(xs, ys, indexing="ij")
            a.contour(Y, X, _signed_distance(X, Y, z["poly"]), levels=[z["margin"]], colors="#d62728",
                      linestyles=":", linewidths=0.8)
        else:
            a.fill(poly[:, 1], poly[:, 0], facecolor="none", edgecolor="gray", hatch="//", lw=0.8)
            a.annotate(f"overflown\n(top {z['h'][1]:.0f} m)", (poly[:, 1].mean(), poly[:, 0].mean()), fontsize=6,
                       ha="center", va="center", color="gray")


def _draw_points(a, pts):
    s, e = (pts["x"][0], pts["y"][0]), (pts["x"][-1], pts["y"][-1])
    for i in range(1, len(pts["x"]) - 1):
        a.plot(pts["y"][i], pts["x"][i], "D", color="#2ca02c", ms=5)
        a.add_patch(plt.Circle((pts["y"][i], pts["x"][i]), pts["tolerance"][i], fill=False, color="#2ca02c", lw=0.8))
    a.plot(s[1], s[0], "^", color="k", ms=6)
    a.plot(e[1], e[0], "s", color="k", ms=5)
    a.set(xlabel="East [m]", ylabel="North [m]")
    a.set_aspect("equal", adjustable="datalim")


def _top_view(a, d, mid, zones, legend=True):
    pts = load(d / f"{mid}_points.csv")
    path, uf, fc = load(d / f"{mid}_path.csv"), load(d / f"{mid}_uf.csv"), load(d / f"{mid}_fc.csv")
    _draw_zones(a, zones)
    s, e = (pts["x"][0], pts["y"][0]), (pts["x"][-1], pts["y"][-1])
    a.plot([s[1], e[1]], [s[0], e[0]], ":", color="gray", lw=1, label="straight line (cold start)")
    a.plot(path["y"], path["x"], color="k", lw=0.8, label="Hybrid A* path")
    a.plot(uf["y"], uf["x"], color=UF_COLOR, lw=1.4, label="UF flown")
    a.plot(fc["y"], fc["x"], color=FC_COLOR, ls="--", lw=1.2, label="FC flown")
    _draw_points(a, pts)
    if legend:
        a.legend(fontsize=6, loc="lower right")


def plot_missions(out):
    d = ROOT / "uc07"
    paths = sorted(d.glob("M-*_path.csv"))
    if not paths:
        print("skip UC-07: run data/cases/uc07_nfz_missions.json first")
        return
    ids = [p.stem.replace("_path", "") for p in paths]

    fig, ax = plt.subplots(2, 2, figsize=(11, 10))
    for a, mid in zip(ax.flat, ids):
        zones = _zones(d, mid)
        _top_view(a, d, mid, zones, legend=(mid == ids[0]))
        a.set_title(mid, fontsize=9)
    fig.suptitle("Missions with no-fly zones and waypoints: straight line (dotted) crosses the zones, "
                 "the planned and flown paths do not", fontsize=9)
    save(fig, out, "uc07_missions_overview.png")

    # Cover: the four missions side by side with one shared legend.
    titles = {"M-01": "M-01 · one no-fly zone", "M-02": "M-02 · slalom between two zones",
              "M-03": "M-03 · two waypoints", "M-04": "M-04 · zone + waypoint + climb\n(low zone overflown)"}
    fig, ax = plt.subplots(1, len(ids), figsize=(16, 5.6))
    for a, mid in zip(np.atleast_1d(ax), ids):
        _top_view(a, d, mid, _zones(d, mid), legend=False)
        a.set_title(titles.get(mid, mid), fontsize=10)
        a.tick_params(labelsize=7)
    handles = [plt.Line2D([], [], color="gray", ls=":", label="straight line (cold start)"),
               plt.Line2D([], [], color="k", lw=0.8, label="Hybrid A* path"),
               plt.Line2D([], [], color=UF_COLOR, lw=1.6, label="point mass (UF), flown"),
               plt.Line2D([], [], color=FC_COLOR, ls="--", lw=1.4, label="6-DoF (FC), flown"),
               plt.matplotlib.patches.Patch(facecolor="#d62728", alpha=0.3, edgecolor="#d62728",
                                            label="no-fly zone (margin dotted)"),
               plt.matplotlib.patches.Patch(facecolor="none", edgecolor="gray", hatch="//",
                                            label="zone below flight altitude"),
               plt.Line2D([], [], color="#2ca02c", marker="D", ls="none", label="waypoint")]
    fig.legend(handles=handles, loc="lower center", ncol=7, fontsize=8, frameon=False)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(out / "uc07_cover.png", dpi=150)
    plt.close(fig)
    print("  ", out / "uc07_cover.png")

    for mid in ids:
        zones = _zones(d, mid)
        uf, fc = load(d / f"{mid}_uf.csv"), load(d / f"{mid}_fc.csv")
        fig = plt.figure(figsize=(12, 6.5))
        gs = fig.add_gridspec(3, 2, width_ratios=[1.25, 1])
        _top_view(fig.add_subplot(gs[:, 0]), d, mid, zones)
        a = fig.add_subplot(gs[0, 1])
        a.plot(uf["s"], uf["h"], color=UF_COLOR, label="UF")
        a.plot(fc["s"], fc["h"], color=FC_COLOR, ls="--", label="FC")
        a.set(ylabel="altitude [m]", title=f"{mid}: profiles along the path")
        a.legend(fontsize=7)
        a = fig.add_subplot(gs[1, 1])
        a.plot(uf["s"], uf["bank_deg"], color=UF_COLOR)
        a.plot(fc["s"], fc["bank_deg"], color=FC_COLOR, ls="--")
        a.set(ylabel="bank [deg]")
        a = fig.add_subplot(gs[2, 1])
        cu, cf = _clearance(zones, uf["x"], uf["y"]), _clearance(zones, fc["x"], fc["y"])
        if cu is None:
            a.text(0.5, 0.5, "no active no-fly zone", ha="center", va="center", transform=a.transAxes, color="gray")
        else:
            a.plot(uf["s"], cu, color=UF_COLOR)
            a.plot(fc["s"], cf, color=FC_COLOR, ls="--")
            a.axhline(0.0, color="#d62728", lw=1, label="margin")
            a.set_ylim(-20, min(400, max(cu.max(), cf.max())))
            a.legend(fontsize=7)
        a.set(xlabel="along-path distance [m]", ylabel="NFZ clearance [m]")
        save(fig, out, f"uc07_{mid}.png")


# ── UC-08 ───────────────────────────────────────────────────────────────────
WS_COLOR = "#7f7f7f"


def _optimal_top_view(a, d, mid, legend=True):
    zones, pts = _zones(d, mid), load(d / f"{mid}_points.csv")
    ws, uf, fc = load(d / f"{mid}_ws.csv"), load(d / f"{mid}_uf.csv"), load(d / f"{mid}_fc.csv")
    _draw_zones(a, zones)
    if (d / f"{mid}_path.csv").exists():
        path = load(d / f"{mid}_path.csv")
        a.plot(path["y"], path["x"], color="k", lw=0.6, alpha=0.6, label="Hybrid A* path")
    a.plot(ws["y"], ws["x"], color=WS_COLOR, ls="-.", lw=1.0, label="warm start (flown)")
    a.plot(uf["y"], uf["x"], color=UF_COLOR, lw=1.6, label="UF optimum")
    a.plot(fc["y"], fc["x"], color=FC_COLOR, ls="--", lw=1.3, label="FC optimum")
    _draw_points(a, pts)
    if legend:
        a.legend(fontsize=6, loc="lower right")
    return uf, fc


def plot_optimal(out):
    d = ROOT / "uc08"
    ids = [p.stem.replace("_uf", "") for p in sorted(d.glob("M-*_uf.csv"))]
    if not ids:
        print("skip UC-08: run data/cases/uc08_optimal_missions.json with an IPOPT build first")
        return

    fig, ax = plt.subplots(1, len(ids), figsize=(4.0 * len(ids), 5.6))
    for a, mid in zip(np.atleast_1d(ax), ids):
        uf, fc = _optimal_top_view(a, d, mid, legend=False)
        fuel = lambda t: 1e3 * (t["m"][0] - t["m"][-1])  # noqa: E731
        a.set_title(f"{mid}: UF {uf['t'][-1]:.0f} s, {fuel(uf):.1f} g · FC {fc['t'][-1]:.0f} s, {fuel(fc):.1f} g",
                    fontsize=8)
        a.tick_params(labelsize=7)
    handles = [plt.Line2D([], [], color="k", lw=0.6, alpha=0.6, label="Hybrid A* path"),
               plt.Line2D([], [], color=WS_COLOR, ls="-.", label="warm start (flown by the guidance)"),
               plt.Line2D([], [], color=UF_COLOR, lw=1.6, label="point-mass optimum (UF)"),
               plt.Line2D([], [], color=FC_COLOR, ls="--", lw=1.3, label="6-DoF optimum (FC)"),
               plt.matplotlib.patches.Patch(facecolor="#d62728", alpha=0.3, edgecolor="#d62728",
                                            label="no-fly zone (margin dotted)"),
               plt.Line2D([], [], color="#2ca02c", marker="D", ls="none", label="waypoint")]
    fig.legend(handles=handles, loc="lower center", ncol=6, fontsize=8, frameon=False)
    fig.suptitle("Minimum-fuel trajectories optimised with IPOPT: warm start -> point mass -> 6-DoF", fontsize=10)
    fig.tight_layout(rect=(0, 0.06, 1, 0.96))
    fig.savefig(out / "uc08_cover.png", dpi=150)
    plt.close(fig)
    print("  ", out / "uc08_cover.png")

    for mid in ids:
        fig = plt.figure(figsize=(12, 7.5))
        gs = fig.add_gridspec(4, 2, width_ratios=[1.2, 1])
        uf, fc = _optimal_top_view(fig.add_subplot(gs[:, 0]), d, mid)
        ws = load(d / f"{mid}_ws.csv")
        rows = [("altitude [m]", "h", "h", "h"), ("airspeed [m/s]", "V", "V", "Va"),
                ("bank [deg]", "bank_deg", "bank_deg", "phi_deg"), ("thrust [N]", "T", "T", "T")]
        for i, (label, kws, kuf, kfc) in enumerate(rows):
            a = fig.add_subplot(gs[i, 1])
            a.plot(ws["t"], ws[kws], color=WS_COLOR, ls="-.", lw=1.0, label="warm start")
            a.plot(uf["t"], uf[kuf], color=UF_COLOR, lw=1.4, label="UF optimum")
            a.plot(fc["t"], fc[kfc], color=FC_COLOR, ls="--", lw=1.2, label="FC optimum")
            a.set(ylabel=label)
            if i == 0:
                a.set_title(f"{mid}: profiles in time", fontsize=9)
                a.legend(fontsize=7)
            if i == len(rows) - 1:
                a.set_xlabel("t [s]")
        save(fig, out, f"uc08_{mid}.png")


# ── UC-09 ───────────────────────────────────────────────────────────────────
def plot_benchmark(out):
    path = ROOT / "uc09" / "benchmark.json"
    if not path.exists():
        print("skip UC-09: run data/cases/uc09_warmstart_benchmark.json with an IPOPT build first")
        return
    with open(path) as f:
        rec = json.load(f)
    missions = sorted({r["mission"] for r in rec})
    get = {(r["mission"], r["pipeline"]): r for r in rec}
    x = np.arange(len(missions))

    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    # UF stage from a cold and from a warm start.
    for i, (lab, col) in enumerate([("COLD>UF", "#bbbbbb"), ("WS>UF", UF_COLOR)]):
        it = [get[(m, lab)]["uf"]["iterations"] if (m, lab) in get else 0 for m in missions]
        ax[0].bar(x + (i - 0.5) * 0.38, it, 0.38, color=col, label=lab.replace(">", " > "))
    ax[0].set(xticks=x, xticklabels=missions, ylabel="IPOPT iterations", title="UF stage: cold vs warm start")
    ax[0].legend(fontsize=8)
    # Objective of the UF optimum relative to the warm-started one.
    rel = [100.0 * (get[(m, "COLD>UF")]["uf"]["objective"] / get[(m, "WS>UF")]["uf"]["objective"] - 1.0)
           if (m, "COLD>UF") in get and (m, "WS>UF") in get else 0.0 for m in missions]
    ax[1].bar(x, rel, 0.5, color="#bbbbbb")
    ax[1].axhline(0.0, color="k", lw=0.8)
    ax[1].set(xticks=x, xticklabels=missions, ylabel="cost increase [%]",
              title="UF optimum reached from a cold start\n(relative to the warm-started optimum)")
    # FC stage seeded directly or through the UF optimum.
    for i, (lab, col) in enumerate([("WS>FC", "#e8a0a0"), ("WS>UF>FC", FC_COLOR)]):
        ms = [get[(m, lab)]["fc"]["millis"] if (m, lab) in get else 0 for m in missions]
        ax[2].bar(x + (i - 0.5) * 0.38, ms, 0.38, color=col, label=lab.replace(">", " > "))
    ax[2].set(xticks=x, xticklabels=missions, ylabel="FC solve time [ms]",
              title="FC stage: seeded by the flown warm start\nor by the UF optimum (machine-dependent)")
    ax[2].legend(fontsize=8)
    save(fig, out, "uc09_benchmark.png")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(ROOT / "figures"), help="directory for the PNG files")
    args = parser.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for fn in (plot_simulation, plot_transcription, plot_design_charts, plot_scenarios, plot_missions, plot_optimal,
               plot_benchmark):
        fn(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
