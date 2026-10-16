"""Plot the CSV files written by the use cases.

Usage (from code/, after running the use cases):
    python utils/plot_results.py                      # output/ -> output/figures/
    python utils/plot_results.py --out ../docs/assets/figures/code
Requires numpy and matplotlib.
"""

import argparse
import csv
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(ROOT / "figures"), help="directory for the PNG files")
    args = parser.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for fn in (plot_simulation, plot_transcription, plot_design_charts, plot_scenarios):
        fn(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
