"""mission_controller.py — S02 full mission controller (this session's own design).

Hardware contract (read from BUILDARENA_MACHINE_BSG):
  root Starting Block on the recovery stack, two big fuel tanks below it (upper tank has
  two retro-facing boosters on its top edge faces, lower tank has four main boosters on its
  bottom inner faces), four reaction wheels, four landing legs, and above the root a
  Decoupler carrying the inert payload (five zero-drag wooden poles).

Mission profile ("vertical pop-up, apogee insertion, retro-kill, pad landing"):
  pad            engines off, settle, record launch reference
  ascent_vert    main engines full, nose up, until vertical speed reaches V_CUT
  ascent_coast   engines off, slew nose toward prograde while coasting to apogee
  ascent_insert  main engines, hold altitude and build horizontal speed to local circular
  sep            fire the decoupler; payload becomes independent
  retro_kill     fire the two retro boosters (nose still prograde) until horizontal speed
                 is killed; the stack then falls almost straight down over the pad
  hover_trim     brake to a hover, translate over the pad, kill the lateral speed (all lateral
  descent        free fall to the ignition altitude
  final_descent  PI throttle on sink rate + lateral feedback, engine cutoff at contact
  touchdown      static settling (10 s)
  wait_payload   keep simulation and telemetry running until the payload completes 3
                 unpowered revolutions (1080 deg) -- the Evaluation Endpoint requirement
  done

Outputs (run directory): control_log.csv, mission_summary.json
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))

from controller_sdk import ControllerClient, targets_by_guid  # noqa: E402
from controller_sdk.protocol import ENV_MACHINE_BSG, ENV_RUN_DIR  # noqa: E402

MU = 8456800.0
R_SURFACE = 600.0
PLANET_CENTER = np.array([0.0, -600.0, 0.0])
G_MAX = 22.0
HARD_R = 1100.0

BID_STARTING = 0
BID_DECOUPLER = 4
BID_BOOSTER = 90
BID_TANK_BIG = 99
BID_REACTION_WHEEL = 101
BID_WOODEN_POLE = 41
BID_DOUBLE_WOOD = 1

# public build weights; the big tank is listed at its empty mass
BLOCK_MASS = {0: 0.25, 1: 0.5, 4: 0.5, 41: 0.5, 90: 1.0, 99: 1.5, 101: 0.25}
FUEL_UNIT_MASS = 0.035
ENGINE_FORCE_U1 = 210.0

CFG = {
    "pad_time": 0.5,
    "wheel_speed": 0.5,
    "wheel_deadband": 0.03,
    "wheel_damper": 0.5,
    "kp_att": 1.1,
    "kd_att": 3.2,
    "att_lead": 0.35,
    "throttle_max": 2.0,
    # ascent: cut the vertical burn when the ballistic apogee reaches this radius
    "r_apogee_target": 1400.0,   # alt 800: payload rp~1400 -> 3 revs ~307 s (return altitude is now covered by the hover brake)   # alt 1050 pop-up apogee: the return needs ~850 m of altitude
    "ascent_slew_deg_s": 15.0,
    "insert_align_gate": 0.50,
    "insert_kr": 2.0,
    "insert_kt": 6.0,
    "rp_target": 1280.0,        # payload periapsis target (separation is prograde, rp = sep radius)
    "insert_timeout": 12.0,
    "insert_alt_min": 380.0,
    # separation / retro
    "sep_fire_hold": 0.20,
    "retro_throttle": 2.0,
    "retro_vt_stop": 1.0,
    "retro_pad_tilt": 0.16,
    "retro_up_mix": 0.55,
    "retro_timeout": 9.0,
    "retro_align_gate": 0.50,
    # descent / landing
    "reorient_time_max": 8.0,
    # above-the-pad hover translation (the only place lateral control is reliable)
    "hover_kv": 2.2,            # vertical rate gain while hovering
    "hover_lead": 12.0,
    "hover_kp_pos": 0.022,
    "hover_kd_vel": 0.30,          # s: horizon used to turn the offset into a lateral velocity
    "hover_vmax": 6.0,          # m/s cap on the lateral translation speed
    "hover_tilt": 0.80,         # tan(tilt): max lateral accel / g during the hover
    "hover_tol_off": 12.0,       # m: offset considered trimmed
    "hover_tol_vh": 1.2,        # m/s: lateral speed considered trimmed
    "hover_time": 60.0,
    "hover_slew_deg_s": 9.0,
    "term_lat_alt": 55.0,
    "term_lat_kv": 1.6,
    "term_lat_max": 6.0,         # s: hard cap on the hover phase
    "hover_fuel_floor": 45.0,   # fuel units: leave enough for the powered descent
    "trim_time": 4.0,   # unused (kept for log compatibility)
    "ign_margin": 6.0,
    "ign_floor": 40.0,
    "terminal_kp": 1.7,
    "terminal_ki": 0.22,
    "terminal_int_max": 1.5,
    "sink_k": 0.55,
    "sink_min": 0.5,
    "sink_max": 38.0,
    "thrust_filter": 0.6,
    "touch_speed": 1.2,
    "settle_time": 10.5,
    "land_lat_gain_p": 0.05,
    "land_lat_gain_d": 1.1,
    # payload gate
    "payload_gate_deg": 1085.0,
    "payload_timeout": 620.0,
    # abort
    "abort_low_energy_t": 16.0,
    "abort_low_energy_alt": 40.0,
    "abort_vr_dive": -70.0,
    "abort_fuel_frac": 0.005,
}


def unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def qmat(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def elements(pos, vel):
    rel = pos - PLANET_CENTER
    r = float(np.linalg.norm(rel))
    v2 = float(vel @ vel)
    eps = v2 / 2.0 - MU / r
    h_vec = np.cross(rel, vel)
    h2 = float(h_vec @ h_vec)
    e_vec = ((v2 - MU / r) * rel - float(rel @ vel) * vel) / MU
    e = float(np.linalg.norm(e_vec))
    rp = h2 / (MU * (1.0 + e))
    ra = h2 / (MU * (1.0 - e)) if e < 1.0 else float("inf")
    a = -MU / (2.0 * eps) if eps < 0 else float("inf")
    return dict(r=r, eps=eps, h_vec=h_vec, h2=h2, e=e, e_vec=e_vec, rp=rp, ra=ra, a=a)


def slew_dir(current, target, max_step):
    delta = target - current
    n = float(np.linalg.norm(delta))
    if n <= max_step or n < 1e-9:
        return target.copy()
    return current + delta * (max_step / n)


def load_machine(bsg_path: Path):
    root = ET.parse(str(bsg_path)).getroot()
    blocks = []
    for node in root.findall("Blocks/Block"):
        tr = node.find("Transform")
        pos = [float(tr.find("Position").get(k, 0.0)) for k in ("x", "y", "z")]
        rot = [float(tr.find("Rotation").get(k, 0.0)) for k in ("x", "y", "z", "w")]
        blocks.append(dict(guid=str(node.get("guid", "")).lower(),
                           bid=int(node.get("id", "-1")), pos=pos, rot=rot))
    by_bid = {}
    for b in blocks:
        by_bid.setdefault(b["bid"], []).append(b)
    decouplers = by_bid.get(BID_DECOUPLER, [])
    cut_y = max((d["pos"][1] for d in decouplers), default=1e9)
    payload, recovery = [], []
    for b in blocks:
        if b["bid"] == BID_DECOUPLER:
            continue
        if b["pos"][1] > cut_y + 0.02:
            payload.append(b)
        else:
            recovery.append(b)
    # main boosters sit under the lower tank (low in the machine frame);
    # retro boosters sit on the upper tank top edge (high in the machine frame)
    boosters = by_bid.get(BID_BOOSTER, [])
    y_sorted = sorted(b["pos"][1] for b in boosters)
    retro_cut = (y_sorted[0] + y_sorted[-1]) / 2.0 if len(boosters) > 1 else -1e9
    retro = [b for b in boosters if b["pos"][1] > retro_cut]
    main = [b for b in boosters if b["pos"][1] <= retro_cut]
    dry_payload = sum(BLOCK_MASS.get(b["bid"], 0.5) for b in payload)
    dry_recovery = sum(BLOCK_MASS.get(b["bid"], 0.5) for b in recovery) + 0.5 * len(decouplers)
    return dict(
        root=by_bid.get(BID_STARTING, [{}])[0],
        boosters=[b["guid"] for b in main],
        retro=[b["guid"] for b in retro],
        tanks=[b["guid"] for b in by_bid.get(BID_TANK_BIG, [])],
        wheels=[b["guid"] for b in by_bid.get(BID_REACTION_WHEEL, [])],
        decouplers=[b["guid"] for b in decouplers],
        payload=[b["guid"] for b in payload],
        recovery=[b["guid"] for b in recovery] + [b["guid"] for b in decouplers],
        dry_payload=dry_payload,
        dry_recovery=dry_recovery,
        dry_total=dry_payload + dry_recovery,
        tank_fuel_capacity=100.0 * len(by_bid.get(BID_TANK_BIG, [])),
        wheel_axis_local={b["guid"]: unit(qmat(b["rot"]) @ np.array([0.0, 0.0, 1.0]))
                          for b in by_bid.get(BID_REACTION_WHEEL, [])},
        saved_rot={b["guid"]: np.array(b["rot"]) for b in blocks},
    )


class Mission:
    def __init__(self, hw, client, run_dir: Path):
        self.hw = hw
        self.c = client
        self.run_dir = run_dir
        self.rows = []
        self.events = []
        self.warnings = []
        self.phase = "pad"
        self.t0 = None
        self.last_t = 0.0
        self.dt = 0.04
        self.abort = ""
        self.ref_dir = None
        self.decoupler_fired = False
        # blocks that leave the recovery machine at separation (decoupler rides with the payload)
        self.detached = set()
        self.hover_ref = None
        self.sep_t = None
        self.retro_start = None
        self.insert_start = None
        self.trim_start = None
        self.ignite_start = None
        self.touch_t = None
        self.touch_pos = None
        self.payload_sweep = 0.0
        self.payload_prev_n = None
        self.payload_rmin = 1e18
        self.payload = {}
        self.payload_missing = False
        self.fuel_total_full = hw["tank_fuel_capacity"]
        self.integrator = 0.0
        self.launch_n = None
        self.pos0 = None
        self.h_ref = None
        self.vr_prev = 0.0
        self.insert_elements = None
        # strict G1 bookkeeping: last time any engine was lit, and the payload sweep at that time
        self.powered_now = False
        self.last_thrust_sim_t = None
        self.last_thrust_t = None
        self.thrust_end_sweep = 0.0
        self.u_filtered = 0.0

    # -- logging -------------------------------------------------------------
    def event(self, name, **kw):
        rec = {"t": round(self.last_t, 3), "event": name}
        rec.update(kw)
        self.events.append(rec)
        print(f"[t={self.last_t:7.2f}] EVENT {name} {kw}", flush=True)

    def warn(self, msg):
        self.warnings.append({"t": round(self.last_t, 3), "warning": msg})
        print(f"[t={self.last_t:7.2f}] WARN {msg}", flush=True)

    # -- state ---------------------------------------------------------------
    def read_state(self, frame):
        targets = targets_by_guid(frame, block_guids=self.c._block_guids)
        hw = self.hw
        rec_pos, rec_vel, missing = [], [], []
        for g in hw["recovery"]:
            row = targets.get(g)
            if row is None:
                missing.append(g)
                continue
            if g in self.detached:
                # the decoupler leaves with the payload: including it would drag the state
                # estimate (alt/position) toward the payload after separation
                continue
            rec_pos.append(row["position"])
            rec_vel.append(row["velocity"])
        if not rec_pos:
            raise RuntimeError("no recovery blocks in telemetry")
        pos = np.mean(np.array(rec_pos), axis=0)
        vel = np.mean(np.array(rec_vel), axis=0)
        root = targets.get(hw["root"]["guid"])
        if root is None:
            raise RuntimeError("root block missing from telemetry")
        rot = qmat(root["rotation"]) @ qmat(hw["saved_rot"][hw["root"]["guid"]]).T
        omega = np.array(root["angular_velocity"])
        rel = pos - PLANET_CENTER
        r = float(np.linalg.norm(rel))
        n = rel / r
        alt = r - R_SURFACE
        vr = float(vel @ n)
        vt = float(np.linalg.norm(vel - vr * n))
        fuel = 0.0
        for g in hw["tanks"]:
            row = targets.get(g)
            if row is not None and row.get("fuel") is not None:
                fuel += float(row["fuel"][0])
        payload = {}
        pos_list, vel_list = [], []
        for g in hw["payload"]:
            row = targets.get(g)
            if row is not None:
                payload[g] = row
                pos_list.append(row["position"])
                vel_list.append(row["velocity"])
        if pos_list:
            ppos = np.mean(np.array(pos_list), axis=0)
            pvel = np.mean(np.array(vel_list), axis=0)
            p_rel = ppos - PLANET_CENTER
            pr = float(np.linalg.norm(p_rel))
            pn = p_rel / pr
            pvr = float(pvel @ pn)
            pvt = float(np.linalg.norm(pvel - pvr * pn))
            payload.update(pos=ppos, vel=pvel, r=pr, alt=pr - R_SURFACE,
                           vr=pvr, vt=pvt, n=pn, count=len(pos_list))
        return dict(targets=targets, pos=pos, vel=vel, rot=rot, omega=omega, r=r, n=n,
                    alt=alt, vr=vr, vt=vt, speed=float(np.linalg.norm(vel)),
                    fuel=fuel, payload=payload, missing=missing,
                    alive=getattr(frame.machine, "alive_block_count", None),
                    integrity=getattr(frame.machine, "machine_integrity", None))

    # -- helpers -------------------------------------------------------------
    def accel_u1(self, st, engines=None):
        n_eng = len(engines) if engines is not None else len(self.hw["boosters"])
        mass = self.hw["dry_total"] + FUEL_UNIT_MASS * st["fuel"]
        return n_eng * ENGINE_FORCE_U1 / max(mass, 1e-6)

    def horizontal(self, st):
        n = st["n"]
        off = st["pos"] - PLANET_CENTER
        off_h = off - float(off @ n) * n
        v_h = st["vel"] - float(st["vel"] @ n) * n
        return off_h, v_h

    def offset_from_start(self, st):
        """Horizontal offset of the recovery stack from its launch-pad position."""
        if self.pos0 is None:
            return np.zeros(3)
        n = st["n"]
        delta = st["pos"] - self.pos0
        return delta - float(delta @ n) * n

    def prograde_dir(self, st):
        vt_vec = st["vel"] - st["vr"] * st["n"]
        if np.linalg.norm(vt_vec) > 2.0:
            return unit(vt_vec)
        h_hat = unit(np.cross(st["pos"] - PLANET_CENTER, st["vel"])) \
            if np.linalg.norm(st["vel"]) > 1.0 else np.array([0.0, 0.0, 1.0])
        return unit(np.cross(h_hat, st["n"]))

    def h_agl(self, st):
        if self.h_ref is None:
            return st["alt"]
        return st["alt"] - self.h_ref

    def ign_h(self, st, g):
        """Derating-aware ignition altitude: the thrust collapses at high axial speed, so the
        braking distance is far larger than v^2/(2(a-g)) with the static thrust."""
        v = max(st["speed"], 1.0)
        return min(300.0, max(150.0, 120.0 + v * v / 60.0))

    def ignition_ready(self, st, g):
        return self.h_agl(st) <= self.ign_h(st, g) and st["vr"] < 0.0

    # -- attitude ------------------------------------------------------------
    def attitude_cmd(self, st, desired_dir):
        """PD attitude control.

        The four/eight reaction wheels are key-driven (bang-bang), so the demanded
        torque magnitude is also mapped onto each wheel's 'speed' slider (torque
        scale).  That makes the actuator quasi-proportional: a large error commands a
        large spin-rate target, a small error a small one -- pure bang-bang at full
        speed spins the machine out of control (measured in run v2b).
        """
        hw = self.hw
        rot = st["rot"]
        up = unit(rot @ np.array([0.0, 1.0, 0.0]))
        d = unit(desired_dir)
        up_pred = unit(up + CFG["att_lead"] * np.cross(st["omega"], up))
        err = np.cross(up_pred, d)
        torque = CFG["kp_att"] * err - CFG["kd_att"] * st["omega"]
        cmds = []
        speeds = {}
        for g in hw["wheels"]:
            axis_world = rot @ hw["wheel_axis_local"][g]
            val = float(torque @ (-axis_world))
            speeds[g] = CFG["wheel_speed"]
            if abs(val) > CFG["wheel_deadband"]:
                cmds.append((g, "LeftKey" if val > 0 else "RightKey"))
        return cmds, float(up @ d), speeds

    def send(self, u_main, wheel_cmds, u_retro=0.0, extra_channels=(), wheel_speeds=None):
        hw = self.hw
        wheel_speeds = wheel_speeds or {}
        sliders = []
        for g in hw["boosters"]:
            sliders.append((g, "fthrust", float(u_main)))
        for g in hw["retro"]:
            sliders.append((g, "fthrust", float(u_retro)))
        for g in hw["wheels"]:
            sliders.append((g, "speed",
                            float(wheel_speeds.get(g, CFG["wheel_speed"]))))
            sliders.append((g, "damper", CFG["wheel_damper"]))
        channels = [(g, "ThrustKey") for g in hw["boosters"] if u_main > 0.02]
        channels.extend((g, "ThrustKey") for g in hw["retro"] if u_retro > 0.02)
        channels.extend(wheel_cmds)
        channels.extend(extra_channels)
        self.c.send_sliders(sliders)
        self.c.send_channels(channels=channels)

    # -- payload tracking ----------------------------------------------------
    def _update_payload(self, st):
        p = st["payload"]
        if p.get("pos") is None:
            if self.sep_t is not None and not self.payload_missing:
                self.payload_missing = True
                self.warn("payload_not_in_telemetry")
            return
        n = p["n"]
        self.payload = p
        self.payload_rmin = min(self.payload_rmin, p["r"])
        if self.payload_prev_n is not None:
            c = max(-1.0, min(1.0, float(self.payload_prev_n @ n)))
            self.payload_sweep += math.degrees(math.acos(c))
        self.payload_prev_n = n.copy()

    # -- abort ---------------------------------------------------------------
    def check_abort(self, st):
        if st["missing"] and self.phase in ("ascent_vert", "ascent_coast", "ascent_insert",
                                            "sep", "retro_kill"):
            return f"recovery_blocks_lost:{len(st['missing'])}"
        t = self.last_t
        fuel_frac = st["fuel"] / max(1e-6, self.fuel_total_full)
        if self.phase.startswith("ascent") and t > CFG["abort_low_energy_t"] \
                and st["alt"] < CFG["abort_low_energy_alt"] and st["vt"] < 40.0:
            return f"low_energy:t={t:.0f} alt={st['alt']:.0f} vt={st['vt']:.0f}"
        if st["vr"] < CFG["abort_vr_dive"] and self.phase.startswith("ascent"):
            return f"diving:vr={st['vr']:.0f}"
        if (fuel_frac < CFG["abort_fuel_frac"] and st["alt"] > 60.0
                and self.phase in ("ascent_vert", "ascent_insert", "retro_kill", "final_descent")):
            return f"fuel_low:{fuel_frac:.3f}"
        return ""

    # -- decision ------------------------------------------------------------
    def decide(self, st):
        t = self.last_t
        hw = self.hw
        u = 0.0
        u_retro = 0.0
        extra = []
        desired = st["n"]
        n = st["n"]
        g = min(G_MAX, MU / st["r"] ** 2)

        if self.phase == "pad":
            if t >= CFG["pad_time"]:
                self.phase = "ascent_vert"
                self.event("phase:ascent_vert", alt=round(st["alt"], 2),
                           a_u1=round(self.accel_u1(st), 2))
            return u, u_retro, extra, desired

        if self.phase == "ascent_vert":
            desired = n
            align = float(unit(st["rot"] @ np.array([0.0, 1.0, 0.0])) @ desired)
            u = CFG["throttle_max"] if align > 0.85 else 0.0
            el = elements(st["pos"], st["vel"])
            apogee_ok = (math.isfinite(el["ra"]) and el["ra"] >= CFG["r_apogee_target"]
                         and st["vr"] > 0.5)
            if apogee_ok:
                self.event("vertical_cut", alt=round(st["alt"], 1), vr=round(st["vr"], 2),
                           apogee=round(el["ra"] - R_SURFACE, 1), fuel=round(st["fuel"], 1))
                self.phase = "ascent_coast"
                self.ref_dir = None
            elif t > 25.0:
                self.abort = "vertical_timeout"
                self.phase = "done"
            return u, u_retro, extra, desired

        if self.phase == "ascent_coast":
            desired = self.prograde_dir(st)
            if self.ref_dir is None:
                self.ref_dir = desired.copy()
            else:
                self.ref_dir = slew_dir(self.ref_dir, desired,
                                        math.radians(CFG["ascent_slew_deg_s"]) * self.dt)
            desired = self.ref_dir
            if st["vr"] <= 1.0:
                self.event("apogee", alt=round(st["alt"], 1), vt=round(st["vt"], 2),
                           align=round(float(unit(st["rot"] @ np.array([0.0, 1.0, 0.0])) @ desired), 2))
                self.phase = "ascent_insert"
                self.insert_start = t
            elif t > 40.0:
                self.abort = "coast_timeout"
                self.phase = "done"
            return u, u_retro, extra, desired

        if self.phase == "ascent_insert":
            # apogee point: target tangential speed for an ellipse (r_a = current r, r_p = rp_target)
            r = st["r"]
            a_tgt = 0.5 * (r + CFG["rp_target"])
            vt_target = math.sqrt(max(0.0, MU * (2.0 / r - 1.0 / a_tgt)))
            east = self.prograde_dir(st)
            a_r = g - st["vt"] ** 2 / r + max(-40.0, min(40.0, CFG["insert_kr"] * (0.0 - st["vr"])))
            a_t = max(-10.0, min(80.0, CFG["insert_kt"] * (vt_target - st["vt"])))
            a_vec = a_r * n + a_t * east
            d = unit(a_vec)
            up = unit(st["rot"] @ np.array([0.0, 1.0, 0.0]))
            align = float(up @ d)
            wait_att = (align < 0.75 and abs(st["vr"]) < 6.0
                         and t - self.insert_start < 6.0)
            u = CFG["throttle_max"]
            if align < CFG["insert_align_gate"] or wait_att:
                u = 0.0
            desired = d
            el = elements(st["pos"], st["vel"])
            rp_ok = el["rp"] >= CFG["rp_target"] * 0.97
            overspeed = st["vt"] >= vt_target * 1.10
            done = ((rp_ok and st["vr"] > -4.0) or overspeed
                     or st["vt"] >= vt_target * 1.02) and st["alt"] > CFG["insert_alt_min"]
            if done:
                self.insert_elements = el
                self.event("insertion", alt=round(st["alt"], 1), vt=round(st["vt"], 2),
                           vr=round(st["vr"], 2), rp=round(el["rp"], 1), ra=round(el["ra"], 1),
                           fuel=round(st["fuel"], 1))
                self.phase = "sep"
            elif t - self.insert_start > CFG["insert_timeout"]:
                self.abort = "insert_timeout"
                self.event("abort:insert_timeout", alt=round(st["alt"], 1),
                           vt=round(st["vt"], 2), rp=round(el["rp"], 1))
                self.phase = "done"
            return u, u_retro, extra, desired

        if self.phase == "sep":
            desired = self.prograde_dir(st)
            if not self.decoupler_fired:
                extra = [(g2, "ExplodeKey") for g2 in hw["decouplers"]]
                self.decoupler_fired = True
                self.sep_t = t
                self.detached = {g2 for g2 in hw["decouplers"]}
                self.event("separation", alt=round(st["alt"], 1), vt=round(st["vt"], 2),
                           vr=round(st["vr"], 2), payload_count=st["payload"].get("count"))
            elif t - self.sep_t > CFG["sep_fire_hold"]:
                self.phase = "retro_kill"
                self.retro_start = t
                # payload-scope gate bookkeeping starts at separation
                self.payload_sweep = 0.0
                self.payload_rmin = 1e18
                self.payload_prev_n = None
                self.event("phase:retro_kill", vt=round(st["vt"], 2))
            return u, u_retro, extra, desired

        if self.phase == "retro_kill":
            self._update_payload(st)
            off_h, v_h = self.horizontal(st)
            vh_norm = float(np.linalg.norm(v_h))
            offset = self.offset_from_start(st)
            off_norm = float(np.linalg.norm(offset))
            h_agl = self.h_agl(st)
            # time still available before the terminal burn must start
            t_fall = max(6.0, (max(h_agl - 340.0, 0.0)) / max(12.0, -st["vr"]) + 6.0)
            v_goal = (-offset / t_fall) if off_norm > 20.0 else np.zeros(3)
            dv_needed = v_goal - v_h
            dv_norm = float(np.linalg.norm(dv_needed))
            if dv_norm > 0.8:
                # the burn axis is the direction the thrust must add delta-v: kill the
                # horizontal velocity and set the residual velocity that drifts the stack
                # back over the launch pad (nose = opposite of the required delta-v)
                desired = -unit(dv_needed)
                # rotate upright during the last part of the burn (fall has no time for a full
                # 90 deg slew afterwards: measured 5.7 deg/s -> 16 s vs ~13 s of fall)
                late = max(0.0, min(1.0, (35.0 - st["vt"]) / 35.0))
                desired = unit(desired + CFG["retro_up_mix"] * late * n * 1.4)
            else:
                desired = n
            up = unit(st["rot"] @ np.array([0.0, 1.0, 0.0]))
            align = float(up @ desired)
            u_retro = CFG["retro_throttle"] if align > CFG["retro_align_gate"] else 0.0
            if (dv_norm <= 1.2 or st["alt"] < 450.0 or
                    (self.retro_start is not None and t - self.retro_start > CFG["retro_timeout"])):
                self.event("retro_kill_end", vt=round(st["vt"], 2), vr=round(st["vr"], 2),
                           alt=round(st["alt"], 1), fuel=round(st["fuel"], 1),
                           dv_left=round(dv_norm, 2))
                self.phase = "reorient"
                self.trim_start = t
            return u, u_retro, extra, desired

        if (self.phase == "reorient" and self.h_agl(st) < 140.0 and st["vr"] < -50.0
                and float(unit(st["rot"] @ np.array([0.0, 1.0, 0.0])) @ n) > 0.70):
            self.phase = "final_descent"
            self.ignite_start = t
            self.integrator = 0.0
            self.event("ignition", alt=round(st["alt"], 1), h_agl=round(self.h_agl(st), 1),
                       vr=round(st["vr"], 2), speed=round(st["speed"], 2),
                       h_ign=round(self.ign_h(st, g), 1), from_phase="reorient_emergency")

        if self.phase == "reorient":
            # rotate upright before any further thrust (no thrust during the slew)
            self._update_payload(st)
            off_h, v_h = self.horizontal(st)
            up = unit(st["rot"] @ np.array([0.0, 1.0, 0.0]))
            align = float(up @ n)
            desired = n
            if align > 0.97 or (t - self.trim_start) > CFG["reorient_time_max"]:
                self.event("reorient_end", align=round(align, 3),
                           off_h=round(float(np.linalg.norm(self.offset_from_start(st))), 1),
                           v_h=round(float(np.linalg.norm(v_h)), 2))
                self.phase = "hover_trim"
                self.trim_start = t
            return 0.0, 0.0, extra, desired

        if self.phase == "hover_trim":
            # Stop the fall, then translate over the pad in a *near-hover* state: at v~0 the
            # dynamic pressure is ~0, the aero windvane torque disappears and the reaction
            # wheels can actually hold/aim the stack.  All lateral work happens here -- the
            # powered descent afterwards is strictly vertical (diagnosed 16:35: any tilt in
            # the atmospheric terminal burn injects tens of m/s of uncontrolled lateral speed).
            self._update_payload(st)
            off_h, v_h = self.horizontal(st)
            offset = self.offset_from_start(st)
            off_norm = float(np.linalg.norm(offset))
            vh_norm = float(np.linalg.norm(v_h))
            h_agl = self.h_agl(st)
            a_up = max(0.0, g + CFG["hover_kv"] * (0.0 - st["vr"]))
            # a_lat = -(w^2 * offset + 2*zeta*w * v_h): critically damped position loop with
            # w ~ 0.11 rad/s, so the demanded tilt stays under ~20 deg and the wheels track it
            a_lat = -(CFG["hover_kp_pos"] * offset + CFG["hover_kd_vel"] * v_h)
            mag = float(np.linalg.norm(a_lat))
            a_lat_max = CFG["hover_tilt"] * g      # tan(angle) = hover_tilt
            if mag > a_lat_max:
                a_lat *= a_lat_max / mag
            a_vec = a_up * n + a_lat
            d_raw = unit(a_vec)
            if self.hover_ref is None:
                self.hover_ref = d_raw.copy()
            else:
                self.hover_ref = slew_dir(self.hover_ref, d_raw,
                                          math.radians(CFG["hover_slew_deg_s"]) * self.dt)
            d = self.hover_ref
            up = unit(st["rot"] @ np.array([0.0, 1.0, 0.0]))
            align = float(up @ d)
            u_raw = min(CFG["throttle_max"],
                        float(np.linalg.norm(a_vec)) / max(1e-6, self.accel_u1(st)))
            u_raw *= min(1.0, max(0.0, align) * 1.3)
            self.u_filtered = CFG["thrust_filter"] * self.u_filtered + \
                (1.0 - CFG["thrust_filter"]) * u_raw
            u = self.u_filtered
            desired = d
            hover_ok = (off_norm < CFG["hover_tol_off"] and vh_norm < CFG["hover_tol_vh"]
                        and abs(st["vr"]) < 2.0)
            emergency = (h_agl < 120.0 and st["vr"] < -45.0)
            if (hover_ok or emergency or (t - self.trim_start) > CFG["hover_time"]
                    or st["fuel"] < CFG["hover_fuel_floor"]):
                self.event("hover_trim_end", off=round(off_norm, 1), v_h=round(vh_norm, 2),
                           vr=round(st["vr"], 2), alt=round(st["alt"], 1),
                           fuel=round(st["fuel"], 1), ok=bool(hover_ok), emergency=bool(emergency))
                self.phase = "descent"
                self.integrator = 0.0
            return u, 0.0, extra, desired

        if self.phase == "descent":
            self._update_payload(st)
            desired = n
            h_agl = self.h_agl(st)
            h_ign = self.ign_h(st, g)
            if self.ignition_ready(st, g):
                self.event("ignition", alt=round(st["alt"], 1), h_agl=round(h_agl, 1),
                           vr=round(st["vr"], 2), speed=round(st["speed"], 2),
                           h_ign=round(h_ign, 1))
                self.phase = "final_descent"
                self.ignite_start = t
                self.integrator = 0.0
            return u, u_retro, extra, desired

        if self.phase == "final_descent":
            # STRICTLY VERTICAL powered descent: the thrust command has no lateral component,
            # so an attitude tracking lag can never inject horizontal velocity.
            self._update_payload(st)
            h_agl = max(0.0, self.h_agl(st))
            target_sink = -max(CFG["sink_min"], min(CFG["sink_max"], CFG["sink_k"] * h_agl))
            err = target_sink - st["vr"]
            self.integrator = max(-CFG["terminal_int_max"],
                                  min(CFG["terminal_int_max"],
                                      self.integrator + CFG["terminal_ki"] * err * self.dt))
            acc_des = g + CFG["terminal_kp"] * err + 6.0 * self.integrator
            a_lat = np.zeros(3)
            if h_agl < CFG["term_lat_alt"]:
                off_h, v_h = self.horizontal(st)
                vh = float(np.linalg.norm(v_h))
                if vh > 0.2:
                    a_lat = -CFG["term_lat_kv"] * v_h
                    m = float(np.linalg.norm(a_lat))
                    if m > CFG["term_lat_max"]:
                        a_lat *= CFG["term_lat_max"] / m
            a_vec = max(-8.0, min(70.0, acc_des)) * n + a_lat
            d = unit(a_vec)
            up = unit(st["rot"] @ np.array([0.0, 1.0, 0.0]))
            align = float(up @ d)
            u_raw = min(CFG["throttle_max"],
                        float(np.linalg.norm(a_vec)) / max(1e-6, self.accel_u1(st)))
            u_raw *= min(1.0, max(0.0, align) * 1.3)   # soft attitude scaling, never a hard cut
            self.u_filtered = CFG["thrust_filter"] * self.u_filtered + \
                (1.0 - CFG["thrust_filter"]) * u_raw
            u = self.u_filtered
            desired = d
            contact = (h_agl < 1.5 and (t - self.ignite_start) > 1.0) or (h_agl < -4.0) or \
                      (self.vr_prev < -1.5 and st["vr"] > -0.6 and h_agl < 30.0)
            if contact or (h_agl < 0.8 and st["vr"] < 0.0 and st["speed"] < 6.0):
                self.event("touchdown", alt=round(st["alt"], 2), h_agl=round(h_agl, 2),
                           speed=round(st["speed"], 3), vr=round(st["vr"], 3),
                           fuel=round(st["fuel"], 1))
                self.phase = "touchdown"
                self.touch_t = t
                self.touch_pos = st["pos"].copy()
            return u, 0.0, extra, desired

        if self.phase == "touchdown":
            self._update_payload(st)
            desired = n
            if self.touch_t is not None and t - self.touch_t > CFG["settle_time"]:
                self.event("settled", pos=[round(float(c), 2) for c in st["pos"]],
                           speed=round(st["speed"], 3), integrity=st["integrity"],
                           alive=st["alive"])
                self.phase = "wait_payload"
            return 0.0, 0.0, extra, desired

        if self.phase == "wait_payload":
            self._update_payload(st)
            desired = n
            strict_sweep = self.payload_sweep - self.thrust_end_sweep
            if not self.payload_missing and strict_sweep >= CFG["payload_gate_deg"]:
                self.event("payload_gate_reached", strict_sweep_deg=round(strict_sweep, 2),
                           sweep_from_sep_deg=round(self.payload_sweep, 2),
                           rmin=round(self.payload_rmin, 1),
                           last_thrust_t=self.last_thrust_t)
                self.phase = "done"
            elif t > CFG["payload_timeout"]:
                self.abort = "payload_timeout"
                self.event("abort:payload_timeout", sweep_deg=round(self.payload_sweep, 2),
                           strict_sweep_deg=round(strict_sweep, 2))
                self.phase = "done"
            return 0.0, 0.0, extra, desired

        if self.phase == "done":
            return 0.0, 0.0, extra, desired
        return u, u_retro, extra, desired

    # -- frame ---------------------------------------------------------------
    def step(self, frame):
        st = self.read_state(frame)
        if self.t0 is None:
            self.t0 = float(frame.simulation_time)
            self.launch_n = st["n"].copy()
            self.pos0 = st["pos"].copy()
            self.event("controller_start", alt=round(st["alt"], 2),
                       mass_est=round(self.hw["dry_total"] + FUEL_UNIT_MASS * st["fuel"], 2),
                       dry_payload=self.hw["dry_payload"],
                       dry_recovery=self.hw["dry_recovery"],
                       payload_blocks=len(self.hw["payload"]),
                       main_boosters=len(self.hw["boosters"]),
                       retro_boosters=len(self.hw["retro"]))
        self.last_t = float(frame.simulation_time) - self.t0
        if self.rows:
            self.dt = max(1e-3, min(0.2, self.last_t - self.rows[-1]["t"]))
        if self.h_ref is None:
            self.h_ref = st["alt"]
            self.event("ground_reference", h_ref=round(self.h_ref, 3))
        self._update_payload(st)
        abort = self.check_abort(st)
        if abort and not self.abort:
            self.abort = abort
            self.event("abort", reason=abort)
            self.phase = "done"
        u, u_retro, extra, desired = self.decide(st)
        wheel_cmds, align, wheel_speeds = self.attitude_cmd(st, desired)
        self.send(u, wheel_cmds, u_retro, extra, wheel_speeds)
        # strict-G1 bookkeeping: track the last moment any engine was lit
        sim_t = float(frame.simulation_time)
        powered = (float(u) > 0.02) or (float(u_retro) > 0.02)
        if powered:
            self.powered_now = True
            self.last_thrust_sim_t = sim_t
            self.last_thrust_t = sim_t
        elif self.powered_now:
            self.powered_now = False
            self.thrust_end_sweep = self.payload_sweep
        strict_sweep = self.payload_sweep - self.thrust_end_sweep
        off_h, v_h = self.horizontal(st)
        rec = {
            "t": round(self.last_t, 4),
            "sim_t": round(sim_t, 4), "phase": self.phase,
            "duty": round(max(float(u), float(u_retro)), 4),
            "u": round(float(u), 4), "u_retro": round(float(u_retro), 4),
            "alt": round(st["alt"], 3), "h_agl": round(self.h_agl(st), 3),
            "r": round(st["r"], 3), "vr": round(st["vr"], 3), "vt": round(st["vt"], 3),
            "speed": round(st["speed"], 3), "align": round(align, 4),
            "fuel": round(st["fuel"], 2),
            "mass": round(self.hw["dry_total"] + FUEL_UNIT_MASS * st["fuel"], 3),
            "off_h": round(float(np.linalg.norm(off_h)), 2),
            "n": [round(float(c), 5) for c in st["n"]],
            "pos": [round(float(c), 3) for c in st["pos"]],
            "vel": [round(float(c), 4) for c in st["vel"]],
            "payload_r": round(st["payload"].get("r", float("nan")), 3),
            "payload_alt": round(st["payload"].get("alt", float("nan")), 2),
            "payload_vt": round(st["payload"].get("vt", float("nan")), 3),
            "payload_sweep": round(self.payload_sweep, 3),
            "strict_sweep_deg": round(strict_sweep, 3),
            "last_thrust_t": "" if self.last_thrust_t is None else round(self.last_thrust_t, 3),
            "payload_rmin": round(self.payload_rmin, 2) if self.payload_rmin < 1e17 else "",
            "payload_count": st["payload"].get("count", 0),
            "integrity": st["integrity"], "alive": st["alive"],
        }
        self.rows.append(rec)
        if len(self.rows) % 250 == 0:
            print(f"t={self.last_t:7.2f} {self.phase:14s} alt={st['alt']:8.2f} vr={st['vr']:8.2f} "
                  f"vt={st['vt']:7.2f} u={u:4.2f}/{u_retro:4.2f} fuel={st['fuel']:6.1f} "
                  f"off={rec['off_h']:6.1f} pl_r={st['payload'].get('r', float('nan')):8.1f} "
                  f"sweep={self.payload_sweep:8.2f}", flush=True)
        self.vr_prev = st["vr"]
        return self.phase == "done"

    # -- outputs -------------------------------------------------------------
    def write_outputs(self):
        if self.rows:
            header = list(self.rows[0].keys())
            with (self.run_dir / "control_log.csv").open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle)
                writer.writerow(header)
                for row in self.rows:
                    writer.writerow([
                        json.dumps(row[k]) if isinstance(row[k], (list, dict)) else row[k]
                        for k in header])
        el = self.insert_elements
        summary = {
            "frames": len(self.rows),
            "abort": self.abort,
            "events": self.events,
            "warnings": self.warnings,
            "payload_sweep_deg": round(self.payload_sweep, 3),
            "payload_rmin": round(self.payload_rmin, 2) if self.payload_rmin < 1e17 else None,
            "payload_missing": self.payload_missing,
            "final_phase": self.phase,
            "final": self.rows[-1] if self.rows else None,
            "dry_payload": self.hw["dry_payload"],
            "dry_recovery": self.hw["dry_recovery"],
            "insert_elements": None if el is None else {
                "rp": round(el["rp"], 2), "ra": round(el["ra"], 2),
                "period": round(2 * math.pi * math.sqrt(el["a"] ** 3 / MU), 2) if el["a"] else None},
            "launch_pos": None if self.pos0 is None else [round(float(c), 3) for c in self.pos0],
            "touch_pos": None if self.touch_pos is None else [round(float(c), 3) for c in self.touch_pos],
        }
        (self.run_dir / "mission_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"mission_controller: frames={len(self.rows)} phase={self.phase} abort={self.abort!r} "
              f"sweep={self.payload_sweep:.2f} rmin={summary['payload_rmin']} "
              f"missing={self.payload_missing}", flush=True)


def main() -> int:
    bsg = Path(os.environ[ENV_MACHINE_BSG])
    run_dir = Path(os.environ[ENV_RUN_DIR])
    hw = load_machine(bsg)
    print(f"mission_controller: root={hw['root']['guid']} main={len(hw['boosters'])} "
          f"retro={len(hw['retro'])} tanks={len(hw['tanks'])} wheels={len(hw['wheels'])} "
          f"decouplers={len(hw['decouplers'])} payload_blocks={len(hw['payload'])} "
          f"dry_payload={hw['dry_payload']} dry_recovery={hw['dry_recovery']}", flush=True)
    client = ControllerClient.from_environment(poll_interval=0.005)
    mission = Mission(hw, client, run_dir)
    client.arm()
    try:
        frame = client.wait_until_running(timeout=60.0)
        client.load_block_table()
        print(f"mission_controller: live at t={frame.simulation_time:.3f}", flush=True)
        while True:
            if mission.step(frame):
                break
            frame = client.next_sample(timeout=5.0)
    except Exception as exc:  # noqa: BLE001 - diagnostics are part of the deliverable
        mission.abort = f"{type(exc).__name__}: {exc}"
        mission.events.append({"t": round(mission.last_t, 3), "event": "exception",
                               "detail": mission.abort})
        print(f"mission_controller ABORT: {mission.abort}", flush=True)
    finally:
        mission.write_outputs()
        try:
            client.close()
        except Exception as exc:  # noqa: BLE001
            print(f"mission_controller: close failed: {exc}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
