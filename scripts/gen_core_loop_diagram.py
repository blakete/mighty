#!/usr/bin/env python3
"""Draw docs/goal_selector_core_loop.{svg,png}: the MAPPER -> GOAL SELECTOR <-> PLANNER loop.

Companion of scripts/gen_architecture_diagram.py (which is left untouched and imported as a library):
the layout here is hand-placed, but every topic edge drawn is VERIFIED against the pubs/subs that
gen_architecture_diagram.build_model() extracts from the C++ sources (+ docs/external_nodes.yaml for
the mapper / RViz / MPC), and every parameter value shown is READ from config/hw_*.yaml at run time.
Python 3 + matplotlib only.   python3 scripts/gen_core_loop_diagram.py [--check]   (--check: exit 1 if stale)
"""

import argparse
import io
import re
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

import gen_architecture_diagram as gad  # noqa: E402

ROOT = gad.ROOT
OUT_SVG = ROOT / "docs" / "goal_selector_core_loop.svg"
OUT_PNG = ROOT / "docs" / "goal_selector_core_loop.png"
PLANNER_YAML = ROOT / "config" / "hw_mighty_ground_robot.yaml"
SELECTOR_YAML = ROOT / "config" / "hw_goal_selector.yaml"   # layered on top of the planner YAML
STATUS_MSG = ROOT.parent / "goal_selector_msgs" / "msg" / "PlannerStatus.msg"


# ------------------------------------------------------------------------------------------------
# Verification against the code
# ------------------------------------------------------------------------------------------------
# (topic, message type, publisher node or None, [subscriber nodes]); node ids as in build_model().
EDGES = [
    ("occ_2d_topic", "nav_msgs/OccupancyGrid", "mapper", ["goal_selector", "mighty_node"]),
    ("planning_occ_2d_topic", "nav_msgs/OccupancyGrid", "mapper", []),   # no subscriber anywhere
    ("term_goal", "geometry_msgs/PoseStamped", "goal_selector", ["mighty_node"]),
    ("selector_map_2d", "nav_msgs/OccupancyGrid", "goal_selector", []),        # RViz only (viz)
    ("point_selector_goal", "geometry_msgs/PointStamped", "goal_selector", []),  # RViz only (viz)
    ("planner_status", "goal_selector_msgs/PlannerStatus", "mighty_node", ["goal_selector"]),
    ("term_goal_rviz", "geometry_msgs/PoseStamped", "rviz", ["goal_selector"]),
    ("state", "dynus_interfaces/State", "convert_odom_to_state", ["goal_selector", "mighty_node"]),
    ("mpc_waypoints", "dynus_interfaces/SpeedyPath", "mighty_node", ["mpc"]),
]


def verify_edges(nodes):
    def eps(node, direction, topic):
        if node not in nodes:
            raise SystemExit(f"EDGE CHECK FAILED: node '{node}' not found in the extracted model "
                             f"(have: {', '.join(sorted(nodes))})")
        return [e for e in nodes[node].endpoints
                if e.kind == "topic" and e.direction == direction and e.topic == topic and e.active]

    for topic, mtype, pub, subs in EDGES:
        mtype = gad.norm_type(mtype)
        for node, direction in [(pub, "pub")] + [(s, "sub") for s in subs]:
            found = eps(node, direction, topic)
            if not found:
                raise SystemExit(f"EDGE CHECK FAILED: {node} does not {direction} '{topic}' "
                                 f"(code / external_nodes.yaml changed?)")
            if not any(gad.norm_type(e.mtype) == mtype for e in found):
                raise SystemExit(f"EDGE CHECK FAILED: {node} {direction} '{topic}' has type "
                                 f"{sorted({e.mtype for e in found})}, drawn as {mtype}")
        # Nobody else may subscribe to a topic drawn with a fixed subscriber list.
        others = [n for n in nodes for e in nodes[n].endpoints
                  if e.kind == "topic" and e.direction == "sub" and e.topic == topic and e.active
                  and n not in subs and n != "rviz"]
        if others:
            raise SystemExit(f"EDGE CHECK FAILED: '{topic}' also has subscribers {sorted(set(others))} "
                             f"that the diagram does not draw")


def read_statuses():
    """PlannerStatus.msg constants -> {NAME: value}."""
    if not STATUS_MSG.exists():
        raise SystemExit(f"missing {STATUS_MSG}")
    out = {}
    for line in STATUS_MSG.read_text().splitlines():
        m = re.match(r"\s*uint8\s+([A-Z_]+)\s*=\s*(\d+)", line)
        if m:
            out[m.group(1)] = int(m.group(2))
    return out


# ------------------------------------------------------------------------------------------------
# Parameters read from the hw YAMLs (simple "key: value  # comment" line parse)
# ------------------------------------------------------------------------------------------------
def load_params():
    params = {}
    for f in (PLANNER_YAML, SELECTOR_YAML):     # selector layer wins, as at launch
        if not f.exists():
            raise SystemExit(f"missing parameter file {f}")
        for line in f.read_text().splitlines():
            m = re.match(r"^\s+([A-Za-z0-9_.]+):\s*([^\s#]+)", line)
            if m and not line.lstrip().startswith("#"):
                params[m.group(1)] = m.group(2)
    return params


class P:
    def __init__(self, d):
        self.d = d

    def __call__(self, key):
        if key not in self.d:
            raise SystemExit(f"parameter '{key}' not found in {PLANNER_YAML.name} / {SELECTOR_YAML.name}")
        return self.d[key]


# ------------------------------------------------------------------------------------------------
# Drawing
# ------------------------------------------------------------------------------------------------
BG = "#fbfaf7"
INK = "#26282b"
MUTE = "#6b7078"
GREY = "#a4a8ae"
ACC = {"mapper": "#2f7f86", "selector": "#b5702a", "planner": "#4a5aa8", "side": "#7b8088"}
W, H = 256.0, 120.0
MONO = "DejaVu Sans Mono"


def tint(hex_color, t):
    """Mix a colour with white; t=1 keeps the colour."""
    c = [int(hex_color[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{int(255 - (255 - v) * t):02x}" for v in c)


class Canvas:
    def __init__(self):
        self.fig = plt.figure(figsize=(W / 10, H / 10), facecolor=BG)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, W)
        self.ax.set_ylim(0, H)
        self.ax.axis("off")
        self.fits = []      # (text artist, x0, x1, y0, y1) container the text must stay inside

    def text(self, x, y, s, size=8, color=INK, weight="normal", ha="left", va="center",
             family=None, bg=False, fit=None, style="normal", zorder=6, linespacing=1.25):
        kw = {}
        if family:
            kw["family"] = family
        if bg:
            kw["bbox"] = dict(fc=BG, ec="none", pad=1.2)
        t = self.ax.text(x, y, s, fontsize=size, color=color, fontweight=weight, ha=ha, va=va,
                         style=style, zorder=zorder, linespacing=linespacing, **kw)
        if fit:
            self.fits.append((t, *fit))
        return t

    def rbox(self, x0, y0, w, h, fc, ec, lw=1.2, ls="-", rs=1.4, zorder=2):
        self.ax.add_patch(FancyBboxPatch((x0, y0), w, h, boxstyle=f"round,pad=0,rounding_size={rs}",
                                         fc=fc, ec=ec, lw=lw, ls=ls, zorder=zorder))

    def arrow_path(self, pts, color, lw=1.6, ls="-", head=True, dot=False):
        xs, ys = zip(*pts)
        last = pts[-2:]
        if len(pts) > 2:
            self.ax.plot(xs[:-1], ys[:-1], color=color, lw=lw, ls=ls, solid_capstyle="butt", zorder=4)
        if head:
            self.ax.add_patch(FancyArrowPatch(last[0], last[1], arrowstyle="-|>", mutation_scale=11,
                                              color=color, lw=lw, ls=ls, shrinkA=0, shrinkB=0, zorder=4))
        else:
            self.ax.plot(*zip(*last), color=color, lw=lw, ls=ls, zorder=4)
        if dot:
            self.ax.plot([pts[0][0]], [pts[0][1]], "o", color=color, ms=4, zorder=5)

    def check_fits(self):
        self.fig.canvas.draw()
        r = self.fig.canvas.get_renderer()
        inv = self.ax.transData.inverted()
        bad = []
        for t, x0, x1, y0, y1 in self.fits:
            bb = t.get_window_extent(r)
            (a, b), (c, d) = inv.transform((bb.x0, bb.y0)), inv.transform((bb.x1, bb.y1))
            if a < x0 - 0.2 or c > x1 + 0.2 or b < y0 - 0.2 or d > y1 + 0.2:
                bad.append(f"{t.get_text()[:50]!r}: text box x[{a:.1f},{c:.1f}] y[{b:.1f},{d:.1f}] "
                           f"outside x[{x0},{x1}] y[{y0},{y1}]")
        if bad:
            raise SystemExit("LAYOUT CHECK FAILED (text outside its box):\n  " + "\n  ".join(bad))


def stage_box(cv, x0, ytop, w, h, accent, title, desc, params):
    """A pipeline box: bold title, plain description lines, monospace parameter lines."""
    y0 = ytop - h
    cv.rbox(x0, y0, w, h, fc="white", ec=accent, lw=1.1)
    box = (x0, x0 + w, y0, ytop)
    cv.text(x0 + 1.5, ytop - 2.0, title, size=9, color=accent, weight="bold", fit=box)
    y = ytop - 4.6
    for line in desc:
        cv.text(x0 + 1.5, y, line, size=7.6, fit=box)
        y -= 1.5
    y -= 0.35
    for line in params:
        cv.text(x0 + 1.5, y, line, size=7.1, color=tint(accent, 1.0), family=MONO, fit=box)
        y -= 1.5
    if y < y0 - 0.1:
        raise SystemExit(f"box '{title}' overflows by {y0 - y:.1f} units")
    return (x0, y0, w, h)


def build(p, statuses):
    cv = Canvas()
    ax = cv.ax
    S, PL, M, SD = ACC["selector"], ACC["planner"], ACC["mapper"], ACC["side"]

    cv.text(2, 117.6, "MIGHTY core loop: mapper, goal selector, planner", size=17, weight="bold", fit=(0, W, 0, H))
    cv.text(2, 113.9, "branch feature/modular-goal-selection (Phase 5).  Edges are ROS topics, verified against the C++ "
            "sources and docs/external_nodes.yaml; parameter values are read from config/hw_mighty_ground_robot.yaml "
            "+ config/hw_goal_selector.yaml.", size=8.2, color=MUTE, fit=(0, W, 0, H))

    # ---- process blocks --------------------------------------------------------------------
    MX0, MX1, MY0, MY1 = 2, 30, 44, 100
    SX0, SX1, BY0, BY1 = 56, 126, 14, 100
    PX0, PX1 = 182, 252
    for (x0, x1, y0, y1, acc, ttl, sub) in [
        (MX0, MX1, MY0, MY1, M, "MAPPER", None),
        (SX0, SX1, BY0, BY1, S, "GOAL SELECTOR", "goal_selector node"),
        (PX0, PX1, BY0, BY1, PL, "PLANNER", "mighty_node"),
    ]:
        cv.rbox(x0, y0, x1 - x0, y1 - y0, fc=tint(acc, 0.09), ec=acc, lw=2.0, rs=2.2, zorder=1)
        cv.text(x0 + 2, y1 - 2.6, ttl, size=13, color=acc, weight="bold", fit=(x0, x1, y0, y1))
        if sub:
            cv.text(x0 + 2 + (len(ttl) * 1.55 + 3.0), y1 - 2.6, sub, size=8.6, color=MUTE, fit=(x0, x1, y0, y1))

    # ---- mapper ----------------------------------------------------------------------------
    mapper_lines = [
        ("elevation_mapping_cupy", "bold", INK), ("external repo, not in this workspace", "normal", MUTE),
        ("", "normal", INK),
        ("persistent-occupancy plugin", "normal", INK), ("0.1 m cells, 5 Hz", "normal", INK),
        ("", "normal", INK), ("grid values", "bold", INK), ("free 0 / unknown -1", "normal", INK),
        ("/ occupied 100", "normal", INK),
    ]
    y = MY1 - 7.3
    for s, wgt, col in mapper_lines:
        cv.text(MX0 + 2, y, s, size=7.8, color=col, weight=wgt, fit=(MX0, MX1, MY0, MY1))
        y -= 2.1

    # ---- selector pipeline -----------------------------------------------------------------
    bx, bw = SX0 + 4, SX1 - SX0 - 8
    sh, sg, top0 = 14.0, 1.5, 93.5
    sel = []
    sel.append(stage_box(cv, bx, top0, bw, sh, S, "1  Visited map (persistent)",
                         ["absorbs every occ_2d_topic frame and is fused into the local grid,",
                          "so revisited corridors keep their explored cells"],
                         [f"visited_map {p('exploration.visited_map.width_m')} x {p('exploration.visited_map.height_m')} m @ "
                          f"{p('exploration.visited_map.resolution_m')} m",
                          f"fuse_into_local={p('exploration.visited_map.fuse_into_local')}  "
                          f"detect_on_visited_map={p('exploration.visited_map.detect_on_visited_map')}"]))
    sel.append(stage_box(cv, bx, top0 - (sh + sg), bw, sh, S, "2  Derived frontier grid",
                         ["occupied band: cells within inflation_2d_m of an occupied cell -> OCCUPIED",
                          "unknown band: unknown cells + cells within unknown_inflation_2d_m -> UNKNOWN",
                          "(the unknown band is NOT occupied; it stays walkable for the reachability walk)"],
                         [f"inflation_2d_m={p('inflation_2d_m')} m   unknown_inflation_2d_m={p('unknown_inflation_2d_m')} m"]))
    sel.append(stage_box(cv, bx, top0 - 2 * (sh + sg), bw, sh, S, "3  Frontier detection",
                         ["BFS from the robot over free + unknown-band cells (never real unknown",
                          "or the occupied band); frontier = free cell outside both bands that is",
                          "8-adjacent to the unknown band; frontier cells are clustered"],
                         [f"cluster_min_cells={p('exploration.detector.cluster_min_cells')}  "
                          f"border_margin_cells={p('exploration.detector.border_margin_cells')}  "
                          f"obstacle_clearance_cells={p('exploration.detector.obstacle_clearance_cells')}",
                          f"robot_snap_radius_m={p('exploration.detector.robot_snap_radius_m')}"]))
    sel.append(stage_box(cv, bx, top0 - 3 * (sh + sg), bw, sh, S, "4  FrontierManager",
                         ["records ACTIVE / VISITED / INVALIDATED; VISITED once no unknown-band cell",
                          "is within verify_radius_cells of the centroid; utility ranks the ACTIVE ones",
                          "(size, distance, info, heading) and can preempt the current goal"],
                         [f"verify_radius_cells={p('exploration.manager.verify_radius_cells')}  "
                          f"merge_radius_m={p('exploration.manager.merge_radius_m')}  "
                          f"invalidation_cooldown_sec={p('exploration.manager.invalidation_cooldown_sec')}"]))
    sel.append(stage_box(cv, bx, top0 - 4 * (sh + sg), bw, sh, S, "5  Commit + watchdogs",
                         ["one commitment at a time: frontier | manual (term_goal_rviz) | return-home;",
                          "each commitment gets a new stamp. Stuck watchdog (also releases manual goals)",
                          "and pursuit timeout (invalidates a frontier) end a commitment"],
                         [f"stuck_timeout_sec={p('exploration.manager.stuck_timeout_sec')} s  "
                          f"stuck_move_thresh_m={p('exploration.manager.stuck_move_thresh_m')} m",
                          f"pursuit: max({p('exploration.manager.pursuit_timeout_min_sec')} s, dist/"
                          f"{p('exploration.manager.pursuit_timeout_v_ref')} * {p('exploration.manager.pursuit_timeout_factor')})",
                          f"exploration.enabled={p('exploration.enabled')} (hw default)  select_rate_hz={p('exploration.select_rate_hz')}"]))
    for a, b in zip(sel[:-1], sel[1:]):
        cv.arrow_path([(bx + bw / 2, a[1]), (bx + bw / 2, b[1] + b[3])], S, lw=1.3)

    # ---- planner pipeline ------------------------------------------------------------------
    px0 = PX0 + 4
    ph, pg = 17.5, 2.0
    unk_cost = 1.0 + float(p("w_unknown"))
    pl = []
    pl.append(stage_box(cv, px0, top0, bw, ph, PL, "A  Planning 2D map",
                        ["built from the RAW occ_2d_topic (tri-state)",
                         "occupied cells inflated by inflation_2d_m -> blocked; unknown stays unknown",
                         "(traversable for A*, blocked for the executed prefix)"],
                        [f"inflation_2d_m={p('inflation_2d_m')} m   map res mighty_map_res={p('mighty_map_res')} m"]))
    pl.append(stage_box(cv, px0, top0 - (ph + pg), bw, ph, PL, "B  Global path (A*)",
                        ["A* to the horizon point G inside the robot-centred window;",
                         "each unknown step costs extra (w_unknown); a goal in unknown is plannable",
                         "goal on occupied / occupied band -> plan fails -> status FAILED"],
                        [f"horizon={p('horizon')} m   w_unknown={p('w_unknown')} (unknown step = {unk_cost:g}x free)"]))
    pl.append(stage_box(cv, px0, top0 - 2 * (ph + pg), bw, ph, PL, "C  Executed prefix",
                        ["global path is cut where it enters a run of unknown cells of the raw grid",
                         "(edge of observed space), then backed off along the path;",
                         "shorter unknown holes (mapper speckle) are driven through"],
                        [f"trim_min_unknown_run_cells={p('trim_min_unknown_run_cells')}   "
                         f"back-off unknown_inflation_2d_m={p('unknown_inflation_2d_m')} m"]))
    pl.append(stage_box(cv, px0, top0 - 3 * (ph + pg), bw, ph, PL, "D  Local trajectory (MIGHTY, L-BFGS)",
                        ["optimises a trajectory over the prefix (end point = prefix end)",
                         "and publishes it as waypoints for the MPC"],
                        [f"num_N={p('num_N')} pieces   use_esdf_cost={p('use_esdf_cost')}"]))
    for a, b in zip(pl[:-1], pl[1:]):
        cv.arrow_path([(px0 + bw / 2, a[1]), (px0 + bw / 2, b[1] + b[3])], PL, lw=1.3)

    # ---- occ_2d_topic (mapper -> selector + planner) over the top ---------------------------
    ylane = 106.6
    xs_sel, xp = bx + bw - 6, px0 + bw - 6
    cv.arrow_path([(16, MY1), (16, ylane), (xp, ylane), (xp, top0)], M, lw=2.0)
    cv.arrow_path([(xs_sel, ylane), (xs_sel, top0)], M, lw=2.0, dot=True)
    cv.text(34, ylane + 1.9, "occ_2d_topic", size=9.5, color=M, weight="bold", bg=True)
    cv.text(48.5, ylane + 1.9, "nav_msgs/OccupancyGrid  raw tri-state (free / unknown / occupied), best_effort; "
            "to selector and planner (planner no longer reads planning_occ_2d_topic)", size=8, color=INK, bg=True)

    # ---- planning_occ_2d_topic (mapper -> nobody), greyed -----------------------------------
    yp = 70.0
    cv.arrow_path([(MX1, yp), (50.5, yp)], GREY, lw=1.5, ls=(0, (4, 3)), head=False)
    ax.plot([52.6], [yp], marker="x", color=GREY, ms=7, mew=1.8, zorder=5)
    cv.text(31, yp + 5.0, "planning_occ_2d_topic", size=7.4, color=MUTE, weight="bold", fit=(MX1, SX0, 0, H))
    cv.text(31, yp + 3.2, "nav_msgs/OccupancyGrid", size=7.2, color=MUTE, fit=(MX1, SX0, 0, H))
    cv.text(31, yp - 2.6, "unknown regions > 4 cells\nfilled as occupied\n(border regions exempt)", size=6.9,
            color=MUTE, va="top", fit=(MX1, SX0, 0, H))
    cv.text(31, yp - 12.0, "NO SUBSCRIBER\nsince Phase 5", size=7.4, color="#9a4a4a", weight="bold", va="top",
            fit=(MX1, SX0, 0, H))

    # ---- term_goal (selector -> planner) and planner_status (planner -> selector) -----------
    s5 = sel[4]
    p2 = pl[1]
    y_tg_out, y_st_in = s5[1] + s5[3] * 0.70, s5[1] + s5[3] * 0.22
    y_tg_in, y_st_out = p2[1] + p2[3] * 0.80, p2[1] + p2[3] * 0.30
    lane_tg, lane_st = 132.0, 176.0
    cv.arrow_path([(bx + bw, y_tg_out), (lane_tg, y_tg_out), (lane_tg, y_tg_in), (px0, y_tg_in)], S, lw=2.0)
    cv.arrow_path([(px0, y_st_out), (lane_st, y_st_out), (lane_st, y_st_in), (bx + bw, y_st_in)], PL, lw=2.0)
    cv.text(133, y_tg_in + 2.4, "term_goal", size=9.5, color=S, weight="bold", bg=True)
    cv.text(147, y_tg_in + 2.4, "geometry_msgs/PoseStamped", size=8, bg=True)
    cv.text(133, y_tg_in + 5.9, "stamp rule: ONE new, strictly increasing stamp per commitment;\n"
            "the planner echoes it as goal_stamp in planner_status", size=7.2, color=MUTE, va="center", bg=True)
    cv.text(152, y_st_in - 2.2, "planner_status", size=9.5, color=PL, weight="bold", ha="center", bg=True)
    cv.text(152, y_st_in + 2.3, "goal_selector_msgs/PlannerStatus  reliable, one per replan tick (+ REACHED)",
            size=7.6, ha="center", bg=True)

    # status table, between the two lanes
    action = {
        "SUCCESS": ("global plan reached the goal", "reset fail counter"),
        "PARTIAL": ("A* fell back to best node", "reset fail counter"),
        "FAILED": ("global plan failed", f"+1; {p('exploration.manager.unreachable_consec_thresh')} in a row: "
                                          "INVALIDATE (manual: warn)"),
        "SKIPPED": ("replan did not run (not ready, at goal, yawing)", "neutral: neither counts nor resets"),
        "REACHED": (f"within goal_radius {p('goal_radius')} m", "frontier -> VISITED; manual goal released"),
    }
    if set(action) != set(statuses):
        raise SystemExit(f"PlannerStatus.msg constants {sorted(statuses)} differ from the drawn rows {sorted(action)}")
    tx0, tx1, ty1 = lane_tg + 5, lane_st - 4, 70.0
    row_h = 4.55
    ty0 = ty1 - 6.4 - row_h * len(action) - 4.0
    cv.rbox(tx0, ty0, tx1 - tx0, ty1 - ty0, fc="white", ec=GREY, lw=1.0)
    tb = (tx0, tx1, ty0, ty1)
    cv.text(tx0 + 1.2, ty1 - 2.0, "how the selector uses each status", size=8, weight="bold", fit=tb)
    cv.text(tx0 + 1.2, ty1 - 4.5, "only FAILED counts toward the threshold", size=7, color=MUTE, fit=tb)
    y = ty1 - 8.6
    for name in sorted(statuses, key=statuses.get):
        what, use = action[name]
        cv.text(tx0 + 1.2, y, f"{name}", size=7.4, weight="bold", color=PL, fit=tb)
        cv.text(tx0 + 10.8, y, what, size=6.6, color=MUTE, fit=tb)
        cv.text(tx0 + 10.8, y - 1.85, use, size=6.9, fit=tb)
        y -= row_h
    cv.text(tx0 + 1.2, ty0 + 1.7, "ignored unless goal_stamp == current stamp (0 = no goal)", size=6.6,
            color=MUTE, style="italic", fit=tb)

    # ---- side inputs / outputs (de-emphasised) ----------------------------------------------
    def side_box(x0, w, title, sub):
        cv.rbox(x0, 1.2, w, 7.4, fc=tint(SD, 0.07), ec=SD, lw=1.0, ls=(0, (3, 2)), rs=1.2)
        cv.text(x0 + w / 2, 6.0, title, size=7.8, color=MUTE, weight="bold", ha="center", fit=(x0, x0 + w, 1.2, 8.6))
        cv.text(x0 + w / 2, 3.2, sub, size=6.8, color=MUTE, ha="center", fit=(x0, x0 + w, 1.2, 8.6))

    side_box(60, 34, "RViz 2D Goal Pose", "operator laptop")
    cv.arrow_path([(77, 8.6), (77, s5[1])], SD, lw=1.2)
    cv.text(79, 12.1, "term_goal_rviz", size=7.8, color=MUTE, weight="bold", bg=True)
    cv.text(79, 10.0, "geometry_msgs/PoseStamped", size=7, color=MUTE, bg=True)

    # visualisation outputs (selector -> RViz), small and de-emphasised
    side_box(97, 22, "RViz displays", "map + goal point")
    for xv in (101.5, 112.5):
        cv.arrow_path([(xv, BY0), (xv, 8.6)], SD, lw=1.0, ls=(0, (3, 2)))
    cv.text(100.5, 14.6, "selector_map_2d (1 Hz)", size=6.2, color=MUTE, ha="left", bg=True)
    cv.text(100.5, 11.6, "point_selector_goal", size=6.2, color=MUTE, ha="left", bg=True)

    side_box(134, 36, "convert_odom_to_state", "publishes state")
    xs_in, xp_in = 121, 190
    cv.arrow_path([(134, 4.9), (xs_in, 4.9), (xs_in, BY0)], SD, lw=1.2)
    cv.arrow_path([(170, 4.9), (xp_in, 4.9), (xp_in, BY0)], SD, lw=1.2)
    cv.text(150, 11.2, "state  dynus_interfaces/State", size=7.2, color=MUTE, weight="bold", ha="center", bg=True)
    cv.text(150, 9.4, "robot pose to selector and planner", size=6.8, color=MUTE, ha="center", bg=True)

    side_box(204, 34, "MPC controller", "consumes the trajectory")
    xm = px0 + bw / 2
    cv.arrow_path([(xm, pl[3][1]), (xm, 8.6)], PL, lw=1.6)
    cv.text(xm + 1.5, 11.8, "mpc_waypoints", size=8, color=PL, weight="bold", bg=True)
    cv.text(xm + 1.5, 10.0, "dynus_interfaces/SpeedyPath", size=6.8, bg=True)

    cv.text(2, 7.4, "Legend", size=7.8, color=MUTE, weight="bold")
    cv.text(2, 5.0, "solid arrow: topic with a subscriber\ndashed grey: published, nobody subscribes\n"
            "dashed boxes: side inputs / consumers", size=6.8, color=MUTE, va="top", fit=(0, 58, 0, 9))
    cv.check_fits()
    return cv


def render():
    plt.rcParams["svg.hashsalt"] = "mighty-core-loop"
    plt.rcParams["svg.fonttype"] = "path"
    plt.rcParams["font.family"] = "DejaVu Sans"
    nodes, _panes, _launch = gad.build_model()
    verify_edges(nodes)
    cv = build(P(load_params()), read_statuses())
    return cv


def save_bytes(cv):
    buf = io.BytesIO()
    cv.fig.savefig(buf, format="svg", facecolor=BG, metadata={"Date": None})
    return buf.getvalue()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="exit 1 if the committed SVG differs from a fresh render")
    args = ap.parse_args(argv)
    cv = render()
    svg = save_bytes(cv)
    if args.check:
        if not OUT_SVG.exists() or OUT_SVG.read_bytes() != svg:
            print(f"STALE: {OUT_SVG.relative_to(ROOT)} differs from the regenerated diagram; "
                  f"run python3 scripts/gen_core_loop_diagram.py", file=sys.stderr)
            return 1
        print("up to date")
        return 0
    OUT_SVG.write_bytes(svg)
    cv.fig.savefig(OUT_PNG, format="png", dpi=170, facecolor=BG, metadata={"Software": None})
    print(f"wrote {OUT_SVG.relative_to(ROOT)} and {OUT_PNG.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
