#!/usr/bin/env python3
"""Bag test (3) and the roll-up: parse the recorded logs of a bag-test run and merge the
monitor reports into one summary (REPORT.md + summary.json).

Pure Python 3 (no ROS needed), runs on the host or in the container.

INPUT: a run directory written by docker/dev/replay/run_bag_tests.sh:
    <run>/logs/*.log                   tmux pane captures of the planner, selector, MPC panes
                                       (or any text logs passed with --log)
    <run>/trajectory_vs_map.json       from monitor_trajectory_vs_map.py   (optional)
    <run>/selector_contract.json       from monitor_selector_contract.py   (optional)
    <run>/pass2.log                    pass2_laptop.sh output              (optional, echoed)

WHAT IT COUNTS FROM THE LOGS (selector messages: src/goal_selector/goal_selector.cpp)
    frontier commits       "Exploration: -> frontier <id> at (x, y)"                  [INFO]
    invalidations          by logged reason: "unreachable after N HGP failures",
                           "frontier <id> stuck"                                       [INFO]
                           (pursuit-timeout and occupied-band invalidations, and VISITED, are NOT logged by
                           the selector; they cannot be counted from logs. Use the number of commits vs
                           exploration/current_goal for a cross-check.)
    preemptions            "preempting frontier"                                       [INFO]
    return-home events     "No frontiers left ... Returning to captured start",
                           "Return-home: heading", "return-home complete"              [INFO]
    manual goals           "Manual goal (...) committed", "... start timeout ...", "... stuck ...",
                           "... unreachable after N consecutive planner failures"       [INFO]
    relocation lines       any line matching /relocat/i in ANY log                     [PASS if 0, FAIL otherwise]
                           (spec N5/§11.15 "No relocation anywhere", notes item 3 "must be 0")
    other WARN/ERROR       counts of "[WARN]" / "[ERROR]" lines per log file, with the 5 most frequent
                           messages (digits normalised)                                [INFO]

USAGE
    python3 summarize_run.py <run_dir>            # writes <run_dir>/REPORT.md and summary.json
    python3 summarize_run.py --log a.log --log b.log     # logs only
OUTPUT: the Markdown report is also printed. Exit code 1 if any monitor or the relocation check FAILed.
"""
import argparse
import glob
import json
import os
import re
import sys
from collections import Counter

ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')


def read_logs(paths):
    out = {}
    for p in paths:
        try:
            with open(p, errors='replace') as f:
                out[p] = [ANSI.sub('', ln.rstrip('\n')) for ln in f]
        except OSError as e:
            out[p] = [f'<unreadable: {e}>']
    return out


PATTERNS = [
    ('frontier_commits', r'Exploration: -> frontier'),
    ('inv_unreachable', r'frontier \d+ unreachable after'),
    ('inv_stuck', r'Exploration: frontier \d+ stuck'),
    ('preemptions', r'Exploration: preempting frontier'),
    ('no_frontiers_left', r'No frontiers left'),
    ('return_home_heading', r'Return-home: heading'),
    ('return_home_complete', r'return-home complete'),
    ('return_home_ignored', r'Return-home trigger received'),
    ('manual_committed', r'Manual goal \(.*\) committed'),
    ('manual_start_timeout', r'Manual goal .*start timeout'),
    ('manual_stuck', r'Manual goal .*stuck \(no motion'),
    ('manual_unreachable_kept', r'Manual goal .*unreachable after'),
    ('exploration_started', r'Exploration: starting from'),
]


def analyse_logs(logs):
    counts = Counter()
    reloc = []
    warn = {}
    for path, lines in logs.items():
        wc = Counter()
        for ln in lines:
            for key, pat in PATTERNS:
                if re.search(pat, ln):
                    counts[key] += 1
            if re.search(r'relocat', ln, re.I):
                reloc.append(f'{os.path.basename(path)}: {ln.strip()[:200]}')
            if '[WARN]' in ln or '[ERROR]' in ln:
                lvl = 'ERROR' if '[ERROR]' in ln else 'WARN'
                msg = re.sub(r'\d+(\.\d+)?', 'N', ln.split(']: ', 1)[-1])[:140]
                wc[(lvl, msg)] += 1
        warn[os.path.basename(path)] = wc
    return counts, reloc, warn


def md_table(rows, head):
    s = '| ' + ' | '.join(head) + ' |\n|' + '---|' * len(head) + '\n'
    for r in rows:
        s += '| ' + ' | '.join(str(c) for c in r) + ' |\n'
    return s


def main():
    ap = argparse.ArgumentParser(description='Summarise a goal-selector bag-test run', epilog=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir', nargs='?', default='')
    ap.add_argument('--log', action='append', default=[], help='extra log file (repeatable)')
    a = ap.parse_args()
    if not a.run_dir and not a.log:
        ap.error('give a run directory or at least one --log')
    logs_paths = list(a.log)
    if a.run_dir:
        logs_paths += sorted(glob.glob(os.path.join(a.run_dir, 'logs', '*.log')))
    logs = read_logs(logs_paths)
    counts, reloc, warn = analyse_logs(logs)

    md = ['# Goal selector bag-test report\n']
    fails = []

    def load(name):
        p = os.path.join(a.run_dir, name) if a.run_dir else ''
        if p and os.path.exists(p):
            with open(p) as f:
                return json.load(f)
        return None

    # ---- roll-up of the monitors
    rows = []
    for fname in ('trajectory_vs_map.json', 'selector_contract.json'):
        r = load(fname)
        if r is None:
            rows.append((fname, '(missing)', '', ''))
            continue
        for c in r['checks']:
            rows.append((r['title'][:28], c['name'], c['verdict'], c['summary'][:230]))
            if c['verdict'] == 'FAIL':
                fails.append(f"{c['name']}: {c['summary']}")
    md.append('## Monitor checks\n')
    md.append(md_table(rows, ['monitor', 'check', 'verdict', 'summary']))

    # ---- trajectory numbers
    tv = load('trajectory_vs_map.json')
    if tv:
        D = tv.get('data', {})
        md.append('\n## Trajectory / prefix vs map (counts)\n')
        md.append(md_table(
            [(ch, d['evaluated'], d['distinct'], d['samples'], f"{d['msgs_unknown']} ({d['distinct_unknown']})",
              f"{d['msgs_occ']} ({d['distinct_occ']})", f"{d['msgs_run_ge_trim']} ({d['distinct_run_ge_trim']})",
              d['max_unknown_run_cells']) for ch, d in D.items()],
            ['channel', 'messages', 'distinct', 'samples', 'unknown: msgs (distinct)',
             'OCCUPIED: msgs (distinct)', 'run>=trim: msgs (distinct)', 'longest unknown run (cells)']))
    sc = load('selector_contract.json')
    if sc:
        d = sc.get('data', {})
        md.append('\n## Planner statuses\n')
        md.append(md_table([(k, v) for k, v in d.get('status_counts', {}).items()], ['status', 'count']))
        md.append(f"\nzero-stamp statuses: {d.get('zero_stamp_statuses')}, stale-goal statuses: "
                  f"{d.get('stale_statuses')}, FAILED bursts: {d.get('failed_bursts')} "
                  f"(>= threshold: {d.get('failed_bursts_ge_thresh')}), term_goals: {d.get('term_goals')}, "
                  f"exploration/current_goal msgs: {d.get('current_goals')}, goal_reached msgs: "
                  f"{d.get('goal_reached_msgs')}\n")

    # ---- log totals
    md.append('\n## Log-derived regression totals\n')
    if not logs:
        md.append('_no logs found; run_bag_tests.sh captures the tmux panes into <run>/logs/_\n')
    else:
        md.append(f"logs: {', '.join(os.path.basename(p) for p in logs)}\n\n")
        names = {
            'frontier_commits': 'frontier commits', 'inv_unreachable': 'invalidated: unreachable (FAILED x N)',
            'inv_stuck': 'invalidated: stuck watchdog', 'preemptions': 'preemptions',
            'no_frontiers_left': 'return home: "No frontiers left"', 'return_home_heading': 'return home: heading',
            'return_home_complete': 'return home: complete', 'return_home_ignored': 'return-home trigger msgs',
            'manual_committed': 'manual goals committed', 'manual_start_timeout': 'manual: start timeout release',
            'manual_stuck': 'manual: stuck release', 'manual_unreachable_kept': 'manual: unreachable (kept)',
            'exploration_started': 'exploration started'}
        md.append(md_table([(names[k], counts.get(k, 0)) for k, _ in PATTERNS], ['event', 'count']))
        verdict = 'FAIL' if reloc else 'PASS'
        md.append(f"\n**relocation log lines (must be 0): {len(reloc)} -> {verdict}**\n")
        for r in reloc[:20]:
            md.append(f'- `{r}`\n')
        if reloc:
            fails.append(f'{len(reloc)} log line(s) mention relocation')
        md.append('\n### WARN/ERROR lines per log (top 5 each)\n')
        for f, wc in warn.items():
            tot = sum(wc.values())
            md.append(f'- `{f}`: {tot} WARN/ERROR lines\n')
            for (lvl, msg), n in wc.most_common(5):
                md.append(f'    - {n}x [{lvl}] {msg}\n')

    p2 = os.path.join(a.run_dir, 'pass2.log') if a.run_dir else ''
    if p2 and os.path.exists(p2):
        md.append('\n## pass2_laptop.sh output (tail)\n```\n')
        md.append(ANSI.sub('', ''.join(open(p2, errors='replace').readlines()[-25:])))
        md.append('```\n')

    md.append('\n## Overall\n')
    md.append('**FAIL** (see list)\n' if fails else '**no FAIL check** (INFO items need reading)\n')
    for f in fails:
        md.append(f'- {f}\n')
    text = ''.join(md)
    print(text)
    if a.run_dir:
        with open(os.path.join(a.run_dir, 'REPORT.md'), 'w') as f:
            f.write(text)
        with open(os.path.join(a.run_dir, 'summary.json'), 'w') as f:
            json.dump({'log_counts': dict(counts), 'relocation_lines': reloc, 'fails': fails}, f, indent=1)
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
