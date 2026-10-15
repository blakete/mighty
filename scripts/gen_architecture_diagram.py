#!/usr/bin/env python3
"""Generate docs/goal_selector_architecture.md (Mermaid diagram + topic table) from the code.

Python 3 standard library only.  Usage:

    python3 scripts/gen_architecture_diagram.py              # (re)write the output
    python3 scripts/gen_architecture_diagram.py --check      # exit 1 if the committed output is stale
    python3 scripts/gen_architecture_diagram.py --runtime    # also compare with `ros2 node info`
    python3 scripts/gen_architecture_diagram.py --output X   # write/check X instead of the default

What is read (nothing is hand-drawn):
  * docker/mighty_hw.sh               which launch nodes run (one tmux pane each), and with which launch args
  * launch/onboard_mighty.launch.py   parsed with `ast`: Node() actions, remappings, and the parameter
                                      YAML files each node loads (data-flow through the variables, conditions
                                      evaluated with the hardware launch args)
  * CMakeLists.txt                    which sources make up each executable (add_executable / add_library /
                                      target_link_libraries), so only code that runs in that process is scanned
  * the C++ sources of those targets  create_publisher / create_subscription / create_client / create_service,
                                      message_filters ::subscribe, tf2 listeners, declare_parameter; topic name,
                                      message type and QoS are extracted statically; enclosing `if`/`else` chains
                                      and early-`return` guards are recorded as conditions
  * docs/external_nodes.yaml          the only hand-maintained input (nodes whose code is not in this package)

Static-extraction limits are listed in the "How this is generated" section of the output.
"""

import argparse
import ast
import difflib
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "docs" / "goal_selector_architecture.md"
EXTERNAL_YAML = ROOT / "docs" / "external_nodes.yaml"
LAUNCH_FILE = ROOT / "launch" / "onboard_mighty.launch.py"
HW_SCRIPT = ROOT / "docker" / "mighty_hw.sh"
CMAKE_FILE = ROOT / "CMakeLists.txt"

# Parsed (in-package) executables are only scanned if the hardware stack launches them.
SELECTOR_EXE = "goal_selector"

# Message types drawn as "visualization" when nothing subscribes to them (collapsed into one edge).
VIZ_TYPE_PREFIXES = ("visualization_msgs/", "decomp_ros_msgs/")
VIZ_TYPES = {"sensor_msgs/PointCloud2", "geometry_msgs/PointStamped"}


def rel(path):
    try:
        return str(Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(path)


# =============================================================================================
# Minimal YAML-subset parser (for docs/external_nodes.yaml); subset documented in that file.
# =============================================================================================
class YamlError(Exception):
    pass


def _strip_comment(line):
    """Return (text_without_comment, comment_text_or_None); '#' inside quotes is not a comment."""
    quote = None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            return line[:i].rstrip(), line[i + 1:].strip()
    return line.rstrip(), None


def _scalar(text):
    text = text.strip()
    if text == "":
        return None
    if text[0] in "\"'" and text[-1] == text[0] and len(text) >= 2:
        return text[1:-1]
    if text.startswith("[") and text.endswith("]"):
        inner = text[1:-1].strip()
        if not inner:
            return []
        parts, cur, quote, depth = [], "", None, 0
        for ch in inner:
            if quote:
                cur += ch
                if ch == quote:
                    quote = None
            elif ch in "\"'":
                quote = ch
                cur += ch
            elif ch == "," and depth == 0:
                parts.append(cur)
                cur = ""
            else:
                cur += ch
        parts.append(cur)
        return [_scalar(p) for p in parts]
    low = text.lower()
    if low in ("true", "false"):
        return low == "true"
    if low in ("null", "~"):
        return None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        pass
    return text


def parse_yaml_subset(text):
    lines = []
    for raw in text.splitlines():
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise YamlError("tabs are not allowed for indentation")
        body, comment = _strip_comment(raw)
        if not body.strip():
            continue
        indent = len(body) - len(body.lstrip(" "))
        lines.append([indent, body.strip(), bool(comment and "unverified" in comment.lower())])
    pos = [0]

    def block(indent):
        if pos[0] >= len(lines):
            return None
        if lines[pos[0]][1].startswith("- ") or lines[pos[0]][1] == "-":
            return parse_list(indent)
        return parse_map(indent)

    def parse_list(indent):
        out = []
        while pos[0] < len(lines) and lines[pos[0]][0] == indent and (
                lines[pos[0]][1].startswith("- ") or lines[pos[0]][1] == "-"):
            line = lines[pos[0]]
            rest = line[1][1:].lstrip()
            if not rest:
                pos[0] += 1
                out.append(block(lines[pos[0]][0]) if pos[0] < len(lines) and lines[pos[0]][0] > indent else None)
            elif re.match(r"^[\w.\-/]+\s*:(\s|$)", rest) and not rest.startswith(("\"", "'")):
                # "- key: value": turn the item into a mapping indented by the "- " width
                line[0] = indent + (len(line[1]) - len(rest))
                line[1] = rest
                out.append(parse_map(line[0]))
            else:
                out.append(_scalar(rest))
                pos[0] += 1
        return out

    def parse_map(indent):
        out = {}
        while pos[0] < len(lines) and lines[pos[0]][0] == indent and not lines[pos[0]][1].startswith("- "):
            ind, body, unverified = lines[pos[0]]
            m = re.match(r"^([\w.\-/]+)\s*:(?:\s+(.*))?$", body)
            if not m:
                raise YamlError("cannot parse line: %r" % body)
            key, val = m.group(1), m.group(2)
            if unverified:
                out["__unverified__"] = True
            pos[0] += 1
            if val is None or val == "":
                if pos[0] < len(lines) and lines[pos[0]][0] > indent:
                    out[key] = block(lines[pos[0]][0])
                elif (pos[0] < len(lines) and lines[pos[0]][0] == indent
                      and lines[pos[0]][1].startswith("- ")):
                    out[key] = parse_list(indent)  # list at the same indent as its key
                else:
                    out[key] = None
            else:
                out[key] = _scalar(val)
        return out

    if not lines:
        return {}
    result = block(lines[0][0])
    if pos[0] != len(lines):
        raise YamlError("unparsed content near: %r" % lines[pos[0]][1])
    return result


# =============================================================================================
# C++ source helpers
# =============================================================================================
def strip_comments(src):
    """Blank out // and /* */ comments, keeping string literals and all offsets/newlines."""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if c == '"' or c == "'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            out.append(src[i:j + 1])
            i = j + 1
        elif src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join("\n" if ch == "\n" else " " for ch in src[i:j]))
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def mask_strings(src):
    """Same length as src, string/char literal contents replaced by spaces."""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if c == '"' or c == "'":
            j = i + 1
            while j < n and src[j] != c:
                j += 2 if src[j] == "\\" else 1
            out.append(c + " " * (j - i - 1) + (c if j < n else ""))
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)[:n]


def balanced(text, start, open_ch="(", close_ch=")"):
    """text[start] == open_ch; return index of the matching close (string-aware, caller passes masked)."""
    depth = 0
    for i in range(start, len(text)):
        if text[i] == open_ch:
            depth += 1
        elif text[i] == close_ch:
            depth -= 1
            if depth == 0:
                return i
    return -1


def split_top(text_orig, text_masked, seps=","):
    """Split at top-level separators (outside (), {}, []), using the masked text to find them."""
    parts, depth, last = [], 0, 0
    for i, ch in enumerate(text_masked):
        if ch in "({[":
            depth += 1
        elif ch in ")}]":
            depth -= 1
        elif ch in seps and depth == 0:
            parts.append(text_orig[last:i].strip())
            last = i + 1
    parts.append(text_orig[last:].strip())
    return [p for p in parts if p != ""] if seps == "," else parts


def line_of(src, offset):
    return src.count("\n", 0, offset) + 1


def strip_parens(cond):
    cond = cond.strip()
    while cond.startswith("(") and balanced(mask_strings(cond), 0) == len(cond) - 1:
        cond = cond[1:-1].strip()
    return cond


def neg(cond):
    cond = strip_parens(cond)
    if cond.startswith("!") and not cond.startswith("!="):
        inner = strip_parens(cond[1:])
        if inner.startswith("(") or re.fullmatch(r"[\w:.>\-]+", inner):
            return inner
        if balanced(mask_strings("(" + inner + ")"), 0) == len(inner) + 1:
            return inner
    return "!" + cond if re.fullmatch(r"[\w:.>\-]+", cond) else "!(%s)" % cond


# ---------------------------------------------------------------------------------------------
# Scope / condition analysis
# ---------------------------------------------------------------------------------------------
class Scope:
    def __init__(self, kind, cond=None, prior=None, saved_paren=0):
        self.kind = kind                # 'if' | 'else' | 'block'
        self.cond = cond                # condition text of an if / else-if branch
        self.prior = prior or []        # conditions of the earlier branches of the chain (all false here)
        self.saved_paren = saved_paren
        self.guards = []                # conditions known true after an early-return `if (...) {...return;}`
        self.last_chain = []            # conditions of the most recent closed if-chain at this level
        self.has_return = False

    def effective(self):
        conds = [neg(p) for p in self.prior]
        if self.cond:
            conds.append(self.cond)
        return conds


def parse_if_head(head):
    """head like 'else if (cond)' / 'if (cond)' / 'else'. Return (is_else, cond or None, is_if)."""
    m = re.match(r"^\s*(else\s+)?if\s*(constexpr\s*)?\(", head)
    if m:
        open_idx = head.index("(", m.start())
        close = balanced(head, open_idx)
        if close < 0:
            return bool(m.group(1)), None, False
        return bool(m.group(1)), head[open_idx + 1:close].strip(), True
    if re.match(r"^\s*else\s*$", head):
        return True, None, False
    return False, None, False


def condition_map(src, queries):
    """For each query offset return the list of conditions that hold there (outermost first)."""
    masked = mask_strings(src)
    events = [(m.start(), m.group()) for m in re.finditer(r"[{};()]|\breturn\b", masked)]
    queries = sorted(queries)
    qi = 0
    result = {}
    stack = [Scope("block")]
    paren = 0
    last_delim = 0

    def snapshot(at):
        conds = []
        for sc in stack:
            conds.extend(sc.effective())
            conds.extend(sc.guards)
        head = src[last_delim:at]
        is_else, cond, is_if = parse_if_head(head)
        if is_if and cond:           # single-statement `if (c) pub = create_publisher(...)`
            conds_extra = []
            if is_else:
                conds_extra.extend(neg(p) for p in stack[-1].last_chain)
            conds_extra.append(cond)
            conds.extend(conds_extra)
        return conds

    for pos, tok in events:
        while qi < len(queries) and queries[qi] < pos:
            result[queries[qi]] = snapshot(queries[qi])
            qi += 1
        if tok == "(":
            paren += 1
        elif tok == ")":
            paren -= 1
        elif tok == "{":
            head = src[last_delim:pos]
            is_else, cond, is_if = parse_if_head(head) if paren == 0 else (False, None, False)
            parent = stack[-1]
            if is_if and cond:
                prior = list(parent.last_chain) if is_else else []
                sc = Scope("if", cond, prior, paren)
            elif is_else:
                sc = Scope("else", None, list(parent.last_chain), paren)
            else:
                sc = Scope("block", None, None, paren)
            sc.saved_paren = paren
            stack.append(sc)
            paren = 0
            last_delim = pos + 1
        elif tok == "}":
            if len(stack) > 1:
                sc = stack.pop()
                paren = sc.saved_paren
                parent = stack[-1]
                if sc.kind in ("if", "else"):
                    parent.last_chain = sc.prior + ([sc.cond] if sc.cond else [])
                    if sc.has_return and sc.kind == "if":
                        eff = sc.effective()
                        parent.guards.append(neg(eff[0] if len(eff) == 1 else
                                                 " && ".join("(%s)" % c for c in eff)))
                else:
                    parent.last_chain = []
            last_delim = pos + 1
        elif tok == ";":
            if paren == 0:
                stack[-1].last_chain = []
                last_delim = pos + 1
        elif tok == "return":
            stack[-1].has_return = True
    while qi < len(queries):
        result[queries[qi]] = snapshot(queries[qi])
        qi += 1
    return result


def pretty_cond(c):
    c = c.strip()
    c = re.sub(r"\bpar_\.", "", c)
    c = c.replace("this->", "")
    c = re.sub(r"\b(\w+?)_\b", r"\1", c)
    c = re.sub(r"\s+", " ", c)
    return c


# ---------------------------------------------------------------------------------------------
# QoS evaluation
# ---------------------------------------------------------------------------------------------
class QoSProfile:
    def __init__(self):
        self.depth = 10
        self.reliability = "reliable"
        self.durability = "volatile"
        self.note = None

    def text(self):
        s = "%s, %s, keep_last %s" % (self.reliability, self.durability, self.depth)
        return s + (" (%s)" % self.note if self.note else "")


def parse_chain(expr):
    """'a::B(x).c().d(y)' -> [('a::B','x'), ('c',''), ('d','y')]; None when it is not a call chain."""
    masked = mask_strings(expr)
    i, n, out = 0, len(expr), []
    while i < n:
        while i < n and expr[i] in " \t\n":
            i += 1
        m = re.match(r"[A-Za-z_][\w:]*", expr[i:])
        if not m:
            return None
        name = m.group()
        i += m.end()
        while i < n and expr[i] in " \t\n":
            i += 1
        args = ""
        if i < n and expr[i] == "(":
            close = balanced(masked, i)
            if close < 0:
                return None
            args = expr[i + 1:close]
            i = close + 1
        elif out == [] and i >= n:
            return [(name, None)]
        else:
            return None
        out.append((name, args))
        while i < n and expr[i] in " \t\n":
            i += 1
        if i < n and expr[i] == ".":
            i += 1
        elif i < n and expr[i:i + 2] == "->":
            i += 2
        elif i < n:
            return None
    return out


def apply_qos_call(p, name, args):
    base = name.split("::")[-1]
    a = (args or "").strip()
    if base in ("QoS",):
        m = re.match(r"^(?:rclcpp::)?KeepLast\((\d+)\)$", a) or re.match(r"^(\d+)$", a)
        if m:
            p.depth = int(m.group(1))
        elif "from_rmw" in a:
            p.depth = 5 if "sensor_data" in a else p.depth
            p.note = "from_rmw copies only history+depth; reliability/durability stay default"
        else:
            return False
    elif base == "KeepLast":
        p.depth = int(a)
    elif base == "SensorDataQoS":
        p.depth, p.reliability, p.durability = 5, "best_effort", "volatile"
    elif base in ("SystemDefaultsQoS", "ParametersQoS", "ServicesQoS", "ClockQoS", "ParameterEventsQoS"):
        p.note = base
    elif base == "reliable":
        p.reliability = "reliable"
    elif base == "best_effort":
        p.reliability = "best_effort"
    elif base == "transient_local":
        p.durability = "transient_local"
    elif base in ("durability_volatile", "volatile"):
        p.durability = "volatile"
    elif base == "keep_last":
        p.depth = int(a) if a.isdigit() else p.depth
    elif base == "durability":
        p.durability = "transient_local" if "TransientLocal" in a else "volatile"
    elif base == "reliability":
        p.reliability = "best_effort" if "BestEffort" in a else "reliable"
    else:
        return False
    return True


def eval_qos(expr, src, pos, depth=0):
    """Return (text, profile-or-None).  `src` is the comment-stripped file, `pos` the call offset."""
    expr = expr.strip()
    if depth > 4:
        return "unresolved: " + expr, None
    if re.fullmatch(r"\d+", expr):
        p = QoSProfile()
        p.depth = int(expr)
        return p.text(), p
    if "rmw_qos_profile_sensor_data" in expr and "from_rmw" not in expr:
        p = QoSProfile()
        p.depth, p.reliability = 5, "best_effort"
        return p.text(), p
    chain = parse_chain(expr)
    if chain and chain[0][1] is not None:
        p = QoSProfile()
        ok = all(apply_qos_call(p, n, a) for n, a in chain)
        if ok:
            return p.text(), p
        return "unresolved: " + re.sub(r"\s+", " ", expr), None
    if re.fullmatch(r"[A-Za-z_]\w*", expr):
        # a variable: nearest earlier declaration, then chained mutators up to `pos`
        decl = None
        for m in re.finditer(r"(?:rclcpp::QoS|auto|rclcpp::SensorDataQoS)\s+%s\b\s*(?:\(([^;]*?)\)\s*;|=\s*([^;]+);)"
                             % re.escape(expr), src[:pos]):
            decl = m
        if not decl:
            return "unresolved: " + expr, None
        init = decl.group(1) if decl.group(1) is not None else decl.group(2)
        if decl.group(1) is not None and not decl.group().lstrip().startswith("auto"):
            init = "rclcpp::QoS(%s)" % decl.group(1)
        t, p = eval_qos(init, src, decl.start(), depth + 1)
        if p is None:
            return "unresolved: %s = %s" % (expr, init.strip()), None
        for m in re.finditer(r"\b%s((?:\s*\.\s*\w+\s*\([^()]*\))+)\s*;" % re.escape(expr),
                             src[decl.end():pos]):
            for name, args in parse_chain(m.group(1).lstrip(". ")) or []:
                apply_qos_call(p, name, args)
        return p.text(), p
    return "unresolved: " + re.sub(r"\s+", " ", expr), None


# ---------------------------------------------------------------------------------------------
# Topic-name resolution
# ---------------------------------------------------------------------------------------------
def render_string_expr(expr, src, pos, depth=0):
    """Render a C++ string expression: literals verbatim, other terms as <term>."""
    expr = expr.strip()
    masked = mask_strings(expr)
    parts = split_top(expr, masked, "+")
    out = []
    for part in parts:
        part = part.strip()
        m = re.fullmatch(r'((?:"(?:[^"\\]|\\.)*"\s*)+)', part)
        if m:
            out.append("".join(re.findall(r'"((?:[^"\\]|\\.)*)"', part)))
            continue
        inner = re.fullmatch(r"std::string\((.*)\)", part, re.S)
        if inner:
            part = inner.group(1).strip()
        if re.fullmatch(r"[A-Za-z_]\w*", part) and depth < 3:
            decl = None
            for mm in re.finditer(r"(?:const\s+)?(?:std::string|auto)\s+%s\s*=\s*([^;]+);" % re.escape(part),
                                  src[:pos]):
                decl = mm
            if decl:
                out.append(render_string_expr(decl.group(1), src, decl.start(), depth + 1))
                continue
        if part in ("ns_", "ns", "namespace_"):
            out.append("<ns>")
        elif part.startswith("this->get_namespace"):
            out.append("<ns>")
        else:
            out.append("<%s>" % re.sub(r"\s+", " ", part))
    return "".join(out)


def norm_type(t):
    t = t.strip().replace("::msg::", "/").replace("::srv::", "/").replace("::", "/")
    t = re.sub(r"/msg/", "/", t)
    return re.sub(r"\s+", "", t)


# ---------------------------------------------------------------------------------------------
# Extraction of pubs / subs / services / params from one C++ file
# ---------------------------------------------------------------------------------------------
class Endpoint:
    def __init__(self, direction, topic, mtype, qos_text, qos, file, line, conds, node, kind="topic"):
        self.direction = direction      # 'pub' | 'sub' | 'client' | 'service'
        self.topic = topic              # as written (before remaps)
        self.mtype = mtype
        self.qos_text = qos_text
        self.qos = qos
        self.file = file
        self.line = line
        self.conds = conds
        self.node = node
        self.kind = kind
        self.remapped_from = None
        self.active = True              # False when its condition is false for the hardware stack


CALL_RE = re.compile(r"\b(create_publisher|create_subscription|create_client|create_service)\s*(?=[<(])")
MF_SUB_RE = re.compile(r"\b(\w+)\s*\.\s*subscribe\s*\(\s*this\s*,")


def extract_file(path, node):
    raw = path.read_text()
    src = strip_comments(raw)
    masked = mask_strings(src)
    sites = []   # (offset, kind, template_type, args_orig, args_masked)
    for m in CALL_RE.finditer(src):
        i = m.end()
        tmpl = None
        if src[i] == "<":
            depth, j = 0, i
            while j < len(src):
                if src[j] == "<":
                    depth += 1
                elif src[j] == ">":
                    depth -= 1
                    if depth == 0:
                        break
                j += 1
            tmpl = src[i + 1:j]
            i = j + 1
            while i < len(src) and src[i] in " \t\n":
                i += 1
        if i >= len(src) or src[i] != "(":
            continue
        close = balanced(masked, i)
        sites.append((m.start(), m.group(1), tmpl, src[i + 1:close], masked[i + 1:close]))
    for m in MF_SUB_RE.finditer(src):
        i = m.end() - 1
        # find the '(' that opened this call
        open_idx = src.rfind("(", m.start(), m.end())
        close = balanced(masked, open_idx)
        member = m.group(1)
        decl = re.search(r"message_filters::Subscriber<\s*([\w:]+)\s*>\s+%s\b" % re.escape(member),
                         "".join(strip_comments(p.read_text()) for p in (ROOT / "include").rglob("*.hpp")))
        sites.append((m.start(), "mf_subscribe", decl.group(1) if decl else None,
                      src[open_idx + 1:close], masked[open_idx + 1:close]))
    conds = condition_map(src, [s[0] for s in sites]) if sites else {}
    endpoints = []
    for off, kind, tmpl, args, margs in sites:
        parts = split_top(args, margs)
        if kind == "mf_subscribe":
            parts = parts[1:]           # drop `this`
        if not parts:
            continue
        topic_expr = parts[0]
        qos_expr = None
        if kind in ("create_publisher", "create_subscription", "mf_subscribe") and len(parts) > 1:
            qos_expr = parts[1]
        if kind == "create_client" and len(parts) > 1:
            qos_expr = parts[1]
        topic = render_string_expr(topic_expr, src, off)
        if qos_expr is None:
            qtext, qos = ("services default" if kind in ("create_client", "create_service") else "unresolved: (none)"), None
        else:
            qtext, qos = eval_qos(qos_expr, src, off)
        direction = {"create_publisher": "pub", "create_subscription": "sub", "mf_subscribe": "sub",
                     "create_client": "client", "create_service": "service"}[kind]
        ep = Endpoint(direction, topic, norm_type(tmpl) if tmpl else "?", qtext, qos,
                      rel(path), line_of(src, off), [pretty_cond(c) for c in conds.get(off, [])], node,
                      "service" if direction in ("client", "service") else "topic")
        endpoints.append(ep)
    # tf2 listeners / broadcasters are not create_* calls: record them as implicit endpoints
    for m in re.finditer(r"\btf2_ros::TransformListener\b[^;]*?\(", src):
        endpoints.append(Endpoint("sub", "/tf", "tf2_msgs/TFMessage", "tf2 default (via TransformListener)", None,
                                  rel(path), line_of(src, m.start()), [], node))
        endpoints.append(Endpoint("sub", "/tf_static", "tf2_msgs/TFMessage", "tf2 default (via TransformListener)",
                                  None, rel(path), line_of(src, m.start()), [], node))
        break
    for m in re.finditer(r"\btf2_ros::(?:Static)?TransformBroadcaster\b", src):
        static = "Static" in m.group()
        endpoints.append(Endpoint("pub", "/tf_static" if static else "/tf", "tf2_msgs/TFMessage",
                                  "tf2 default (via broadcaster)", None, rel(path), line_of(src, m.start()), [], node))
    params = re.findall(r"declare_parameter(?:<[^>]*>)?\s*\(\s*\"([^\"]+)\"", src)
    return endpoints, params


# ---------------------------------------------------------------------------------------------
# CMake: which sources make up an executable
# ---------------------------------------------------------------------------------------------
def parse_cmake(path):
    text = re.sub(r"#[^\n]*", "", path.read_text())
    targets, links = {}, {}
    for m in re.finditer(r"\b(add_executable|add_library)\s*\(", text):
        close = balanced(text, m.end() - 1)
        toks = text[m.end():close].split()
        if not toks:
            continue
        name = toks[0]
        srcs = [t for t in toks[1:] if t.endswith((".cpp", ".cc", ".c"))]
        targets.setdefault(name, []).extend(srcs)
    for m in re.finditer(r"\btarget_link_libraries\s*\(", text):
        close = balanced(text, m.end() - 1)
        toks = text[m.end():close].split()
        if toks:
            links.setdefault(toks[0], []).extend(toks[1:])

    def sources(name, seen=None):
        seen = seen or set()
        if name in seen or name not in targets:
            return []
        seen.add(name)
        out = list(targets[name])
        for dep in links.get(name, []):
            out.extend(sources(dep, seen))
        return out

    return {name: sources(name) for name in targets}


# =============================================================================================
# Launch file (ast) and hw script
# =============================================================================================
def parse_hw_script(path):
    """Return ([(title, only_nodes_key, tf_gated)], launch_args_dict)."""
    if not path.exists():
        return [], {}
    text = path.read_text()
    args = {}
    m = re.search(r"launch_base='([^']*)'", text)
    if m:
        for k, v in re.findall(r"(\w+):=(\S+)", m.group(1)):
            args[k] = v
    titles = []
    m = re.search(r"titles=\(([^)]*)\)", text)
    if m:
        titles = re.findall(r"'([^']*)'", m.group(1))
    cmds = []
    m = re.search(r"cmds=\((.*?)\n\s*\)\n", text, re.S)
    if m:
        for line in m.group(1).splitlines():
            km = re.search(r"only_nodes:=(\w+)", line)
            if km:
                cmds.append((km.group(1), "tf_gate" in line))
    panes = []
    for i, (key, gated) in enumerate(cmds):
        panes.append((titles[i] if i < len(titles) else key, key, gated))
    return panes, args


class LaunchModel:
    """Facts parsed out of onboard_mighty.launch.py."""

    def __init__(self, path, hw_args):
        self.nodes = {}      # launch name -> dict(package, executable, remaps, files)
        self.defaults = {}
        self.env = {}
        tree = ast.parse(path.read_text())
        for parent in ast.walk(tree):
            for child in ast.iter_child_nodes(parent):
                child.parent = parent
        self.tree = tree
        consts = {}
        for node in tree.body:
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
                for t in node.targets:
                    if isinstance(t, ast.Name):
                        consts[t.id] = node.value.value
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and getattr(node.func, "id", "") == "DeclareLaunchArgument"
                    and node.args and isinstance(node.args[0], ast.Constant)):
                dv = [k.value for k in node.keywords if k.arg == "default_value"]
                if dv and isinstance(dv[0], ast.Constant):
                    self.defaults[node.args[0].value] = dv[0].value
        self.args = dict(self.defaults)
        self.args.update(hw_args)
        self.args["namespace"] = "<ns>"
        setup = next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "launch_setup"), None)
        env = dict(consts)
        env["__builtins__"] = {"float": float, "int": int, "str": str, "bool": bool, "len": len, "radians": lambda x: x}

        outer = self

        class LC:
            def __init__(self, name):
                self.name = name

            def perform(self, ctx):
                return str(outer.args.get(self.name, ""))

        env["LaunchConfiguration"] = LC
        env["convert_str_to_bool"] = lambda s: s in ("true", "True", 1, "1", True)
        env["context"] = None
        self.env = env
        self.setup = setup
        if setup is None:
            return
        # evaluate the straight-line assignments we can (arguments -> python values)
        for st in setup.body:
            if isinstance(st, ast.Assign) and len(st.targets) == 1 and isinstance(st.targets[0], ast.Name):
                try:
                    env[st.targets[0].id] = eval(compile(ast.Expression(st.value), "<launch>", "eval"), env)
                except Exception:
                    pass
        self._collect_deps(setup)
        self._collect_nodes(setup)

    # -- helpers --------------------------------------------------------------------------
    def _eval(self, node):
        try:
            return eval(compile(ast.Expression(node), "<launch>", "eval"), self.env)
        except Exception:
            return None

    def _conditions(self, node):
        """Walk up the AST collecting (test_node, polarity) for enclosing If/IfExp."""
        out = []
        child = node
        parent = getattr(node, "parent", None)
        while parent is not None and parent is not self.setup:
            if isinstance(parent, (ast.If, ast.IfExp)):
                if child is parent.test:
                    pass
                else:
                    in_body = (child in parent.body) if isinstance(parent, ast.If) else (child is parent.body)
                    out.append((parent.test, in_body))
            child, parent = parent, getattr(parent, "parent", None)
        return out

    def _active(self, node):
        """True / False / None (unknown) for a node's enclosing conditions under the hw args."""
        result = True
        for test, pol in self._conditions(node):
            v = self._eval(test)
            if v is None and not isinstance(test, ast.Constant):
                try:
                    ast.unparse(test)
                except Exception:
                    pass
                return None if result is not False else False
            if bool(v) != pol:
                return False
        return result

    def _names(self, node):
        return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}

    def _yaml_consts(self, node):
        files = []
        for n in ast.walk(node):
            if isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value.endswith((".yaml", ".yml", ".json5")):
                files.append(n)
        return files

    def _collect_deps(self, setup):
        self.deps = {}      # var -> set(vars)
        self.var_files = {}  # var -> [(Constant node, pkg)]
        for n in ast.walk(setup):
            targets, value = [], None
            if isinstance(n, ast.Assign):
                targets = [t.id for t in n.targets if isinstance(t, ast.Name)]
                value = n.value
            elif isinstance(n, ast.Expr) and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Attribute) \
                    and n.value.func.attr == "update" and isinstance(n.value.func.value, ast.Name):
                targets = [n.value.func.value.id]
                value = n.value
            if not targets or value is None:
                continue
            names = self._names(value)
            # `with open(p) as f: x = load(f)`: f is a lexically scoped alias for p (the name f is reused)
            anc = getattr(n, "parent", None)
            while anc is not None and anc is not setup:
                if isinstance(anc, ast.With):
                    for item in anc.items:
                        if isinstance(item.optional_vars, ast.Name) and item.optional_vars.id in names:
                            names = (names - {item.optional_vars.id}) | self._names(item.context_expr)
                anc = getattr(anc, "parent", None)
            for t in targets:
                self.deps.setdefault(t, set()).update(names - {t})
                for c in self._yaml_consts(value):
                    pkg = "mighty"
                    for call in ast.walk(value):
                        if isinstance(call, ast.Call) and getattr(call.func, "id", "") == "get_package_share_directory" \
                                and call.args and isinstance(call.args[0], ast.Constant):
                            pkg = call.args[0].value
                    in_config = any(isinstance(x, ast.Constant) and x.value == "config" for x in ast.walk(value))
                    self.var_files.setdefault(t, []).append((c, pkg, in_config))

    def _closure(self, names):
        seen, todo = set(), list(names)
        while todo:
            v = todo.pop()
            if v in seen:
                continue
            seen.add(v)
            todo.extend(self.deps.get(v, ()))
        return seen

    def _collect_nodes(self, setup):
        for call in ast.walk(setup):
            if not (isinstance(call, ast.Call) and getattr(call.func, "id", "") == "Node"):
                continue
            kw = {k.arg: k.value for k in call.keywords}

            def const(k):
                v = kw.get(k)
                return v.value if isinstance(v, ast.Constant) else None

            name = const("name") or const("executable")
            if name is None:
                continue
            remaps = []
            if isinstance(kw.get("remappings"), (ast.List, ast.Tuple)):
                for el in kw["remappings"].elts:
                    if isinstance(el, ast.Tuple) and len(el.elts) == 2:
                        a, b = self._eval(el.elts[0]), self._eval(el.elts[1])
                        if a is None and isinstance(el.elts[0], ast.Constant):
                            a = el.elts[0].value
                        remaps.append((a, b if b is not None else "<%s>" % ast.unparse(el.elts[1])))
            files = []
            if "parameters" in kw:
                names = self._names(kw["parameters"])
                found = list(self._yaml_consts(kw["parameters"]))
                direct = [(c, "mighty", False) for c in found]
                inherited = []
                for v in self._closure(names):
                    inherited.extend(self.var_files.get(v, []))
                seen_ids = set()
                for c, pkg, in_config in direct + inherited:
                    if id(c) in seen_ids:
                        continue
                    seen_ids.add(id(c))
                    active = self._active(c)
                    files.append(dict(name=c.value, pkg=pkg, line=c.lineno, active=active,
                                      cond=self._cond_text(c)))
                files.sort(key=lambda f: f["line"])
            self.nodes[name] = dict(package=const("package"), executable=const("executable"), remaps=remaps,
                                    files=files, line=call.lineno)

    def _cond_text(self, node):
        parts = []
        for test, pol in reversed(self._conditions(node)):
            t = ast.unparse(test)
            parts.append(t if pol else "not (%s)" % t)
        return " and ".join(parts)


# =============================================================================================
# Model assembly
# =============================================================================================
class Node:
    def __init__(self, name, label, process, kind):
        self.name = name
        self.label = label
        self.process = process      # subgraph title
        self.kind = kind            # 'parsed' | 'external'
        self.endpoints = []
        self.files = []             # parameter files (dicts)
        self.declared = []
        self.remaps = []
        self.exe = None
        self.package = None
        self.unverified = False
        self.group = None
        self.ros_name = None
        self.sources = []
        self.viz_sink = False
        self.pane = None
        self.gated = False


def build_model():
    ext = parse_yaml_subset(EXTERNAL_YAML.read_text()) if EXTERNAL_YAML.exists() else {"nodes": []}
    panes, hw_args = parse_hw_script(HW_SCRIPT)
    launch = LaunchModel(LAUNCH_FILE, hw_args)
    cmake = parse_cmake(CMAKE_FILE)

    nodes = {}
    launched_keys = [p[1] for p in panes] or list(launch.nodes)
    pane_by_key = {p[1]: p for p in panes}
    ext_by_launch = {e.get("launch_name"): e for e in ext.get("nodes", []) if e.get("launch_name")}

    for key in launched_keys:
        ln = launch.nodes.get(key)
        if ln is None:
            continue
        if key in ext_by_launch:
            continue        # described in external_nodes.yaml, only launch facts merged below
        if ln["package"] != "mighty":
            continue
        exe = ln["executable"]
        n = Node(key, key, None, "parsed")
        n.exe, n.package, n.ros_name = exe, ln["package"], key
        pane = pane_by_key.get(key)
        n.pane = pane[0] if pane else None
        n.gated = bool(pane and pane[2])
        n.files = [f for f in ln["files"]]
        n.remaps = ln["remaps"]
        srcs = [ROOT / s for s in cmake.get(exe, [])]
        n.sources = [rel(s) for s in srcs]
        for s in srcs:
            if not s.exists():
                continue
            eps, params = extract_file(s, key)
            n.endpoints.extend(eps)
            n.declared.extend(params)
        nodes[key] = n
    # the selector reads the planner's merged parameters too (spec 4.5); that is already in its Node(parameters=...) closure

    for e in ext.get("nodes", []):
        n = Node(e["name"], e.get("label", e["name"]), None, "external")
        n.group = e.get("group", "External")
        n.unverified = bool(e.get("__unverified__"))
        n.ros_name = e.get("ros_name")
        n.viz_sink = bool(e.get("displays_unconsumed_viz"))
        ln = launch.nodes.get(e.get("launch_name")) if e.get("launch_name") else None
        if ln:
            n.files = ln["files"]
            n.remaps = ln["remaps"]
            n.package = ln["package"]
            n.exe = ln["executable"]
            pane = pane_by_key.get(e["launch_name"])
            n.pane = pane[0] if pane else None
            n.gated = bool(pane and pane[2])
            n.ros_name = e["launch_name"]
        for direction, lst in (("pub", e.get("publishes") or []), ("sub", e.get("subscribes") or [])):
            for t in lst:
                qos_text = t.get("qos") or "unspecified"
                ep = Endpoint(direction, str(t["topic"]), norm_type(str(t.get("type", "?"))), qos_text,
                              None, rel(EXTERNAL_YAML), 0, [], n.name)
                ep.note = t.get("note")
                ep.unverified = bool(t.get("__unverified__"))
                ep.qos = qos_from_text(t.get("qos"))
                n.endpoints.append(ep)
        nodes[n.name] = n
    return nodes, panes, launch


def qos_from_text(text):
    if not text:
        return None
    p = QoSProfile()
    low = str(text).lower()
    p.reliability = "best_effort" if "best_effort" in low else ("reliable" if "reliable" in low else None)
    p.durability = "transient_local" if "transient" in low else ("volatile" if "volatile" in low else None)
    m = re.search(r"keep_last\s*(\d+)", low)
    p.depth = int(m.group(1)) if m else None
    return p


# -- hardware-scope evaluation of C++ conditions ------------------------------------------------
def hw_context(nodes, launch):
    """Scalar facts for evaluating conditions: launch args + scalars from the planner's YAML layers."""
    ctx = {"use_hardware": True, "vehicle_type": "ground_robot"}
    layers = []
    for n in nodes.values():
        if n.exe == "mighty":
            layers = n.files
    for f in layers:
        if f.get("active") is False:
            continue
        p = ROOT / "config" / f["name"]
        if not p.exists():
            continue
        for line in p.read_text().splitlines():
            m = re.match(r"^\s+([A-Za-z_][\w.]*):\s*([^#\s][^#]*?)\s*(?:#.*)?$", line)
            if m and m.group(1) != "ros__parameters":
                v = m.group(2).strip().strip("\"'")
                ctx[m.group(1)] = {"true": True, "false": False}.get(v.lower(), v)
    ctx["use_hardware"] = True
    ctx["vehicle_type"] = "ground_robot"
    return ctx


def eval_cond(cond, ctx):
    """True / False / None for a pretty condition string made of simple atoms joined by && / ||."""
    cond = cond.strip()
    if cond.startswith("(") and balanced(cond, 0) == len(cond) - 1:
        return eval_cond(cond[1:-1], ctx)
    masked = mask_strings(cond)
    for op, fn in (("||", any), ("&&", all)):
        parts = split_top(cond, masked, "|") if False else None
    # split on top-level || then &&
    for op in ("||", "&&"):
        idx, depth, parts, last = 0, 0, [], 0
        for i, ch in enumerate(masked):
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
            elif depth == 0 and masked.startswith(op, i):
                parts.append(cond[last:i])
                last = i + 2
        if parts:
            parts.append(cond[last:])
            vals = [eval_cond(p, ctx) for p in parts]
            if op == "||":
                return True if any(v is True for v in vals) else (False if all(v is False for v in vals) else None)
            return False if any(v is False for v in vals) else (True if all(v is True for v in vals) else None)
    if cond.startswith("!") and not cond.startswith("!="):
        v = eval_cond(cond[1:], ctx)
        return None if v is None else (not v)
    m = re.fullmatch(r"([\w.]+)\s*(==|!=)\s*\"([^\"]*)\"", cond)
    if m and m.group(1) in ctx:
        eq = str(ctx[m.group(1)]) == m.group(3)
        return eq if m.group(2) == "==" else not eq
    if re.fullmatch(r"[\w.]+", cond) and cond in ctx and isinstance(ctx[cond], bool):
        return ctx[cond]
    return None


def apply_remaps_and_scope(nodes, launch):
    ctx = hw_context(nodes, launch)
    unused_remaps = []
    for n in nodes.values():
        used = set()
        for ep in n.endpoints:
            vals = [eval_cond(c, ctx) for c in ep.conds]
            if any(v is False for v in vals):
                ep.active = False
            ep.cond_unresolved = [c for c, v in zip(ep.conds, vals) if v is None]
            ep.cond_resolved_true = [c for c, v in zip(ep.conds, vals) if v is True]
            for src_name, dst in n.remaps:
                if src_name is None or dst is None:
                    continue
                if ep.topic.lstrip("/") == str(src_name).lstrip("/") and str(dst).lstrip("/") != ep.topic.lstrip("/"):
                    ep.remapped_from = ep.topic
                    ep.topic = str(dst)
                if ep.topic == str(dst) or ep.remapped_from == ep.topic:
                    pass
            for src_name, dst in n.remaps:
                if src_name is not None and (ep.topic.lstrip("/") == str(src_name).lstrip("/")
                                             or ep.remapped_from and ep.remapped_from.lstrip("/") == str(src_name).lstrip("/")):
                    used.add(src_name)
        if n.kind == "parsed" or n.files or n.remaps:
            for src_name, dst in n.remaps:
                if src_name is not None and src_name not in used and str(src_name) != str(dst):
                    unused_remaps.append((n.name, src_name, dst))
    return unused_remaps


def key_of(topic):
    return topic if topic.startswith("/") else topic


def collect_topics(nodes):
    topics = {}
    for n in nodes.values():
        for ep in n.endpoints:
            if ep.kind != "topic":
                continue
            t = topics.setdefault(key_of(ep.topic), {"pubs": [], "subs": []})
            t["pubs" if ep.direction == "pub" else "subs"].append(ep)
    return topics


def qos_compatible(pub, sub):
    """Return a warning string or None.  Only decided when both sides resolved."""
    if pub is None or sub is None:
        return None
    if pub.reliability and sub.reliability == "reliable" and pub.reliability == "best_effort":
        return "subscriber reliable, publisher best_effort: no delivery"
    if pub.durability and sub.durability == "transient_local" and pub.durability == "volatile":
        return "subscriber transient_local, publisher volatile: no delivery"
    return None


# =============================================================================================
# Rendering
# =============================================================================================
def mid(s):
    return re.sub(r"[^A-Za-z0-9_]", "_", s)


def mq(s):
    """Quote a Mermaid label: entity-encode characters Mermaid mis-parses, keep <br/> line breaks."""
    def enc(x):
        x = x.replace('"', "#quot;").replace("<", "#lt;").replace(">", "#gt;")
        return x.replace("|", "#124;")
    return "<br/>".join(enc(part) for part in s.split("<br/>"))


def short(t):
    return t.split("/")[-1] if t else t


def type_label(t):
    return t.replace("/", "/")


def render(nodes, panes, launch, unused_remaps):
    topics = collect_topics(nodes)
    lines = []
    A = lines.append

    A("<!-- GENERATED FILE. DO NOT EDIT BY HAND. -->")
    A("<!-- Regenerate: python3 scripts/gen_architecture_diagram.py   (check: add --check) -->")
    A("")
    A("# Goal selector architecture")
    A("")
    A("> **Generated file, do not edit by hand.** Generated by `python3 scripts/gen_architecture_diagram.py` "
      "from the C++ sources, `launch/onboard_mighty.launch.py`, `docker/mighty_hw.sh`, `CMakeLists.txt` and "
      "`docs/external_nodes.yaml` (the only hand-maintained input). `--check` fails when this file is stale.")
    A("")
    A("Scope: the hardware ground-robot stack started by `docker/mighty_hw.sh` "
      "(`use_hardware:=true`, `robot_type:=red_rover`). Code paths that the hardware parameters disable "
      "(for example the simulation-only map subscriptions) are listed in the table but not drawn.")
    A("")
    A("Topic names: relative names (no leading `/`) resolve under the robot namespace, shown as `<ns>`; "
      "names with a leading `/` are global. Dashed edges are conditional (see the table notes); "
      "dotted edges from a file node are parameter YAML files, numbered in load order (later wins).")
    A("")
    A("## Diagram")
    A("")
    A("```mermaid")
    A("flowchart LR")

    parsed = [n for n in nodes.values() if n.kind == "parsed"]
    launched_ext = [n for n in nodes.values() if n.kind == "external" and n.package]
    pure_ext = [n for n in nodes.values() if n.kind == "external" and not n.package]

    def node_id(n):
        return "N_" + mid(n.name)

    # ---- process subgraphs
    for n in parsed + launched_ext:
        title = "process: %s" % (n.pane or n.name)
        sub = "P_" + mid(n.name)
        A("  subgraph %s[\"%s\"]" % (sub, mq(title)))
        A("    direction TB")
        nlabel = "%s<br/>(%s / %s)" % (n.name, n.package or "", n.exe or "") if n.exe else n.label
        if n.kind == "parsed" and n.declared:
            nlabel += "<br/>declares %d parameters" % len(set(n.declared))
        if n.gated:
            nlabel += "<br/>(waits for TF map and odom)"
        A("    %s[\"%s\"]" % (node_id(n), mq(nlabel)))
        A("  end")
    # ---- external groups
    groups = {}
    for n in pure_ext:
        groups.setdefault(n.group, []).append(n)
    for gi, (g, ns_) in enumerate(sorted(groups.items())):
        A("  subgraph G%d[\"%s\"]" % (gi, mq("external: " + g)))
        A("    direction TB")
        for n in ns_:
            A("    %s[\"%s\"]" % (node_id(n), mq(n.label + (" (unverified)" if n.unverified else ""))))
        A("  end")
    # ---- parameter files
    file_ids = {}
    all_files = []
    for n in parsed + launched_ext:
        for f in n.files:
            if f.get("active") is False:
                continue
            label = ("%s:config/%s" % (f["pkg"], f["name"])) if f["pkg"] != "mighty" else "config/" + f["name"]
            all_files.append(label)
    if all_files:
        A("  subgraph PF[\"parameter files (launch/onboard_mighty.launch.py)\"]")
        A("    direction TB")
        for label in sorted(set(all_files)):
            fid = "F_" + mid(label)
            file_ids[label] = fid
            A("    %s[(\"%s\")]" % (fid, mq(label)))
        A("  end")
    # ---- sinks
    unconsumed_viz = {n.name: [] for n in parsed}
    unused_edges = []
    nopub_edges = []
    edges = []
    for tkey, tv in sorted(topics.items()):
        pubs = [e for e in tv["pubs"] if e.active]
        subs = [e for e in tv["subs"] if e.active]
        if not pubs and not subs:
            continue
        mtype = (pubs or subs)[0].mtype
        lbl = "%s<br/>%s" % (tkey if tkey.startswith("/") else tkey, mtype)
        if pubs and subs:
            seen = set()
            for p in pubs:
                for s in subs:
                    if (p.node, s.node) in seen or (p.node == s.node and nodes[p.node].kind == "external"):
                        continue
                    seen.add((p.node, s.node))
                    dashed = bool(p.conds or s.conds) and (bool(getattr(p, "cond_unresolved", [])) or
                                                           bool(getattr(s, "cond_unresolved", [])))
                    edges.append((node_id(nodes[p.node]), node_id(nodes[s.node]), lbl, dashed))
        elif pubs:
            for p in pubs:
                if nodes[p.node].kind == "parsed" and (p.mtype.startswith(VIZ_TYPE_PREFIXES) or p.mtype in VIZ_TYPES):
                    unconsumed_viz[p.node].append(tkey)
                else:
                    unused_edges.append((node_id(nodes[p.node]), lbl))
        else:
            for s in subs:
                nopub_edges.append((node_id(nodes[s.node]), lbl))
    viz_sinks = [n for n in pure_ext if n.viz_sink]
    if unused_edges or any(unconsumed_viz.values()):
        A("  UNUSED[\"no subscriber in this stack\"]")
    if nopub_edges:
        A("  NOPUB[\"no publisher in this stack\"]")
    for a, b, lbl, dashed in edges:
        A("  %s %s|\"%s\"| %s" % (a, "-.->" if dashed else "-->", mq(lbl), b))
    for a, lbl in unused_edges:
        A("  %s -.->|\"%s\"| UNUSED" % (a, mq(lbl)))
    for node_name, tl in sorted(unconsumed_viz.items()):
        if tl:
            target = node_id(viz_sinks[0]) if viz_sinks else "UNUSED"
            A("  %s -->|\"%s\"| %s" % (node_id(nodes[node_name]), mq("%d visualization topics<br/>(see table)" % len(tl)),
                                        target))
    for b, lbl in nopub_edges:
        A("  NOPUB -.->|\"%s\"| %s" % (mq(lbl), b))
    # parameter file edges
    for n in parsed + launched_ext:
        order = 0
        for f in n.files:
            if f.get("active") is False:
                continue
            order += 1
            label = ("%s:config/%s" % (f["pkg"], f["name"])) if f["pkg"] != "mighty" else "config/" + f["name"]
            A("  %s -.-|\"%s\"| %s" % (file_ids[label], mq("layer %d" % order), node_id(n)))
    A("```")
    A("")

    # ---- parameters section
    A("## Parameter files per process")
    A("")
    A("| node | executable | sources scanned | parameter files in load order (later wins) | declared parameters |")
    A("|---|---|---|---|---|")
    for n in parsed + launched_ext:
        fl = []
        i = 0
        for f in n.files:
            tag = ""
            if f.get("active") is False:
                tag = " (inactive on hardware)"
            elif f.get("active") is None:
                tag = " (conditional)"
            i += 1
            nm = f["name"] if f["pkg"] == "mighty" else "%s:%s" % (f["pkg"], f["name"])
            c = " [%s]" % f["cond"] if f.get("cond") else ""
            fl.append("%d. `%s`%s%s" % (i, nm, tag, c.replace("|", "\\|")))
        decl = len(set(n.declared)) if n.kind == "parsed" else "-"
        srcs = "<br>".join("`%s`" % s for s in n.sources) if n.sources else "(external code)"
        A("| `%s` | `%s/%s` | %s | %s | %s |" % (n.name, n.package or "", n.exe or "", srcs, "<br>".join(fl) or "none",
                                                    decl))
    A("")
    # coverage of declared parameters by the active layers
    for n in parsed:
        keys_by_file = {}
        for f in n.files:
            if f.get("active") is False:
                continue
            p = ROOT / "config" / f["name"]
            if p.exists():
                keys = set(re.findall(r"^\s+([A-Za-z_][\w.]*):", p.read_text(), re.M)) - {"ros__parameters"}
                keys_by_file[f["name"]] = keys
        dec = sorted(set(n.declared))
        if not dec or not keys_by_file:
            continue
        A("`%s`: %d declared parameters; provided by file (any layer may override): %s; "
          "declared but in no active file (code default only): %d." % (
              n.name, len(dec),
              ", ".join("`%s` %d" % (fn, len([d for d in dec if d in ks])) for fn, ks in keys_by_file.items()),
              len([d for d in dec if not any(d in ks for ks in keys_by_file.values())])))
        A("")

    # ---- topic table
    A("## Topics")
    A("")
    A("| topic | message type | publisher(s) | subscriber(s) | QoS | source | notes |")
    A("|---|---|---|---|---|---|---|")

    def src_of(ep):
        return "`%s:%d`" % (ep.file, ep.line) if ep.line else "`%s`" % ep.file

    warnings = []
    for tkey, tv in sorted(topics.items()):
        pubs, subs = tv["pubs"], tv["subs"]
        mtypes = sorted({e.mtype for e in pubs + subs})
        qos_parts = []
        for e in pubs:
            qos_parts.append("pub %s: %s" % (e.node, e.qos_text))
        for e in subs:
            qos_parts.append("sub %s: %s" % (e.node, e.qos_text))
        notes = []
        if len(mtypes) > 1:
            notes.append("TYPE MISMATCH: " + ", ".join(mtypes))
            warnings.append("topic `%s`: message types differ (%s)" % (tkey, ", ".join(mtypes)))
        for e in pubs + subs:
            c = list(getattr(e, "cond_unresolved", e.conds))
            tag = "%s %s" % ("pub" if e.direction == "pub" else "sub", e.node)
            if c:
                notes.append("%s only if `%s`" % (tag, " && ".join(c)))
            if not e.active:
                notes.append("%s inactive on hardware (not drawn): `%s`" % (tag, " && ".join(e.conds)))
            if getattr(e, "remapped_from", None):
                notes.append("%s remapped from `%s` by the launch file" % (tag, e.remapped_from))
            if getattr(e, "note", None):
                notes.append("%s: %s" % (e.node, e.note))
            if getattr(e, "unverified", False):
                notes.append("%s entry unverified" % e.node)
        active_pubs = [e for e in pubs if e.active]
        active_subs = [e for e in subs if e.active]
        if not subs:
            notes.append("**no subscriber in this stack**")
        if not pubs:
            notes.append("**no publisher in this stack**")
        # nodes marked unverified
        for e in pubs + subs:
            if nodes[e.node].unverified:
                notes.append("%s unverified" % e.node)
        for p in active_pubs:
            for s in active_subs:
                w = qos_compatible(p.qos, s.qos)
                if w:
                    msg = "QoS: `%s` %s -> `%s`: %s" % (p.node, tkey, s.node, w)
                    warnings.append("topic `%s`: %s -> %s: %s" % (tkey, p.node, s.node, w))
                    notes.append("QoS WARNING (%s -> %s): %s" % (p.node, s.node, w))
        srcs = sorted({src_of(e) for e in pubs + subs})
        A("| `%s` | %s | %s | %s | %s | %s | %s |" % (
            tkey, "<br>".join("`%s`" % t for t in mtypes),
            "<br>".join("`%s`%s" % (e.node, "" if e.active else " (sim only)") for e in pubs) or "-",
            "<br>".join("`%s`%s" % (e.node, "" if e.active else " (sim only)") for e in subs) or "-",
            "<br>".join(q.replace("|", "\\|") for q in qos_parts) or "-",
            "<br>".join(srcs),
            "<br>".join(sorted(set(x.replace("|", "\\|") for x in notes))) or ""))
    A("")

    # services
    svc = [(n, e) for n in nodes.values() for e in n.endpoints if e.kind == "service" or e.direction in ("client", "service")]
    A("## Services and clients")
    A("")
    if svc:
        A("| name | type | node | role | source |")
        A("|---|---|---|---|---|")
        for n, e in svc:
            A("| `%s` | `%s` | `%s` | %s | %s |" % (e.topic, e.mtype, n.name, e.direction, src_of(e)))
    else:
        A("None: no `create_client` / `create_service` call exists in the scanned sources.")
    A("")

    # warnings
    A("## Findings of the cross-check")
    A("")
    if warnings:
        for w in sorted(set(warnings)):
            A("- " + w)
    else:
        A("No QoS or type mismatches between a publisher and a subscriber that both resolved.")
    unused_topics = sorted(k for k, v in topics.items()
                           if any(e.active for e in v["pubs"]) and not any(e.active for e in v["subs"])
                           and not any(e.mtype.startswith(VIZ_TYPE_PREFIXES) or e.mtype in VIZ_TYPES
                                       for e in v["pubs"] if nodes[e.node].kind == "parsed"))
    nopub_topics = sorted(k for k, v in topics.items()
                          if any(e.active for e in v["subs"]) and not any(e.active for e in v["pubs"]))
    A("")
    A("Published but with no subscriber in this stack (collapsed visualization topics excluded): %s." %
      (", ".join("`%s`" % t for t in unused_topics) or "none"))
    A("")
    A("Subscribed (on hardware) but with no publisher in this stack: %s." %
      (", ".join("`%s`" % t for t in nopub_topics) or "none"))
    if unused_remaps:
        A("")
        A("Launch remappings that do not match any topic the node creates (dead remaps):")
        A("")
        for nname, a, b in unused_remaps:
            A("- `%s`: `%s` -> `%s`" % (nname, a, b))
    A("")
    A("## How this is generated, and its limits")
    A("")
    A("- Which processes exist comes from the pane commands in `docker/mighty_hw.sh` (`only_nodes:=...`); "
      "each name is looked up among the `Node(...)` actions of the launch file (parsed with `ast`).")
    A("- The sources of each in-package executable come from `CMakeLists.txt` (`add_executable`, "
      "`add_library`, `target_link_libraries`). `create_publisher`, `create_subscription`, `create_client`, "
      "`create_service` and `message_filters` `subscribe` calls are found by text scanning of the "
      "comment-stripped source, including multi-line calls and template arguments.")
    A("- Topic names: string literals, adjacent literals, and local `std::string` variables built by `+` are "
      "resolved; any other term is printed as `<expr>` (the namespace member `ns_` becomes `<ns>`).")
    A("- QoS: integers, `rclcpp::QoS(...)` chains, `SensorDataQoS`, `rmw_qos_profile_sensor_data` and local "
      "`rclcpp::QoS` variables with later mutator calls are evaluated; anything else is printed as "
      "`unresolved: <expr>`. Default QoS profile means reliable, volatile.")
    A("- Conditions: enclosing `if` / `else if` / `else` chains and early-`return` guards in the same function "
      "are recorded. Atoms over `use_hardware`, `vehicle_type` and scalar parameters from the active YAML files "
      "are evaluated; calls whose condition is false on hardware are not drawn. Conditions that depend on "
      "run-time state stay in the notes.")
    A("- Not seen by the parser: publishers/subscribers created through helper functions or wrappers, "
      "`image_transport`, dynamically constructed topic names beyond simple `+` concatenation, and ROS "
      "parameter overrides of topic names. TF use is added from `tf2_ros::TransformListener` / broadcaster "
      "declarations.")
    A("- Parameter files: Python data-flow through variables in `launch_setup`; launch conditions are evaluated "
      "with the launch arguments in the `launch_base` of `docker/mighty_hw.sh`.")
    A("- External nodes (and their QoS) are whatever `docs/external_nodes.yaml` says; entries marked "
      "`unverified` there are flagged in the table.")
    A("")
    return "\n".join(lines)


# =============================================================================================
# --runtime
# =============================================================================================
def runtime_check(nodes, namespace):
    exe = shutil.which("ros2")
    if exe is None:
        print("--runtime: `ros2` was not found on PATH (no ROS installation or environment not sourced); "
              "the runtime cross-check was skipped.", file=sys.stderr)
        return 2
    topics_by_node = {}
    problems = 0
    for n in nodes.values():
        if not n.ros_name:
            continue
        full = "/%s/%s" % (namespace, n.ros_name)
        try:
            out = subprocess.run([exe, "node", "info", full], capture_output=True, text=True, timeout=20)
        except (OSError, subprocess.TimeoutExpired) as e:
            print("--runtime: could not run ros2 for %s: %s" % (full, e), file=sys.stderr)
            return 2
        if out.returncode != 0:
            print("--runtime: node %s not found in the running graph (%s)" % (full, out.stderr.strip().splitlines()[-1:]
                                                                              or out.stdout.strip()[:80]))
            problems += 1
            continue
        got = {"Subscribers": set(), "Publishers": set()}
        section = None
        for line in out.stdout.splitlines():
            h = re.match(r"^\s{2}(Subscribers|Publishers|Service Servers|Service Clients|Action Servers|Action Clients):", line)
            if h:
                section = h.group(1)
                continue
            m = re.match(r"^\s{4}(\S+):\s*(\S+)", line)
            if m and section in got:
                got[section].add(m.group(1))
        ignore = {"/parameter_events", "/rosout"}
        for sect, direction in (("Subscribers", "sub"), ("Publishers", "pub")):
            expected = {}
            for ep in n.endpoints:
                if ep.kind != "topic" or ep.direction != direction or not ep.active:
                    continue
                name = ep.topic if ep.topic.startswith("/") else "/%s/%s" % (namespace, ep.topic)
                if "<" in name:
                    continue
                expected[name] = ep
            live = got[sect] - ignore
            for name in sorted(set(expected) - live):
                cond = " (conditional: %s)" % " && ".join(expected[name].conds) if expected[name].conds else ""
                print("%s %s: in code/yaml but NOT live: %s%s" % (full, sect.lower(), name, cond))
                problems += 1
            for name in sorted(live - set(expected)):
                if name.startswith(("/" + namespace + "/<",)):
                    continue
                print("%s %s: live but NOT in code/yaml: %s" % (full, sect.lower(), name))
                problems += 1
    print("--runtime: %d difference(s)" % problems)
    return 1 if problems else 0


# =============================================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--check", action="store_true", help="exit 1 if the output file differs from a fresh render")
    ap.add_argument("--runtime", action="store_true", help="also compare with `ros2 node info` of a running stack")
    ap.add_argument("--output", default=str(DEFAULT_OUTPUT), help="output path (default: %(default)s)")
    ap.add_argument("--namespace", default=os.environ.get("ROBOT_NAME", "RR04"),
                    help="robot namespace for --runtime (default: $ROBOT_NAME or RR04)")
    args = ap.parse_args(argv)

    nodes, panes, launch = build_model()
    unused = apply_remaps_and_scope(nodes, launch)
    text = render(nodes, panes, launch, unused)
    if not text.endswith("\n"):
        text += "\n"
    out = Path(args.output)
    rc = 0
    if args.check:
        old = out.read_text() if out.exists() else ""
        if old != text:
            diff = list(difflib.unified_diff(old.splitlines(), text.splitlines(), "committed", "regenerated",
                                             lineterm="", n=1))
            print("%s is out of date; run: python3 scripts/gen_architecture_diagram.py" % rel(out), file=sys.stderr)
            for l in diff[:40]:
                print(l, file=sys.stderr)
            if len(diff) > 40:
                print("... (%d more diff lines)" % (len(diff) - 40), file=sys.stderr)
            rc = 1
        else:
            print("%s is up to date" % rel(out))
    else:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text)
        print("wrote %s" % rel(out))
    if args.runtime:
        rrc = runtime_check(nodes, args.namespace)
        rc = rc or rrc
    return rc


if __name__ == "__main__":
    sys.exit(main())
