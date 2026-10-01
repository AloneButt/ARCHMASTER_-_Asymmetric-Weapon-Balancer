# Asymmetric Weapon Balancer - Fusion 360 add-in
#
# Adds a "Balance Weapon" button to its own ARCHMASTER tab in the Design
# workspace. In the dialog you select the bodies that spin, pick the spin
# axis, and tick the sketch dimensions (d1, d2, ...) the solver may change.
# Everything else stays as it is. The command adjusts the selected dimensions
# until the combined centre of mass lies on the spin axis in both in-plane
# directions (X and Y for a weapon sketched on XY).
#
# How it works
#   1. Analysis. As soon as the bodies and axis are chosen, every dimension is
#      nudged by a small step (inside Fusion's command preview, and restored)
#      to measure how it moves the centre of mass. The list then shows each
#      dimension's effect, and the summary lists every dimension that moves
#      the centre of mass and what it does, so you choose what is acceptable
#      to change in your design. Ticking dimensions gives an
#      instant controllability verdict for that set (singular values of the
#      2 x n sensitivity matrix, plus a linear estimate of the change needed
#      against the allowed +/- %). If balance looks impossible when you click
#      Balance, you are asked before anything is solved.
#   2. Solve.
#      Two or more dimensions: minimum-norm Gauss-Newton (smallest overall
#      proportional change), reusing the sensitivities from the check, with
#      Broyden updates, bounds of +/- the chosen %, and step halving when a
#      rebuild fails.
#      One dimension: it can only move the centre of mass along one line, so
#      the solver scans the range and root-finds the closest approach to the
#      axis (Illinois method); any offset across that line is reported.
#
# The whole solve is a single undo step: Ctrl+Z restores the previous design.
#
# MIT licence - Soso Chkhortolia @ ARCHMASTER

import adsk.core
import adsk.fusion
import html
import json
import math
import os
import re
import traceback

APP = adsk.core.Application.get()
UI = APP.userInterface

# ============================ SETTINGS ============================

CMD_ID = 'archmaster_asymmetric_weapon_balancer'
CMD_NAME = 'Balance Weapon'
CMD_TOOLTIP = ('Adjust the free sketch dimensions until the centre of mass of the '
               'weapon lies on its spin axis.')
WORKSPACE_ID = 'FusionSolidEnvironment'
TAB_ID = 'ArchmasterTab'              # own tab in the Design workspace toolbar
TAB_NAME = 'ARCHMASTER'
PANEL_ID = 'ArchmasterWeaponPanel'
PANEL_NAME = 'Weapons'
FALLBACK_PANEL_IDS = ('InspectPanel', 'SolidScriptsAddinsPanel')   # if the tab can't be made

DEFAULT_LIMIT_PCT = 45        # each free dimension may change by +/- this %
COM_TOL_MM = 1.0e-4           # balanced when the offset is below this

# one free dimension (scan + root find)
SCAN_SAMPLES = 9
R_TOL_FRACTION = 1.0e-7
MAX_EVALS = 40

# several free dimensions (minimum-change Gauss-Newton)
FD_STEP = 2.0e-3              # finite-difference step, fraction of each dimension
MAX_EVALS_MULTI = 160         # hard limit on rebuilds

# ==================================================================

ATTR_GROUP = 'ArchmasterWeaponBalancer'
ATTR_NAME = 'lastRun'
RESOURCES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources')
CM_TO_MM = 10.0
HIGH_ACC = adsk.fusion.CalculationAccuracy.HighCalculationAccuracy
AXIS_FILTERS = ('CylindricalFaces', 'CircularEdges', 'SketchCircles', 'ConstructionLines')

_handlers = []
_dim_names = []               # names in the "Dimensions to change" list, in order


# ------------------------------------------------------------------
# Geometry
# ------------------------------------------------------------------

def _v(p):
    return (p.x, p.y, p.z)


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _unit(a):
    length = math.sqrt(_dot(a, a))
    if length == 0.0:
        raise RuntimeError('The spin axis has no direction.')
    return (a[0] / length, a[1] / length, a[2] / length)


def _axis_name(v, fallback):
    """'X', 'Y' or 'Z' if v lies along a model axis (made positive), else fallback."""
    i = max(range(3), key=lambda k: abs(v[k]))
    sign = 1.0 if v[i] >= 0.0 else -1.0
    v = (v[0] * sign, v[1] * sign, v[2] * sign)
    return v, ('XYZ'[i] if abs(v[i]) > 0.9999 else fallback)


class SpinAxis:
    """Spin axis in model space (cm) with a basis for the plane normal to it.

    The two in-plane directions follow the model axes where possible, so the
    offsets read like Fusion's Properties dialog: a weapon sketched on XY
    spins about Z and is balanced in X and Y; one sketched on XZ spins about
    Y and is balanced in X and Z.
    """

    def __init__(self, point, direction, label):
        self.p = point
        self.n = _unit(direction)
        k = min(range(3), key=lambda i: abs(self.n[i]))       # model axis most across n
        ref = [0.0, 0.0, 0.0]
        ref[k] = 1.0
        e1 = _unit(_sub(ref, tuple(self.n[i] * self.n[k] for i in range(3))))
        self.e1, name1 = _axis_name(e1, 'U')
        self.e2, name2 = _axis_name(_cross(self.n, self.e1), 'V')
        if name1 in 'XYZ' and name2 in 'XYZ' and name1 > name2:   # list as X-Y, X-Z, Y-Z
            self.e1, self.e2, name1, name2 = self.e2, self.e1, name2, name1
        self.names = (name1, name2)
        self.n_name = _axis_name(self.n, 'axis')[1]
        self.label = label

    def offset_mm(self, com):
        """In-plane offset of a point from the axis, (a, b) in mm."""
        d = _sub(com, self.p)
        return (_dot(d, self.e1) * CM_TO_MM, _dot(d, self.e2) * CM_TO_MM)

    def along_mm(self, com):
        """Position along the spin axis, mm (does not affect balance)."""
        return _dot(_sub(com, self.p), self.n) * CM_TO_MM

    def describe(self, ab, digits=4):
        return '{} {:+.{d}f}   {} {:+.{d}f} mm'.format(
            self.names[0], ab[0], self.names[1], ab[1], d=digits)


def spin_axis_from(entity):
    circle = adsk.fusion.SketchCircle.cast(entity)
    if circle:
        g = circle.worldGeometry
        return SpinAxis(_v(g.center), _v(g.normal),
                        'sketch circle \u00d8{:.3f} mm'.format(g.radius * 2 * CM_TO_MM))

    face = adsk.fusion.BRepFace.cast(entity)
    if face:
        cyl = adsk.core.Cylinder.cast(face.geometry)
        if cyl:
            return SpinAxis(_v(cyl.origin), _v(cyl.axis),
                            'cylindrical face \u00d8{:.3f} mm'.format(cyl.radius * 2 * CM_TO_MM))

    edge = adsk.fusion.BRepEdge.cast(entity)
    if edge:
        g = adsk.core.Circle3D.cast(edge.geometry) or adsk.core.Arc3D.cast(edge.geometry)
        if g:
            return SpinAxis(_v(g.center), _v(g.normal),
                            'circular edge \u00d8{:.3f} mm'.format(g.radius * 2 * CM_TO_MM))

    axis = adsk.fusion.ConstructionAxis.cast(entity)
    if axis:
        g = axis.geometry
        return SpinAxis(_v(g.origin), _v(g.direction),
                        'construction axis "{}"'.format(axis.name))

    raise RuntimeError('The spin-axis selection is not a circle, cylinder or axis.')


def mass_properties(bodies):
    """Combined mass (g) and centre of mass (model space, cm)."""
    total, acc = 0.0, [0.0, 0.0, 0.0]
    for body in bodies:
        try:
            pp = body.getPhysicalProperties(HIGH_ACC)
        except AttributeError:
            pp = body.physicalProperties
        m, c = pp.mass, pp.centerOfMass
        total += m
        acc[0] += m * c.x
        acc[1] += m * c.y
        acc[2] += m * c.z
    if total <= 0.0:
        raise RuntimeError('The selected bodies have no mass.')
    return total * 1000.0, (acc[0] / total, acc[1] / total, acc[2] / total)


def _entities(design, token):
    try:
        return list(design.findEntityByToken(token))
    except Exception:
        return []


def bodies_from_tokens(design, tokens):
    """Re-find the selected bodies after a rebuild (the old objects go stale)."""
    bodies = []
    for token in tokens:
        found = [b for b in (adsk.fusion.BRepBody.cast(e) for e in _entities(design, token)) if b]
        if not found:
            raise RuntimeError('A selected body no longer exists after the rebuild. '
                               'The parameter value probably breaks the model.')
        for b in found:
            if not any(b.entityToken == other.entityToken for other in bodies):
                bodies.append(b)
    return bodies


# ------------------------------------------------------------------
# Dimensions, rebuilds and stored settings
# ------------------------------------------------------------------

SKETCH_PARAM_RE = re.compile(r'^d(\d+)$')


def is_length(units, unit):
    return bool(unit) and units.isValidExpression('1 mm', unit)


def depends_on_others(design, param):
    """True when the expression refers to another parameter (e.g. 'd3 / 2')."""
    for name in re.findall(r'[A-Za-z_][A-Za-z0-9_]*', param.expression):
        if name != param.name and design.allParameters.itemByName(name):
            return True
    return False


def sketch_dimensions(design):
    """Sketch dimension parameters d1, d2, ... sorted by number.

    Returns [(param, sketch_name, driven)] where driven means the dimension is
    defined by an expression that refers to other parameters.
    """
    found = []
    for comp in design.allComponents:
        for p in comp.modelParameters:
            m = SKETCH_PARAM_RE.match(p.name)
            if not m:
                continue
            sketch_name = ''
            try:
                dim = adsk.fusion.SketchDimension.cast(p.createdBy)
                if dim:
                    sketch_name = dim.parentSketch.name
            except Exception:
                pass
            found.append((int(m.group(1)), p, sketch_name, depends_on_others(design, p)))
    found.sort(key=lambda t: t[0])
    return [(p, name, driven) for _, p, name, driven in found]


def auto_range(design, param, pct):
    """Allowed range for one dimension: +/- pct % of its value, kept positive for lengths."""
    units = design.unitsManager
    span = abs(param.value) * pct / 100.0 or units.evaluateExpression('10', param.unit)
    lo, hi = param.value - span, param.value + span
    if param.value > 0.0 and lo <= 0.0 and is_length(units, param.unit):
        lo = min(hi * 0.05, units.evaluateExpression('0.1 mm', param.unit))
    return lo, hi


def error_count(design):
    """Number of timeline items in an error state (failed sketches or features)."""
    n = 0
    try:
        timeline = design.timeline
        for i in range(timeline.count):
            try:
                if timeline.item(i).healthState == \
                        adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState:
                    n += 1
            except Exception:
                pass
    except Exception:
        pass
    return n


class RebuildFailed(Exception):
    pass


def set_values(design, changes):
    """changes: [(param, number or expression string)]"""
    for param, value in changes:
        if isinstance(value, str):
            param.expression = value
        else:
            param.value = value
    if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        design.computeAll()
    adsk.doEvents()


def restore_expressions(design, originals):
    """originals: [(param, expression)]"""
    for param, expression in originals:
        try:
            param.expression = expression
        except Exception:
            pass
    try:
        design.computeAll()
    except Exception:
        pass


def load_prefs(design):
    try:
        attr = design.attributes.itemByName(ATTR_GROUP, ATTR_NAME)
        return json.loads(attr.value) if attr else {}
    except Exception:
        return {}


def save_prefs(design, prefs):
    try:
        design.attributes.add(ATTR_GROUP, ATTR_NAME, json.dumps(prefs))
    except Exception:
        pass


# ------------------------------------------------------------------
# Solvers (pure Python, no Fusion calls)
# ------------------------------------------------------------------

class SolveError(Exception):
    def __init__(self, message, report=''):
        super().__init__(message)
        self.report = report


class Cancelled(Exception):
    pass


def solve(evaluate, lo, hi, x_ref, fmt=str, tick=None):
    """Find the parameter value in [lo, hi] that brings the COM closest to the axis.

    evaluate(x) -> (a, b): in-plane offset of the COM from the axis, mm.
    tick(n) is called before each new rebuild and may raise Cancelled.
    Returns a dict with the root and projection helpers.
    """
    cache = {}

    def ev(x):
        key = round(x, 12)
        if key not in cache:
            if tick:
                tick(len(cache))
            cache[key] = evaluate(x)
        return cache[key]

    # 1. scan
    samples = []
    for i in range(SCAN_SAMPLES):
        x = lo + (hi - lo) * i / (SCAN_SAMPLES - 1)
        try:
            samples.append((x, ev(x)))
        except Cancelled:
            raise
        except Exception:
            samples.append((x, None))
    valid = [(x, ab) for x, ab in samples if ab is not None]

    def scan_report(project=None):
        head = 'COM offset along its travel' if project else 'COM distance from axis'
        lines = ['Scan ({}, mm):'.format(head)]
        for x, ab in samples:
            if ab is None:
                val = 'rebuild failed'
            elif project:
                val = '{:+.6f}'.format(project(ab))
            else:
                val = '{:.6f}'.format(math.hypot(*ab))
            lines.append('  {:>14}  ->  {}'.format(fmt(x), val))
        return '\n'.join(lines)

    if len(valid) < 2:
        raise SolveError('The model failed to rebuild across the scanned range.', scan_report())

    # 2. direction in which the parameter moves the COM
    n = len(valid)
    ma = sum(ab[0] for _, ab in valid) / n
    mb = sum(ab[1] for _, ab in valid) / n
    sxx = sum((ab[0] - ma) ** 2 for _, ab in valid)
    syy = sum((ab[1] - mb) ** 2 for _, ab in valid)
    sxy = sum((ab[0] - ma) * (ab[1] - mb) for _, ab in valid)
    travel = max(math.hypot(p[0] - q[0], p[1] - q[1]) for _, p in valid for _, q in valid)
    if travel < 10 * COM_TOL_MM:
        raise SolveError('Changing this parameter barely moves the centre of mass '
                         '({:.2e} mm over the whole range). Pick the parameter that '
                         'shapes the weapon\'s mass distribution.'.format(travel),
                         scan_report())

    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    u = (math.cos(theta), math.sin(theta))
    first, last = valid[0][1], valid[-1][1]
    if (last[0] - first[0]) * u[0] + (last[1] - first[1]) * u[1] < 0.0:
        u = (-u[0], -u[1])

    def along(ab):
        return ab[0] * u[0] + ab[1] * u[1]

    def across(ab):
        return -ab[0] * u[1] + ab[1] * u[0]

    report = scan_report(along)

        # 3. bracket: of all crossings in the range, take the one nearest the
    #    current value (smallest change), not simply the first one scanned
    pts = [(x, along(ab)) for x, ab in valid]
    candidates = []                       # (estimated root, a, fa, b, fb)
    for x, f in pts:
        if f == 0.0:
            candidates.append((x, x, 0.0, x, 0.0))
    for (x1, f1), (x2, f2) in zip(pts, pts[1:]):
        if f1 * f2 < 0.0:
            est = x1 - f1 * (x2 - x1) / (f2 - f1)      # linear estimate of the root
            candidates.append((est, x1, f1, x2, f2))
    a = fa = b = fb = None
    if candidates:
        _, a, fa, b, fb = min(candidates, key=lambda c: abs(c[0] - x_ref))
    if a is None:
        best = min(pts, key=lambda p: abs(p[1]))
        raise SolveError('The centre of mass never crosses the axis in this range '
                         '(closest: {:.4f} mm at {}).\nWiden the search range or use a '
                         'custom min / max.'.format(abs(best[1]), fmt(best[0])), report)

    # 4. Illinois root finder
    r_tol = max(abs(x_ref), 1e-6) * R_TOL_FRACTION
    side = 0
    root = a if fa == 0.0 else None
    while root is None and len(cache) < MAX_EVALS:
        c = 0.5 * (a + b) if fb == fa else b - fb * (b - a) / (fb - fa)
        span = abs(b - a)
        if not (min(a, b) + 0.01 * span <= c <= max(a, b) - 0.01 * span):
            c = 0.5 * (a + b)
        fc = along(ev(c))
        if abs(fc) <= COM_TOL_MM or span <= r_tol:
            root = c
            break
        if fa * fc < 0.0:
            b, fb = c, fc
            if side == -1:
                fa *= 0.5
            side = -1
        else:
            a, fa = c, fc
            if side == 1:
                fb *= 0.5
            side = 1
    if root is None:
        root = 0.5 * (a + b)

    return {'root': root, 'along': along, 'across': across,
            'evals': len(cache), 'report': report}


class _OutOfRebuilds(Exception):
    pass


def _solve_linear(a, b):
    """Solve the small square system a x = b (Gaussian elimination, partial pivoting)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-300:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def _min_norm_step(jac, res, cols, mu):
    """Smallest step dz over the columns in cols with jac dz = -res (damped)."""
    m = len(res)
    a = [[sum(jac[p][i] * jac[q][i] for i in cols) for q in range(m)] for p in range(m)]
    lam = mu * max(sum(a[i][i] for i in range(m)), 1e-30)
    for i in range(m):
        a[i][i] += lam
    y = _solve_linear(a, list(res))
    step = {}
    if y is None:
        return step
    for i in cols:
        step[i] = -sum(jac[p][i] * y[p] for p in range(m))
    return step


def probe_sensitivity(evaluate, x0, scale, lo, hi, r0):
    """Finite-difference sensitivity of the in-plane offset to each dimension.

    Returns (jac, dead): jac[p][i] = d offset_p / d z_i with z_i = (x_i - x0_i) / scale_i,
    i.e. mm of centre-of-mass movement per 100 % change. dead lists the dimensions
    whose small change in either direction fails to rebuild.
    """
    n, m = len(x0), len(r0)
    jac = [[0.0] * n for _ in range(m)]
    dead = []
    for i in range(n):
        col = None
        for h in (FD_STEP, -FD_STEP):
            x = list(x0)
            x[i] = x0[i] + h * scale[i]
            if not lo[i] <= x[i] <= hi[i]:
                continue
            try:
                rr = evaluate(x)
            except Cancelled:
                raise
            except Exception:
                continue
            col = [(rr[p] - r0[p]) / h for p in range(m)]
            break
        if col is None:
            dead.append(i)
        else:
            for p in range(m):
                jac[p][i] = col[p]
    return jac, dead


def _eig2(a):
    """Eigenvalues (large, small) and the unit eigenvector of the large one, 2x2 symmetric."""
    tr, det = a[0][0] + a[1][1], a[0][0] * a[1][1] - a[0][1] * a[1][0]
    disc = math.sqrt(max(tr * tr / 4.0 - det, 0.0))
    l1, l2 = tr / 2.0 + disc, max(tr / 2.0 - disc, 0.0)
    if abs(a[0][1]) > 1e-300:
        v = (l1 - a[1][1], a[0][1])
    else:
        v = (1.0, 0.0) if a[0][0] >= a[1][1] else (0.0, 1.0)
    length = math.hypot(*v)
    return l1, l2, (v[0] / length, v[1] / length)


def controllability(jac, r0, pct, dead=()):
    """Can these dimensions move the centre of mass in both in-plane directions?

    jac: 2 x n sensitivities per 100 % change (from probe_sensitivity).
    r0: current in-plane offset (mm). pct: allowed change per dimension (%).
    Returns a dict:
      verdict   'both' | 'weak' | 'one line' | 'none'
      per_pct   [(dx, dy)] mm of centre-of-mass movement per +1 % of each dimension
      ratio     weakest / strongest controllable direction (0 = one line only, 1 = even)
      line      unit direction the dimensions move the centre of mass along (one-line case)
      across    offset across that line that no change can remove, mm (one-line case)
      estimate  linear estimate of the change needed per dimension, % (smallest total change)
      reachable whether that estimate stays within +/- pct
    """
    n = len(jac[0])
    live = [i for i in range(n) if i not in dead]
    per_pct = [(jac[0][i] / 100.0, jac[1][i] / 100.0) for i in range(n)]
    a = [[sum(per_pct[i][p] * per_pct[i][q] for i in live) for q in range(2)] for p in range(2)]
    l1, l2, u = _eig2(a)
    s1, s2 = math.sqrt(l1), math.sqrt(l2)
    ratio = s2 / s1 if s1 > 0.0 else 0.0
    out = {'per_pct': per_pct, 'ratio': ratio, 'line': u, 'across': 0.0,
           'estimate': [0.0] * n, 'reachable': False}

    if s1 * pct < 10 * COM_TOL_MM:
        out['verdict'] = 'none'
        return out

    if ratio >= 0.02:
        out['verdict'] = 'both' if ratio >= 0.1 else 'weak'
        det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
        y = ((a[1][1] * -r0[0] - a[0][1] * -r0[1]) / det,
             (-a[1][0] * -r0[0] + a[0][0] * -r0[1]) / det)
        for i in live:
            out['estimate'][i] = per_pct[i][0] * y[0] + per_pct[i][1] * y[1]
    else:
        out['verdict'] = 'one line'
        w = (-u[1], u[0])
        out['across'] = abs(r0[0] * w[0] + r0[1] * w[1])
        g = [per_pct[i][0] * u[0] + per_pct[i][1] * u[1] for i in range(n)]
        gg = sum(g[i] ** 2 for i in live)
        along = r0[0] * u[0] + r0[1] * u[1]
        for i in live:
            out['estimate'][i] = -along * g[i] / gg
    out['reachable'] = max(abs(v) for v in out['estimate']) <= pct
    return out


def rate_dimensions(per_pct, r0, pct, dead=()):
    """What each dimension can do on its own.

    Returns one dict per dimension:
      kind    'ok' | 'none' (no measurable effect within +/- pct) | 'dead' (breaks the model)
      effect  mm of centre-of-mass movement per 1 % change
      share   fraction of the current offset it can cancel on its own (0..1)
      needed  % change that cancels that share
    """
    r = math.hypot(*r0)
    out = []
    for i, (dx, dy) in enumerate(per_pct):
        e = math.hypot(dx, dy)
        if i in dead:
            out.append({'kind': 'dead', 'effect': 0.0, 'share': 0.0, 'needed': 0.0})
        elif e * pct < 10 * COM_TOL_MM:
            out.append({'kind': 'none', 'effect': e, 'share': 0.0, 'needed': 0.0})
        elif r <= COM_TOL_MM:
            out.append({'kind': 'ok', 'effect': e, 'share': 1.0, 'needed': 0.0})
        else:
            dot = r0[0] * dx + r0[1] * dy
            out.append({'kind': 'ok', 'effect': e, 'share': abs(dot) / (e * r),
                        'needed': -dot / (e * e)})
    return out


def solve_multi(evaluate, x0, scale, lo, hi, tick=None, initial=None):
    """Change x as little as possible (relative to scale) so evaluate(x) -> 0.

    evaluate(x) -> residual tuple (mm); raises on a failed rebuild.
    Each x[i] stays within [lo[i], hi[i]].
    initial = (r0, jac, dead) from probe_sensitivity skips the first measurements.
    """
    n = len(x0)
    count = [0]
    zlo = [(lo[i] - x0[i]) / scale[i] for i in range(n)]
    zhi = [(hi[i] - x0[i]) / scale[i] for i in range(n)]

    def ev(z):
        if count[0] >= MAX_EVALS_MULTI:
            raise _OutOfRebuilds
        if tick:
            tick(count[0])
        count[0] += 1
        return tuple(evaluate([x0[i] + z[i] * scale[i] for i in range(n)]))

    def norm(r):
        return math.sqrt(sum(v * v for v in r))

    z = [0.0] * n
    if initial:
        r, jac0, dead0 = initial
        r = tuple(r)
    else:
        try:
            r = ev(z)
        except (Cancelled, _OutOfRebuilds):
            raise
        except Exception as e:
            raise SolveError('The model does not rebuild at its current values: {}'.format(e))
        jac0, dead0 = None, []
    r_start = r
    m = len(r)
    dead, no_effect = list(dead0), []
    active = [i for i in range(n) if i not in dead]

    def jacobian(z, r):
        jac = [[0.0] * n for _ in range(m)]
        for i in list(active):
            col = None
            for h in (FD_STEP, -FD_STEP):
                if not zlo[i] <= z[i] + h <= zhi[i]:
                    continue
                zz = list(z)
                zz[i] += h
                try:
                    rr = ev(zz)
                except (Cancelled, _OutOfRebuilds):
                    raise
                except Exception:
                    continue
                col = [(rr[p] - r[p]) / h for p in range(m)]
                break
            if col is None:
                active.remove(i)
                dead.append(i)
            else:
                for p in range(m):
                    jac[p][i] = col[p]
        return jac

    stop_reason = ''
    try:
        jac = [row[:] for row in jac0] if jac0 else jacobian(z, r)
        # dimensions that cannot move the COM by a measurable amount within their range
        for i in list(active):
            reach = math.sqrt(sum(jac[p][i] ** 2 for p in range(m))) * (zhi[i] - zlo[i])
            if reach < 10 * COM_TOL_MM:
                active.remove(i)
                no_effect.append(i)
        if not active:
            raise SolveError('None of the selected dimensions moves the centre of mass. '
                             'Select the dimensions that shape the weapon.')

        mu, fresh = 1e-10, True
        while norm(r) > COM_TOL_MM:
            # bounded minimum-norm step: pin variables that would leave their range
            cols, dz, target = list(active), [0.0] * n, list(r)
            while cols:
                s = _min_norm_step(jac, target, cols, mu)
                if not s:
                    break
                over = [i for i in cols if not zlo[i] <= z[i] + s[i] <= zhi[i]]
                if not over:
                    for i in cols:
                        dz[i] = s[i]
                    break
                for i in over:
                    dz[i] = min(max(z[i] + s[i], zlo[i]), zhi[i]) - z[i]
                    cols.remove(i)
                    for p in range(m):
                        target[p] += jac[p][i] * dz[i]
            if max(abs(v) for v in dz) < 1e-12:
                if fresh:
                    stop_reason = 'no further improvement is possible within the limits'
                    break
                jac, fresh = jacobian(z, r), True
                continue

            accepted, t = False, 1.0
            for _ in range(6):
                zt = [min(max(z[i] + t * dz[i], zlo[i]), zhi[i]) for i in range(n)]
                try:
                    rt = ev(zt)
                    accepted = norm(rt) < norm(r)
                except (Cancelled, _OutOfRebuilds):
                    raise
                except Exception:
                    accepted = False
                if accepted:
                    break
                t *= 0.5

            if accepted:
                step = [zt[i] - z[i] for i in range(n)]
                ss = sum(v * v for v in step)
                if ss > 0.0:                      # Broyden rank-one update
                    pred = [r[p] + sum(jac[p][i] * step[i] for i in range(n)) for p in range(m)]
                    for p in range(m):
                        k = (rt[p] - pred[p]) / ss
                        for i in active:
                            jac[p][i] += k * step[i]
                z, r = zt, rt
                mu, fresh = max(mu * 0.25, 1e-12), False
            elif fresh:
                stop_reason = ('no step improved the balance: the rest is out of reach of the selected '
                               'dimensions, or the model fails to rebuild nearby')
                break
            else:
                jac, fresh = jacobian(z, r), True
                mu *= 10.0
    except _OutOfRebuilds:
        stop_reason = 'reached the limit of {} rebuilds'.format(MAX_EVALS_MULTI)

    at_limit = [i for i in active
                if abs(z[i] - zlo[i]) < 1e-9 or abs(z[i] - zhi[i]) < 1e-9]

    # do the free dimensions only move the COM along one line? (2D offsets)
    one_line = False
    if m == 2 and active and 'jac' in locals():
        a = [[sum(jac[p][i] * jac[q][i] for i in active) for q in range(2)] for p in range(2)]
        tr = a[0][0] + a[1][1]
        det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
        one_line = tr > 0.0 and det / (0.25 * tr * tr) < 1e-4

    return {'x': [x0[i] + z[i] * scale[i] for i in range(n)], 'residual': r,
            'start': r_start, 'evals': count[0], 'no_effect': no_effect, 'dead': dead,
            'at_limit': at_limit, 'stop_reason': stop_reason, 'one_line': one_line}


# ------------------------------------------------------------------
# Dialog helpers
# ------------------------------------------------------------------

def _find(inputs, input_id):
    """itemById that also looks inside groups."""
    for i in range(inputs.count):
        item = inputs.item(i)
        if item.id == input_id:
            return item
        group = adsk.core.GroupCommandInput.cast(item)
        if group:
            found = _find(group.children, input_id)
            if found:
                return found
    return None


def _design():
    return adsk.fusion.Design.cast(APP.activeProduct)


def _names(names, limit=10):
    if not names:
        return 'none'
    text = ', '.join(names[:limit])
    if len(names) > limit:
        text += ' \u2026 (+{} more)'.format(len(names) - limit)
    return text


def selected_entities(inputs, input_id):
    sel = _find(inputs, input_id)
    return [sel.selection(i).entity for i in range(sel.selectionCount)]


def selected_indices(inputs):
    dd = _find(inputs, 'free')
    if not dd:
        return []
    return [i for i in range(dd.listItems.count)
            if i < len(_dim_names) and dd.listItems.item(i).isSelected]


def selected_free(design, inputs):
    """The ticked dimensions in 'Dimensions to change', in list order."""
    out = []
    for i in selected_indices(inputs):
        p = design.allParameters.itemByName(_dim_names[i])
        if p:
            out.append(p)
    return out


def setup_ranges(design, params, pct):
    """Start values, limits and scale (for relative changes) of the dimensions."""
    units = design.unitsManager
    x0 = [p.value for p in params]
    ranges = [auto_range(design, p, pct) for p in params]
    scale = [abs(v) if abs(v) > 1e-12 else units.evaluateExpression('1', p.unit)
             for p, v in zip(params, x0)]
    return x0, [lo for lo, _ in ranges], [hi for _, hi in ranges], scale


def make_evaluator(design, params, body_tokens, axis):
    """evaluate(xs) sets the dimensions, rebuilds, and returns the in-plane offset (mm).

    A dimension set back to its starting value gets its original expression back,
    so links such as d5 = d3 / 2 keep working while other dimensions are probed.
    """
    baseline_errors = error_count(design)
    start = {p.name: (p.value, p.expression) for p in params}
    applied = {p.name: p.value for p in params}

    def evaluate(xs):
        changes = []
        for p, v in zip(params, xs):
            if applied.get(p.name) == v:
                continue
            v0, expr0 = start[p.name]
            changes.append((p, v, expr0 if v == v0 else v))
        if changes:
            try:
                set_values(design, [(p, target) for p, _, target in changes])
            except Exception:
                applied.clear()
                raise
            for p, v, _ in changes:
                applied[p.name] = v
        if error_count(design) > baseline_errors:
            raise RebuildFailed('A sketch or feature failed to rebuild.')
        _, com = mass_properties(bodies_from_tokens(design, body_tokens))
        return axis.offset_mm(com)

    return evaluate


def control_blocked(c):
    """True when balance in both directions is impossible with these dimensions."""
    return c['verdict'] == 'none' or (c['verdict'] == 'one line' and c['across'] > COM_TOL_MM)


def control_lines(c, names, axis_names, pct, show_effects=True):
    """Plain-text explanation of a controllability result."""
    nx, ny = axis_names
    dead = c.get('dead', ())
    live = [i for i in range(len(names)) if i not in dead]
    lines = []
    if show_effects:
        lines.append('Effect of +1 % on the centre of mass:')
        for i in live:
            dx, dy = c['per_pct'][i]
            lines.append('  {}: {} {:+.4f}, {} {:+.4f} mm'.format(names[i], nx, dx, ny, dy))
    if dead:
        lines.append('Breaks the model when changed: {}'.format(_names([names[i] for i in dead])))

    v = c['verdict']
    if v == 'none':
        lines.append('NOT CONTROLLABLE: none of these dimensions moves the centre of mass '
                     'measurably within \u00b1{} %.'.format(pct))
        return lines
    if v == 'one line':
        ux, uy = c['line']
        lines.append('ONE DIRECTION ONLY: these dimensions all move the centre of mass along '
                     'the same line ({} {:+.2f}, {} {:+.2f}).'.format(nx, ux, ny, uy))
        if c['across'] > COM_TOL_MM:
            lines.append('{:.4f} mm of offset lies across that line and no change to these '
                         'dimensions can remove it. Add a dimension that moves the centre '
                         'of mass sideways.'.format(c['across']))
            return lines
        lines.append('The offset across that line is already zero, so full balance is possible.')
    elif v == 'weak':
        lines.append('BOTH {} AND {}, BUT WEAKLY: the dimensions move the centre of mass in '
                     'nearly the same direction, so correcting the other direction needs large '
                     'changes.'.format(nx, ny))
    else:
        lines.append('CONTROLLABLE IN BOTH {} AND {}.'.format(nx, ny))

    est = ', '.join('{} {:+.1f} %'.format(names[i], c['estimate'][i]) for i in live
                    if abs(c['estimate'][i]) >= 0.05) or 'none'
    worst = max(abs(c['estimate'][i]) for i in live) if live else 0.0
    lines.append('Estimated change to balance: {}.'.format(est))
    if worst <= 0.7 * pct:
        lines.append('Within the \u00b1{} % limit.'.format(pct))
    elif worst <= pct:
        lines.append('Close to the \u00b1{} % limit; the real change may be larger, so the '
                     'solve may stop short.'.format(pct))
    else:
        lines.append('Beyond the \u00b1{} % limit: raise Max change or pick dimensions with '
                     'more effect.'.format(pct))
    return lines


# ------------------------------------------------------------------
# Dimension analysis (every dN, measured once per bodies + axis selection)
# ------------------------------------------------------------------

_analysis = None          # None = not run yet; dict with results or {'error': text}
_base_labels = []         # list labels without the analysis tag
_dim_sketch = []          # sketch name of each dimension, same order as _dim_names


def analyse_all(design, bodies, axis_entity, pct):
    """Probe every dN once. Runs inside the command preview, which Fusion rolls back."""
    params = [design.allParameters.itemByName(n) for n in _dim_names]
    if not params or any(p is None for p in params):
        return {'error': 'Could not read the sketch dimensions.'}
    axis = spin_axis_from(axis_entity)
    tokens = [b.entityToken for b in bodies]
    x0, lo, hi, scale = setup_ranges(design, params, pct)
    originals = [(p, p.expression) for p in params]
    progress = UI.createProgressDialog()
    progress.isCancelButtonShown = True
    progress.cancelButtonText = 'Cancel'
    progress.isBackgroundTranslucent = False
    progress.show(CMD_NAME, 'Measuring how each dimension moves the centre of mass: '
                  '%v of %m', 0, len(params))
    count = [0]
    try:
        evaluate = make_evaluator(design, params, tokens, axis)

        def counted(x):
            progress.progressValue = count[0]
            adsk.doEvents()
            if progress.wasCancelled:
                raise Cancelled()
            count[0] += 1
            return evaluate(x)

        r0 = evaluate(x0)
        jac, dead = probe_sensitivity(counted, x0, scale, lo, hi, r0)
        return {'names': [p.name for p in params], 'values': x0, 'r0': r0, 'jac': jac,
                'dead': set(dead), 'axis_names': axis.names, 'rebuilds': count[0],
                'per_pct': [(jac[0][i] / 100.0, jac[1][i] / 100.0) for i in range(len(params))]}
    except Cancelled:
        return {'error': 'Analysis cancelled. Click Re-analyse to run it again.'}
    except Exception as e:
        return {'error': 'Analysis failed: {}'.format(e)}
    finally:
        progress.hide()
        restore_expressions(design, originals)


def _tag(rt, pct):
    if rt['kind'] == 'dead':
        return 'breaks the model'
    if rt['kind'] == 'none':
        return 'no effect'
    text = '{:.3f} mm/%'.format(rt['effect'])
    if rt['share'] >= 0.995 and abs(rt['needed']) < 1e-9:
        return text
    text += ', alone fixes {:.0f} % at {:+.1f} %'.format(rt['share'] * 100.0, rt['needed'])
    if abs(rt['needed']) > pct:
        text += ' (over limit)'
    return text


def apply_analysis(design, inputs):
    """Write the analysis into the list labels and the summary; no rebuilds."""
    dd = _find(inputs, 'free')
    box = _find(inputs, 'analysis')
    pct = _find(inputs, 'pct').value
    an = _analysis

    if not an or 'error' in an:
        for i in range(min(dd.listItems.count, len(_base_labels))):
            dd.listItems.item(i).name = _base_labels[i]
        if an:
            box.formattedText = html.escape(an['error'], quote=False)
        elif selected_entities(inputs, 'bodies') and selected_entities(inputs, 'axis'):
            box.formattedText = 'Analysing the dimensions\u2026'
        else:
            box.formattedText = ('Select the weapon bodies and the spin axis. Every dimension '
                                 'is then measured so you can see which ones move the centre '
                                 'of mass, and how.')
        update_selection_check(inputs)
        return

    rates = rate_dimensions(an['per_pct'], an['r0'], pct, an['dead'])
    for i in range(min(dd.listItems.count, len(_base_labels))):
        dd.listItems.item(i).name = '{}   \u2014 {}'.format(_base_labels[i], _tag(rates[i], pct))

    nx, ny = an['axis_names']
    names = an['names']
    usable = sorted((i for i, rt in enumerate(rates) if rt['kind'] == 'ok'),
                    key=lambda i: -rates[i]['effect'])
    none = [names[i] for i, rt in enumerate(rates) if rt['kind'] == 'none']
    dead = [names[i] for i, rt in enumerate(rates) if rt['kind'] == 'dead']
    balanced = math.hypot(*an['r0']) <= COM_TOL_MM

    lines = ['<b>{} of {} dimensions move the centre of mass.</b> Offset now: '
             '{} {:+.4f}, {} {:+.4f} mm.'.format(len(usable), len(rates),
                                                  nx, an['r0'][0], ny, an['r0'][1])]
    if usable:
        lines.append('What +1 % of each does, strongest first. Check that the change '
                     'suits your design before ticking it:')
    for i in usable:
        dx, dy = an['per_pct'][i]
        rt = rates[i]
        where = ' ({})'.format(_dim_sketch[i]) if i < len(_dim_sketch) and _dim_sketch[i] else ''
        text = '{}{}: {} {:+.4f}, {} {:+.4f} mm'.format(names[i], where, nx, dx, ny, dy)
        if not balanced:
            if rt['share'] < 0.25:
                text += ', pushes mostly sideways to the offset (alone fixes {:.0f} %)'.format(
                    rt['share'] * 100.0)
            else:
                text += ', alone fixes {:.0f} % of the offset at {:+.1f} %'.format(
                    rt['share'] * 100.0, rt['needed'])
            if abs(rt['needed']) > pct and rt['share'] >= 0.25:
                text += ' (over the \u00b1{} % limit)'.format(pct)
        lines.append('&nbsp;&nbsp;\u2022 ' + html.escape(text, quote=False))
    if none:
        lines.append('No effect: {}.'.format(html.escape(_names(none, 30))))
    if dead:
        lines.append('Break the model when changed: {}.'.format(html.escape(_names(dead, 30))))
    if balanced:
        lines.append('Already balanced.')
    elif usable:
        lines.append('To balance both {} and {}, tick at least two that push in different '
                     'directions: one that fixes a large share plus one that pushes sideways. '
                     'The line under the list checks your choice.'.format(nx, ny))
    box.formattedText = '<br>'.join(lines)
    update_selection_check(inputs)


def update_selection_check(inputs):
    """Controllability of the ticked set, from the cached analysis (no rebuilds)."""
    box = _find(inputs, 'control')
    if not box:
        return
    an = _analysis
    idx = selected_indices(inputs)
    if not an or 'error' in an or not idx:
        box.formattedText = ''
        box.isVisible = False
        return
    pct = _find(inputs, 'pct').value
    jac = [[an['jac'][p][i] for i in idx] for p in range(2)]
    dead = [k for k, i in enumerate(idx) if i in an['dead']]
    c = controllability(jac, an['r0'], pct, dead)
    c['dead'] = dead
    lines = control_lines(c, [an['names'][i] for i in idx], an['axis_names'], pct,
                          show_effects=False)
    box.formattedText = '<b>Ticked:</b> ' + '<br>'.join(html.escape(l, quote=False) for l in lines)
    box.isVisible = True


def update_status(design, inputs):
    status = _find(inputs, 'status')
    if not status:
        return
    try:
        lines = []
        if not _dim_names:
            lines.append('<b>No sketch dimensions (d1, d2, ...) found.</b> '
                         'Dimension the weapon sketch first.')
        else:
            free = selected_free(design, inputs)
            if free:
                lines.append('<b>Changing ({}):</b> {}'.format(
                    len(free), _names([p.name for p in free])))
                driven = [p.name for p in free if depends_on_others(design, p)]
                if driven:
                    lines.append('<i>Defined by expressions, will be replaced by numbers: '
                                 '{}</i>'.format(html.escape(_names(driven))))
            else:
                lines.append('Tick the dimensions to change.')

        bodies = selected_entities(inputs, 'bodies')
        axes = selected_entities(inputs, 'axis')
        if bodies:
            mass_g, com = mass_properties(bodies)
            line = 'Mass: {:.2f} g ({} bod{})'.format(
                mass_g, len(bodies), 'y' if len(bodies) == 1 else 'ies')
            if axes:
                axis = spin_axis_from(axes[0])
                ab = axis.offset_mm(com)
                r = math.hypot(*ab)
                lines.append(line + ' \u00b7 Axis: {}'.format(html.escape(axis.label)))
                lines.append('COM offset: <b>{}</b> \u00b7 total {:.4f} mm \u00b7 '
                             'imbalance {:.1f} g\u00b7mm'.format(
                                 html.escape(axis.describe(ab)), r, mass_g * r))
            else:
                lines.append(line)
        status.formattedText = '<br>'.join(lines)
    except Exception as e:
        status.formattedText = 'Could not measure: {}'.format(html.escape(str(e)))


# ------------------------------------------------------------------
# Command handlers
# ------------------------------------------------------------------

class CommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        global _analysis
        try:
            cmd = args.command
            inputs = cmd.commandInputs
            design = _design()
            _analysis = None

            problem = None
            if not design:
                problem = 'Open a Fusion design first.'
            elif design.designType != adsk.fusion.DesignTypes.ParametricDesignType:
                problem = ('This command needs a parametric design (with a timeline). '
                           'Direct-modelling designs do not rebuild from parameters.')
            if problem:
                box = inputs.addTextBoxCommandInput('status', '', problem, 3, True)
                box.isFullWidth = True
                cmd.isOKButtonVisible = False
                return

            prefs = load_prefs(design)
            cmd.okButtonText = 'Balance'

            sel = inputs.addSelectionInput('bodies', 'Weapon bodies',
                                           'Select every body that spins with the weapon')
            sel.addSelectionFilter('SolidBodies')
            sel.setSelectionLimits(1, 0)
            sel.tooltip = ('Everything that spins: blade, bolted-on teeth, inserts, '
                           'fasteners. Each body uses its own material.')

            sel = inputs.addSelectionInput('axis', 'Spin axis',
                                           'Select the bore face or edge, a sketch circle, '
                                           'or a construction axis')
            for f in AXIS_FILTERS:
                sel.addSelectionFilter(f)
            sel.setSelectionLimits(1, 1)
            sel.tooltip = 'The shaft axis the weapon spins about.'

            box = inputs.addTextBoxCommandInput('analysis', '', '', 12, True)
            box.isFullWidth = True

            b = inputs.addBoolValueInput('reanalyse', 'Re-analyse', False, '', False)
            b.tooltip = 'Measure every dimension again.'

            dd = inputs.addDropDownCommandInput('free', 'Dimensions to change',
                                                adsk.core.DropDownStyles.CheckBoxDropDownStyle)
            dd.tooltip = ('Each entry shows how far the dimension moves the centre of mass '
                          'per 1 % change, and how much of the current offset it can fix on '
                          'its own. Tick the ones the solver may adjust. Names match '
                          'Modify > Change Parameters.')
            dims = sketch_dimensions(design)
            _dim_names[:] = [p.name for p, _, _ in dims]
            _base_labels[:] = []
            _dim_sketch[:] = [sketch_name for _, sketch_name, _ in dims]
            chosen = set(prefs.get('free', []))
            units = design.unitsManager
            for p, sketch_name, driven in dims:
                label = '{} = {}'.format(p.name, units.formatInternalValue(p.value, p.unit, True))
                if sketch_name:
                    label += ' \u00b7 {}'.format(sketch_name)
                if driven:
                    label += ' (= {})'.format(p.expression)
                _base_labels.append(label)
                dd.listItems.add(label, p.name in chosen, '')

            box = inputs.addTextBoxCommandInput('control', '', '', 4, True)
            box.isFullWidth = True
            box.isVisible = False

            group = inputs.addGroupCommandInput('options', 'Options')
            group.isExpanded = False
            g = group.children
            g.addIntegerSpinnerCommandInput('pct', 'Max change \u00b1 %', 1, 95, 5,
                                            int(prefs.get('pct', DEFAULT_LIMIT_PCT)))
            _find(inputs, 'pct').tooltip = 'How far each ticked dimension may move from its current value.'

            box = inputs.addTextBoxCommandInput('status', '', '', 4, True)
            box.isFullWidth = True

            for event, handler in ((cmd.execute, ExecuteHandler()),
                                   (cmd.executePreview, PreviewHandler()),
                                   (cmd.inputChanged, InputChangedHandler()),
                                   (cmd.validateInputs, ValidateInputsHandler()),
                                   (cmd.activate, ActivateHandler())):
                event.add(handler)
                _handlers.append(handler)
        except Exception:
            UI.messageBox('Failed:\n{}'.format(traceback.format_exc()))


class ActivateHandler(adsk.core.CommandEventHandler):
    """Pre-select the bodies and axis used last time in this design."""

    def notify(self, args):
        try:
            design = _design()
            inputs = args.command.commandInputs
            prefs = load_prefs(design)
            bodies_in = _find(inputs, 'bodies')
            for token in prefs.get('bodies', []):
                for ent in _entities(design, token):
                    try:
                        bodies_in.addSelection(ent)
                    except Exception:
                        pass
            token = prefs.get('axis')
            if token:
                ents = _entities(design, token)
                if ents:
                    try:
                        _find(inputs, 'axis').addSelection(ents[0])
                    except Exception:
                        pass
            apply_analysis(design, inputs)
            update_status(design, inputs)
        except Exception:
            pass


class InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        global _analysis
        try:
            changed = args.input
            inputs = changed.parentCommand.commandInputs
            design = _design()
            if changed.id in ('bodies', 'axis', 'reanalyse'):
                _analysis = None                  # the preview measures again
                apply_analysis(design, inputs)
            elif changed.id == 'pct':
                apply_analysis(design, inputs)
            elif changed.id == 'free':
                update_selection_check(inputs)
            if changed.id in ('bodies', 'axis', 'free'):
                update_status(design, inputs)
        except Exception:
            pass


class ValidateInputsHandler(adsk.core.ValidateInputsEventHandler):
    """Bodies + axis are enough for the analysis preview; Balance checks the ticks."""

    def notify(self, args):
        try:
            inputs = args.inputs
            args.areInputsValid = (_find(inputs, 'bodies').selectionCount > 0 and
                                   _find(inputs, 'axis').selectionCount == 1)
        except Exception:
            args.areInputsValid = False


class PreviewHandler(adsk.core.CommandEventHandler):
    """Runs the dimension analysis once per bodies + axis selection.

    Fusion rolls back everything done in a preview, and every dimension is also
    restored explicitly, so the design is never changed by the analysis.
    """

    def notify(self, args):
        global _analysis
        if _analysis is not None:
            return
        try:
            inputs = args.command.commandInputs
            design = _design()
            bodies = selected_entities(inputs, 'bodies')
            axes = selected_entities(inputs, 'axis')
            if not (bodies and axes and _dim_names):
                return
            _analysis = analyse_all(design, bodies, axes[0], _find(inputs, 'pct').value)
            apply_analysis(design, inputs)
        except Exception as e:
            _analysis = {'error': 'Analysis failed: {}'.format(e)}


def _pct_change(v0, v):
    return '' if v0 == 0 else '  ({:+.2f} %)'.format((v - v0) / abs(v0) * 100.0)


def _cached_sensitivity(free, x0, r_start):
    """Reuse the dialog's analysis for the ticked dimensions if the design is unchanged."""
    an = _analysis
    if not an or 'error' in an:
        return None
    try:
        idx = [an['names'].index(p.name) for p in free]
    except ValueError:
        return None
    if any(an['values'][i] != v for i, v in zip(idx, x0)):
        return None
    if math.hypot(an['r0'][0] - r_start[0], an['r0'][1] - r_start[1]) > 1e-6:
        return None
    jac = [[an['jac'][p][i] for i in idx] for p in range(2)]
    dead = [k for k, i in enumerate(idx) if i in an['dead']]
    return jac, dead


class ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        design = _design()
        inputs = args.command.commandInputs
        originals = []
        progress = None

        def fail(message):
            restore_expressions(design, originals)
            note = '\n\nAll dimensions were restored.' if originals else ''
            UI.messageBox(message + note, CMD_NAME)

        def hide():
            if progress:
                progress.hide()

        try:
            units = design.unitsManager
            free = selected_free(design, inputs)
            if not free:
                UI.messageBox('Tick at least one dimension to change, then run '
                              'Balance Weapon again.', CMD_NAME)
                return
            names = [p.name for p in free]

            def fmt(p, v):
                return units.formatInternalValue(v, p.unit, True)

            body_entities = selected_entities(inputs, 'bodies')
            body_tokens = [b.entityToken for b in body_entities]
            axis_entity = selected_entities(inputs, 'axis')[0]
            axis = spin_axis_from(axis_entity)
            axis_token = axis_entity.entityToken
            pct = _find(inputs, 'pct').value

            originals = [(p, p.expression) for p in free]
            x0, lo, hi, scale = setup_ranges(design, free, pct)
            mass0, com0 = mass_properties(body_entities)
            ab0 = axis.offset_mm(com0)
            r0 = math.hypot(*ab0)
            evaluate = make_evaluator(design, free, body_tokens, axis)

            single = len(free) == 1
            limit = len(free) + (SCAN_SAMPLES + MAX_EVALS if single else MAX_EVALS_MULTI)
            progress = UI.createProgressDialog()
            progress.isCancelButtonShown = True
            progress.cancelButtonText = 'Cancel'
            progress.isBackgroundTranslucent = False
            progress.show(CMD_NAME, 'Rebuild %v of up to %m', 0, limit)
            done = [0]

            def tick(n):
                progress.progressValue = min(done[0] + n, limit)
                adsk.doEvents()
                if progress.wasCancelled:
                    raise Cancelled()

            # 1. controllability of the ticked set (from the dialog's analysis if still valid)
            probe_count = [0]

            def probe_eval(x):
                tick(probe_count[0])
                probe_count[0] += 1
                return evaluate(x)

            r_start = evaluate(x0)
            cached = _cached_sensitivity(free, x0, r_start)
            if cached:
                jac, dead = cached
            else:
                jac, dead = probe_sensitivity(probe_eval, x0, scale, lo, hi, r_start)
            ctrl = controllability(jac, r_start, pct, dead)
            ctrl['dead'] = dead
            done[0] = probe_count[0]
            report_lines = control_lines(ctrl, names, axis.names, pct)

            if ctrl['verdict'] == 'none':
                hide()
                progress = None
                raise SolveError('\n'.join(report_lines))
            if control_blocked(ctrl) or (ctrl['verdict'] in ('both', 'weak') and
                                         not ctrl['reachable']):
                hide()
                evaluate(x0)
                answer = UI.messageBox(
                    'Controllability check\n\n' + '\n'.join(report_lines) +
                    '\n\nCOM offset now: {}\n\nBalance as far as possible anyway?'.format(
                        axis.describe(ab0)),
                    CMD_NAME, adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                    adsk.core.MessageBoxIconTypes.WarningIconType)
                if answer != adsk.core.DialogResults.DialogYes:
                    progress = None
                    restore_expressions(design, originals)
                    return
                progress.show(CMD_NAME, 'Rebuild %v of up to %m', 0, limit)

            # 2. solve
            if single:
                p = free[0]
                res = solve(lambda v: evaluate([v]), lo[0], hi[0], x0[0],
                            lambda v: fmt(p, v), tick)
                x = [res['root']]
            else:
                res = solve_multi(evaluate, x0, scale, lo, hi, tick,
                                  initial=(r_start, jac, dead))
                x = res['x']
            hide()
            progress = None

            # 3. lock in the result and measure fresh
            evaluate(x)
            mass_g, com = mass_properties(bodies_from_tokens(design, body_tokens))
            ab = axis.offset_mm(com)
            r = math.hypot(*ab)
            balanced = max(abs(ab[0]), abs(ab[1])) <= COM_TOL_MM and r <= COM_TOL_MM
            changed = [(p, v0, v) for p, v0, v in zip(free, x0, x)
                       if abs(v - v0) > 1e-9 * max(1.0, abs(v0))]

            save_prefs(design, {'free': names, 'pct': pct,
                                'bodies': body_tokens, 'axis': axis_token})

            if balanced:
                head = 'Balanced in both {} and {}.'.format(*axis.names)
            elif r < r0 - COM_TOL_MM:
                head = 'Improved, but not fully balanced.'
            else:
                head = 'Could not improve the balance.'

            msg = head + '\n\n'
            msg += 'COM offset from the spin axis\n'
            msg += '  now: {}   (total {:.6f} mm)\n'.format(axis.describe(ab, 6), r)
            msg += '  was: {}   (total {:.4f} mm)\n'.format(axis.describe(ab0), r0)
            msg += ('  along the spin axis ({}): {:+.4f} mm, the mid-thickness position; '
                    'it does not cause imbalance\n').format(axis.n_name, axis.along_mm(com))
            msg += 'Imbalance: {:.3f} g\u00b7mm   (was {:.1f} g\u00b7mm)\n'.format(mass_g * r, mass0 * r0)
            msg += 'Mass: {:.2f} g   (was {:.2f} g)\n'.format(mass_g, mass0)
            msg += 'Axis: {}\n'.format(axis.label)

            msg += '\nChanged ({}):\n'.format(len(changed))
            for p, v0, v in changed:
                msg += '  {}: {} \u2192 {}{}\n'.format(p.name, fmt(p, v0), fmt(p, v), _pct_change(v0, v))
            if not changed:
                msg += '  nothing\n'
            if not single:
                if res['at_limit']:
                    msg += 'At the \u00b1{} % limit: {}\n'.format(pct, _names([names[i] for i in res['at_limit']]))
                if res['no_effect']:
                    msg += 'No effect on balance, unchanged: {}\n'.format(_names([names[i] for i in res['no_effect']]))
            msg += 'Rebuilds: {}\n'.format(probe_count[0] + res['evals'] + 1)

            if not balanced:
                msg += '\nControllability check (before solving):\n  '
                msg += '\n  '.join(report_lines) + '\n'
                if not single and res['stop_reason']:
                    msg += '\nStopped: {}.\n'.format(res['stop_reason'])
            msg += '\nCtrl+Z undoes the whole balance.'
            if single:
                msg += '\n\n' + res['report']
            UI.messageBox(msg, CMD_NAME)

        except Cancelled:
            hide()
            fail('Cancelled.')
        except SolveError as e:
            hide()
            fail('{}\n\n{}'.format(e, e.report).strip())
        except Exception:
            hide()
            fail('Failed:\n{}'.format(traceback.format_exc()))


# ------------------------------------------------------------------
# Add-in entry points
# ------------------------------------------------------------------

def _workspace():
    return UI.workspaces.itemById(WORKSPACE_ID)


def _get_or_create_panel():
    """Own ARCHMASTER tab + Weapons panel; falls back to a built-in panel."""
    ws = _workspace()
    if not ws:
        return None
    try:
        tab = ws.toolbarTabs.itemById(TAB_ID) or ws.toolbarTabs.add(TAB_ID, TAB_NAME)
        panel = tab.toolbarPanels.itemById(PANEL_ID) or tab.toolbarPanels.add(PANEL_ID, PANEL_NAME)
        if panel:
            return panel
    except Exception:
        pass
    for pid in FALLBACK_PANEL_IDS:
        panel = ws.toolbarPanels.itemById(pid)
        if panel:
            return panel
    return None


def _all_panels():
    ws = _workspace()
    if not ws:
        return []
    panels = []
    tab = ws.toolbarTabs.itemById(TAB_ID)
    if tab and tab.toolbarPanels.itemById(PANEL_ID):
        panels.append(tab.toolbarPanels.itemById(PANEL_ID))
    for pid in FALLBACK_PANEL_IDS:
        panel = ws.toolbarPanels.itemById(pid)
        if panel:
            panels.append(panel)
    return panels


def _remove_controls():
    for panel in _all_panels():
        ctl = panel.controls.itemById(CMD_ID)
        if ctl:
            ctl.deleteMe()


def run(context):
    try:
        _remove_controls()                      # leftovers from an earlier version
        defs = UI.commandDefinitions
        old = defs.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cmd_def = defs.addButtonDefinition(CMD_ID, CMD_NAME, CMD_TOOLTIP, RESOURCES)
        handler = CommandCreatedHandler()
        cmd_def.commandCreated.add(handler)
        _handlers.append(handler)

        panel = _get_or_create_panel()
        if not panel:
            UI.messageBox('{}: could not find a toolbar to add the button to.'.format(CMD_NAME))
            return
        ctl = panel.controls.addCommand(cmd_def)
        ctl.isPromoted = True                   # show on the toolbar, not only in the dropdown
        ctl.isPromotedByDefault = True
    except Exception:
        UI.messageBox('Failed to start {}:\n{}'.format(CMD_NAME, traceback.format_exc()))


def stop(context):
    try:
        _remove_controls()
        cmd_def = UI.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        # remove the panel and tab only when no other ARCHMASTER add-in still uses them
        ws = _workspace()
        tab = ws.toolbarTabs.itemById(TAB_ID) if ws else None
        if tab:
            panel = tab.toolbarPanels.itemById(PANEL_ID)
            if panel and panel.controls.count == 0:
                panel.deleteMe()
            if tab.toolbarPanels.count == 0:
                tab.deleteMe()
    except Exception:
        UI.messageBox('Failed to stop {}:\n{}'.format(CMD_NAME, traceback.format_exc()))
    _handlers.clear()
