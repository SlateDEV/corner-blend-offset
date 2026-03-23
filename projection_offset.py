"""
================================================================================
NURBS CURVE OFFSET - POINT PROJECTION METHOD
Grasshopper Python Component
================================================================================

COMPONENT SPECIFICATIONS:

Inputs:
-------
Name             | Type              | Description
-----------------|-------------------|--------------------------------------------
curve            | Curve             | Corner blend NURBS curve (8 CVs, 90-deg)
offset_distance  | Float             | Distance to offset (positive = +X/+Y)
projection_offset| Float             | Shifts projection point along symmetry axis
                 |                   | (positive = away from curve, negative = toward)

Outputs:
--------
Name             | Type              | Description
-----------------|-------------------|--------------------------------------------
offset_curve     | Curve             | Resulting offset NURBS curve
projection_point | Point3d           | Optimal projection center point location
d_opt            | Float             | Optimal projection center displacement
deviation        | Float             | Offset deviation at curve midpoint (units)
diagnostics      | String            | Diagnostic information

Associated Components:
---------------------
- Python Script component (GhPython)
- Curve parameter (input)
- Number Slider (offset_distance)
- Number Slider (projection_offset, default 0)
- Curve output
- Point output (projection_point)
- Panel (diagnostics)

================================================================================
"""

import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
import scriptcontext as sc
import math

# Save GH document context before rhinoscriptsyntax potentially changes it
_ghdoc = sc.doc


def coerce_curve(curve_input):
    """
    Convert a Grasshopper curve input to a Rhino NurbsCurve.
    Uses rhinoscriptsyntax to handle GUIDs, geometry wrappers, etc.

    Returns:
        NurbsCurve
    """
    # If it's already a NurbsCurve, use it directly
    if isinstance(curve_input, rg.NurbsCurve):
        return curve_input

    # If it's another Rhino curve type, convert
    if isinstance(curve_input, rg.Curve):
        return curve_input.ToNurbsCurve()

    # Otherwise treat it as a GUID and let rs handle the lookup
    geom = rs.coercecurve(curve_input)
    if geom is not None:
        return geom.ToNurbsCurve()

    raise ValueError(
        f"Cannot convert input to NurbsCurve. "
        f"Input type: {type(curve_input).__name__}. "
        "Make sure a valid curve is connected."
    )


# =============================================================================
# GEOMETRY UTILITIES
# =============================================================================

def get_cv_locations(curve):
    """
    Extract control point locations from a NURBS curve.

    Returns:
        list of Point3d
    """
    pts = []
    for i in range(curve.Points.Count):
        pts.append(curve.Points[i].Location)
    return pts


def line_ray_intersection_2d(line_pt, line_dir, ray_origin, ray_dir):
    """
    Find intersection of a line and a ray in 2D (XY plane).

    Line:  line_pt + t * line_dir   (infinite)
    Ray:   ray_origin + s * ray_dir (s >= 0)

    Returns:
        Point3d or None
    """
    det = ray_dir.X * line_dir.Y - ray_dir.Y * line_dir.X
    if abs(det) < 1e-10:
        return None  # parallel

    dx = line_pt.X - ray_origin.X
    dy = line_pt.Y - ray_origin.Y

    s = (dx * line_dir.Y - dy * line_dir.X) / det

    if s < -1e-6:
        return None  # intersection is behind the ray

    return rg.Point3d(
        ray_origin.X + s * ray_dir.X,
        ray_origin.Y + s * ray_dir.Y,
        ray_origin.Z
    )


def offset_line_perpendicular(line_pt, line_dir, offset_distance):
    """
    Offset a line in the +X/+Y direction (away from corner, outward).

    For edge 1 (roughly along X): shifts in +Y
    For edge 2 (roughly along Y): shifts in +X

    The perpendicular direction is computed from the edge direction
    and flipped if necessary to ensure the offset moves toward +X/+Y.

    Returns:
        Point3d: a point on the offset line (direction unchanged)
    """
    # Left-hand perpendicular: (-y, x)
    perp = rg.Vector3d(-line_dir.Y, line_dir.X, 0)
    perp.Unitize()

    # Ensure perpendicular points toward +X/+Y (positive quadrant)
    # The offset should move the edge line outward from the corner
    if perp.X + perp.Y < 0:
        perp = -perp

    return rg.Point3d(
        line_pt.X + perp.X * offset_distance,
        line_pt.Y + perp.Y * offset_distance,
        line_pt.Z
    )


def compute_center_of_curvature(curve, t=0.5):
    """
    Compute center of curvature at parameter t.

    Returns:
        tuple: (coc_point, radius, midpoint)
    """
    midpoint = curve.PointAt(t)
    curvature_vec = curve.CurvatureAt(t)

    if curvature_vec.Length < 1e-12:
        raise ValueError("Curvature is zero at midpoint — curve is straight")

    radius = 1.0 / curvature_vec.Length

    # Center of curvature = midpoint + (curvature_direction * radius)
    curv_dir = rg.Vector3d(curvature_vec)
    curv_dir.Unitize()
    coc = rg.Point3d(
        midpoint.X + curv_dir.X * radius,
        midpoint.Y + curv_dir.Y * radius,
        midpoint.Z
    )

    return coc, radius, midpoint


def compute_symmetry_axis(cvs):
    """
    Compute the axis of symmetry from edge directions.
    Edge 1: CV0 -> CV3 direction
    Edge 2: CV7 -> CV4 direction
    Symmetry axis bisects these two edge directions.

    Returns:
        Vector3d: normalized symmetry axis direction (pointing from
                  corner region toward the curve's interior/CoC)
    """
    edge1_dir = cvs[3] - cvs[0]
    edge1_dir.Unitize()

    edge2_dir = cvs[4] - cvs[7]
    edge2_dir.Unitize()

    axis = edge1_dir + edge2_dir
    axis.Unitize()
    return axis


# =============================================================================
# PROJECTION CORE
# =============================================================================

def project_cvs_at_displacement(curve, cvs, coc, symmetry_axis,
                                 edge1_dir, edge2_dir,
                                 offset_distance, displacement):
    """
    Project all CVs to generate offset positions for a given
    projection center displacement.

    Algorithm:
    1. Projection center = CoC + displacement * symmetry_axis
    2. Offset each edge line by offset_distance (perpendicular, toward +X/+Y)
    3. For each CV, cast ray from center through CV to offset edge line
    4. Ray-line intersection = new CV position

    Args:
        curve: base NURBS curve
        cvs: list of 8 Point3d (control point locations)
        coc: Point3d (center of curvature at midpoint)
        symmetry_axis: Vector3d (axis direction)
        edge1_dir: Vector3d (direction of edge 1)
        edge2_dir: Vector3d (direction of edge 2)
        offset_distance: target offset
        displacement: projection center displacement from CoC

    Returns:
        list of Point3d: 8 offset CV positions
    """
    # Projection center = CoC + displacement along symmetry axis
    center = rg.Point3d(
        coc.X + symmetry_axis.X * displacement,
        coc.Y + symmetry_axis.Y * displacement,
        coc.Z
    )

    # Offset the two edge lines (perpendicular, toward +X/+Y)
    # Edge 1 line: through CV0, direction edge1_dir
    offset_line1_pt = offset_line_perpendicular(
        cvs[0], edge1_dir, offset_distance
    )

    # Edge 2 line: through CV7, direction edge2_dir
    offset_line2_pt = offset_line_perpendicular(
        cvs[7], edge2_dir, offset_distance
    )

    # Project each CV through ray from center to offset edge line
    new_cvs = []
    for i in range(8):
        cv = cvs[i]

        # Ray direction: from center through CV
        ray_dir = cv - center
        ray_dir.Unitize()

        # CVs 0-3 project to offset edge 1, CVs 4-7 to offset edge 2
        if i < 4:
            hit = line_ray_intersection_2d(
                offset_line1_pt, edge1_dir, center, ray_dir
            )
        else:
            hit = line_ray_intersection_2d(
                offset_line2_pt, edge2_dir, center, ray_dir
            )

        if hit is None:
            raise ValueError(
                f"CV{i} ray does not intersect offset line "
                f"(displacement={displacement:.3f})"
            )

        new_cvs.append(hit)

    return new_cvs


def build_curve_internal(original_curve, new_cvs):
    """
    Build a NurbsCurve preserving the original's knots and weights.
    Used internally for bisection math — PointAt(0.5) matches the
    original's parameterization.

    Returns:
        NurbsCurve
    """
    degree = original_curve.Degree
    point_count = len(new_cvs)

    nc = rg.NurbsCurve(degree, point_count)

    for i in range(original_curve.Knots.Count):
        nc.Knots[i] = original_curve.Knots[i]

    for i in range(point_count):
        w = original_curve.Points[i].Weight
        nc.Points.SetPoint(i, new_cvs[i].X, new_cvs[i].Y, new_cvs[i].Z, w)

    return nc


def build_curve_output(original_curve, new_cvs):
    """
    Build a NurbsCurve for Grasshopper output.
    Uses knot-preserving construction (same structure as original curve).

    Returns:
        NurbsCurve
    """
    return build_curve_internal(original_curve, new_cvs)


def measure_deviation(base_curve, offset_curve, target_offset):
    """
    Measure the deviation at curve midpoint.

    deviation = target_offset - actual_distance
    Positive = undershoot, Negative = overshoot

    Returns:
        float: signed deviation
    """
    base_mid = base_curve.PointAt(0.5)
    offset_mid = offset_curve.PointAt(0.5)

    actual = base_mid.DistanceTo(offset_mid)
    return target_offset - actual


# =============================================================================
# BISECTION SEARCH
# =============================================================================

def find_d_opt(curve, cvs, coc, symmetry_axis,
               edge1_dir, edge2_dir,
               target_offset, tolerance=1e-6, max_iter=60):
    """
    Find optimal projection center displacement via bisection.

    The deviation crosses zero monotonically as displacement changes.
    d_opt is independent of offset distance — the same value works
    for any offset on this curve.

    Args:
        tolerance: convergence threshold for displacement (units)
        max_iter: maximum bisection iterations

    Returns:
        tuple: (d_opt, iterations, final_deviation)
    """
    # Use a reference offset for the search
    ref_offset = target_offset
    if ref_offset < 1.0:
        ref_offset = 10.0  # avoid tiny offsets for numerical stability

    def eval_dev(d):
        try:
            new_cvs = project_cvs_at_displacement(
                curve, cvs, coc, symmetry_axis,
                edge1_dir, edge2_dir, ref_offset, d
            )
            oc = build_curve_internal(curve, new_cvs)
            return measure_deviation(curve, oc, ref_offset)
        except:
            return None

    # Step 1: bracket the zero crossing
    # Scan in steps of 5 to find where deviation changes sign
    d_pos = None  # displacement with positive deviation (undershoot)
    d_neg = None  # displacement with negative deviation (overshoot)

    for d_test_int in range(-50, 51, 5):
        d_test = float(d_test_int)
        dev = eval_dev(d_test)
        if dev is None:
            continue
        if dev > 0 and (d_pos is None or abs(d_test) < abs(d_pos)):
            d_pos = d_test
        elif dev < 0 and (d_neg is None or abs(d_test) < abs(d_neg)):
            d_neg = d_test

    if d_pos is None or d_neg is None:
        raise ValueError(
            "Could not bracket d_opt — deviation does not cross zero "
            "in range [-50, 50]"
        )

    # Ensure d_lo = positive deviation side, d_hi = negative deviation side
    d_lo = d_pos
    d_hi = d_neg

    # Step 2: bisection
    for iteration in range(1, max_iter + 1):
        d_mid = (d_lo + d_hi) / 2.0
        dev_mid = eval_dev(d_mid)

        if dev_mid is None:
            d_hi = d_mid
            continue

        if dev_mid > 0:
            d_lo = d_mid
        else:
            d_hi = d_mid

        if abs(d_hi - d_lo) < tolerance:
            break

    d_opt = (d_lo + d_hi) / 2.0
    final_dev = eval_dev(d_opt)

    return d_opt, iteration, final_dev


# =============================================================================
# MAIN ALGORITHM
# =============================================================================

def offset_curve_projection(curve, offset_distance, projection_offset=0.0):
    """
    Offset a corner blend NURBS curve using the Point Projection Method.

    Expects an 8-CV curve with 45-degree symmetry:
    - CVs 0-3 collinear on edge 1
    - CVs 4-7 collinear on edge 2 (perpendicular to edge 1)

    The optimal projection center is found automatically via bisection.
    Offset direction is toward +X/+Y (outward from corner).

    Args:
        curve: Rhino NurbsCurve
        offset_distance: target offset distance (positive = outward)
        projection_offset: shifts the projection point along the symmetry
            axis after d_opt is found. Positive = away from curve,
            negative = toward curve. (default 0.0)

    Returns:
        tuple: (offset_curve, projection_point, d_opt, deviation,
                diagnostics_text)
    """
    diag = []
    diag.append("=== POINT PROJECTION OFFSET ===")

    # Convert input to NurbsCurve (handles GUIDs from Grasshopper)
    curve = coerce_curve(curve)

    # Extract CVs
    cvs = get_cv_locations(curve)
    n_cvs = len(cvs)
    diag.append(f"Control points: {n_cvs}")

    if n_cvs != 8:
        raise ValueError(
            f"Expected 8 control points, got {n_cvs}. "
            "This tool is designed for 8-CV corner blend curves."
        )

    # Edge directions
    edge1_dir = cvs[3] - cvs[0]
    edge1_dir.Unitize()

    edge2_dir = cvs[4] - cvs[7]
    edge2_dir.Unitize()

    diag.append(f"Edge 1 dir: ({edge1_dir.X:.4f}, {edge1_dir.Y:.4f})")
    diag.append(f"Edge 2 dir: ({edge2_dir.X:.4f}, {edge2_dir.Y:.4f})")

    # Symmetry axis
    symmetry_axis = compute_symmetry_axis(cvs)
    diag.append(f"Symmetry axis: ({symmetry_axis.X:.4f}, {symmetry_axis.Y:.4f})")

    # Center of curvature at midpoint
    coc, radius, midpoint = compute_center_of_curvature(curve, 0.5)
    diag.append(f"Midpoint curvature radius: {radius:.6f}")
    diag.append(f"Center of curvature: ({coc.X:.4f}, {coc.Y:.4f})")
    diag.append(f"Curve midpoint: ({midpoint.X:.4f}, {midpoint.Y:.4f})")

    # Find d_opt via bisection
    diag.append("Running bisection search for d_opt...")
    d_opt, iterations, _ = find_d_opt(
        curve, cvs, coc, symmetry_axis,
        edge1_dir, edge2_dir,
        offset_distance
    )
    diag.append(f"d_opt = {d_opt:.6f} (converged in {iterations} iterations)")

    # Apply projection_offset: shift d_opt along symmetry axis
    # Positive projection_offset moves AWAY from curve (same direction
    # as increasing displacement), negative moves TOWARD curve.
    d_effective = d_opt + projection_offset
    if projection_offset != 0.0:
        diag.append(f"projection_offset: {projection_offset:.6f}")
        diag.append(f"d_effective = d_opt + offset = {d_effective:.6f}")

    # Compute projection point (for visualization)
    projection_point = rg.Point3d(
        coc.X + symmetry_axis.X * d_effective,
        coc.Y + symmetry_axis.Y * d_effective,
        coc.Z
    )
    diag.append(f"Projection point: ({projection_point.X:.4f}, {projection_point.Y:.4f}, {projection_point.Z:.4f})")

    # Generate final offset curve
    new_cvs = project_cvs_at_displacement(
        curve, cvs, coc, symmetry_axis,
        edge1_dir, edge2_dir,
        offset_distance, d_effective
    )

    # Build internal curve (knot-preserving) for deviation measurement
    internal_crv = build_curve_internal(curve, new_cvs)

    # Measure deviation using the internal curve
    dev = measure_deviation(curve, internal_crv, offset_distance)
    deviation = dev

    diag.append(f"--- Results ---")
    diag.append(f"Target offset: {offset_distance}")
    diag.append(f"Actual midpoint offset: {offset_distance - dev:.6f}")
    diag.append(f"Deviation: {deviation:.6f}")

    # Build output curve for Grasshopper display
    offset_crv = build_curve_output(curve, new_cvs)
    diag.append(f"Output curve type: {type(offset_crv).__name__}")
    diag.append(f"Output curve valid: {offset_crv.IsValid if offset_crv else 'None'}")

    # CV positions for reference
    diag.append("--- Offset CVs ---")
    for i, pt in enumerate(new_cvs):
        diag.append(f"  CV{i}: ({pt.X:.4f}, {pt.Y:.4f})")

    return offset_crv, projection_point, d_opt, deviation, "\n".join(diag)


# =============================================================================
# GRASSHOPPER SCRIPT EXECUTION
# =============================================================================

try:
    # Default projection_offset to 0 if not provided
    if 'projection_offset' not in dir() or projection_offset is None:
        projection_offset = 0.0

    offset_curve, projection_point, d_opt, deviation, diagnostics = \
        offset_curve_projection(curve, offset_distance, projection_offset)

except Exception as e:
    import traceback
    diagnostics = "ERROR:\n" + str(e) + "\n\n" + traceback.format_exc()
    offset_curve = None
    projection_point = None
    d_opt = 0
    deviation = 0

# Restore GH document context so Grasshopper can read geometry outputs.
# rhinoscriptsyntax may have switched sc.doc to Rhino's ActiveDoc,
# which prevents GH from interpreting curve outputs correctly.
sc.doc = _ghdoc


"""
================================================================================
USAGE IN GRASSHOPPER:
================================================================================

1. Add a Python Script component (GhPython)

2. Copy this entire script into the component

3. Set up inputs (right-click component, "Set input hints"):
   - curve: Curve
   - offset_distance: float
   - projection_offset: float

4. Set up outputs (right-click component, "Set output hints"):
   - offset_curve: Curve
   - projection_point: Point3d
   - d_opt: float
   - deviation: float
   - diagnostics: str

5. Connect:
   - Input curve -> curve parameter
   - Number slider (0.1 to 100) -> offset_distance
   - Number slider (-50 to 50, default 0) -> projection_offset
   - Output to preview/panel

================================================================================
NOTES:
================================================================================

- Designed for 8-CV corner blend curves with 45-degree symmetry
- CVs 0-3 must be collinear on edge 1
- CVs 4-7 must be collinear on edge 2 (perpendicular)
- Offset direction: +X/+Y (outward from corner)
- d_opt is found automatically via bisection (~20 iterations)
- d_opt is independent of offset distance — same value for all offsets
- Typical deviation: < 0.0002 units for a 1.0 unit offset

================================================================================
"""
