"""
================================================================================
NURBS CURVE OFFSET - POINT PROJECTION METHOD
Rhino Python Command Script (no Grasshopper required)
================================================================================

Run this script in Rhino via:
  - EditPythonScript > Open > Run
  - Or drag and drop onto the Rhino viewport
  - Or _RunPythonScript command

The script will prompt you to:
  1. Select an 8-CV corner blend NURBS curve
  2. Enter an offset distance
  3. Optionally enter a projection point offset

Results are added directly to the Rhino document.

================================================================================
"""

import Rhino
import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
import scriptcontext as sc
import math


# =============================================================================
# GEOMETRY UTILITIES
# =============================================================================

def get_cv_locations(curve):
    """Extract control point locations from a NURBS curve."""
    pts = []
    for i in range(curve.Points.Count):
        pts.append(curve.Points[i].Location)
    return pts


def line_ray_intersection_2d(line_pt, line_dir, ray_origin, ray_dir):
    """
    Find intersection of a line and a ray in 2D (XY plane).
    Returns Point3d or None.
    """
    det = ray_dir.X * line_dir.Y - ray_dir.Y * line_dir.X
    if abs(det) < 1e-10:
        return None

    dx = line_pt.X - ray_origin.X
    dy = line_pt.Y - ray_origin.Y

    s = (dx * line_dir.Y - dy * line_dir.X) / det

    if s < -1e-6:
        return None

    return rg.Point3d(
        ray_origin.X + s * ray_dir.X,
        ray_origin.Y + s * ray_dir.Y,
        ray_origin.Z
    )


def offset_line_perpendicular(line_pt, line_dir, offset_distance):
    """Offset a line in the +X/+Y direction (away from corner, outward)."""
    perp = rg.Vector3d(-line_dir.Y, line_dir.X, 0)
    perp.Unitize()

    if perp.X + perp.Y < 0:
        perp = -perp

    return rg.Point3d(
        line_pt.X + perp.X * offset_distance,
        line_pt.Y + perp.Y * offset_distance,
        line_pt.Z
    )


def compute_center_of_curvature(curve, t=0.5):
    """Compute center of curvature at parameter t."""
    midpoint = curve.PointAt(t)
    curvature_vec = curve.CurvatureAt(t)

    if curvature_vec.Length < 1e-12:
        raise ValueError("Curvature is zero at midpoint")

    radius = 1.0 / curvature_vec.Length

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
    Bisects edge 1 (CV0->CV3) and edge 2 (CV7->CV4).
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
    """Project all CVs to generate offset positions."""
    center = rg.Point3d(
        coc.X + symmetry_axis.X * displacement,
        coc.Y + symmetry_axis.Y * displacement,
        coc.Z
    )

    offset_line1_pt = offset_line_perpendicular(
        cvs[0], edge1_dir, offset_distance
    )
    offset_line2_pt = offset_line_perpendicular(
        cvs[7], edge2_dir, offset_distance
    )

    new_cvs = []
    for i in range(8):
        cv = cvs[i]

        ray_dir = cv - center
        ray_dir.Unitize()

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


def build_curve(original_curve, new_cvs):
    """Build a NurbsCurve preserving the original's knots and weights."""
    degree = original_curve.Degree
    point_count = len(new_cvs)

    nc = rg.NurbsCurve(degree, point_count)

    for i in range(original_curve.Knots.Count):
        nc.Knots[i] = original_curve.Knots[i]

    for i in range(point_count):
        w = original_curve.Points[i].Weight
        nc.Points.SetPoint(i, new_cvs[i].X, new_cvs[i].Y, new_cvs[i].Z, w)

    return nc


def measure_deviation(base_curve, offset_curve, target_offset):
    """Measure deviation at curve midpoint (target - actual)."""
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
    """Find optimal projection center displacement via bisection."""
    ref_offset = target_offset
    if ref_offset < 1.0:
        ref_offset = 10.0

    def eval_dev(d):
        try:
            new_cvs = project_cvs_at_displacement(
                curve, cvs, coc, symmetry_axis,
                edge1_dir, edge2_dir, ref_offset, d
            )
            oc = build_curve(curve, new_cvs)
            return measure_deviation(curve, oc, ref_offset)
        except:
            return None

    d_pos = None
    d_neg = None

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
            "Could not bracket d_opt in range [-50, 50]"
        )

    d_lo = d_pos
    d_hi = d_neg

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

    Args:
        curve: Rhino NurbsCurve (8 CVs, 45-degree symmetry)
        offset_distance: target offset distance (positive = outward)
        projection_offset: shifts projection point along symmetry axis
            (positive = away from curve, negative = toward)

    Returns:
        tuple: (offset_curve, projection_point, d_opt, deviation)
    """
    # Validate
    if not isinstance(curve, rg.NurbsCurve):
        curve = curve.ToNurbsCurve()

    cvs = get_cv_locations(curve)
    if len(cvs) != 8:
        raise ValueError(
            f"Expected 8 control points, got {len(cvs)}. "
            "This tool is designed for 8-CV corner blend curves."
        )

    # Edge directions
    edge1_dir = cvs[3] - cvs[0]
    edge1_dir.Unitize()

    edge2_dir = cvs[4] - cvs[7]
    edge2_dir.Unitize()

    # Symmetry axis
    symmetry_axis = compute_symmetry_axis(cvs)

    # Center of curvature at midpoint
    coc, radius, midpoint = compute_center_of_curvature(curve, 0.5)

    # Find d_opt via bisection
    d_opt, iterations, _ = find_d_opt(
        curve, cvs, coc, symmetry_axis,
        edge1_dir, edge2_dir,
        offset_distance
    )

    # Apply projection_offset
    d_effective = d_opt + projection_offset

    # Compute projection point
    projection_point = rg.Point3d(
        coc.X + symmetry_axis.X * d_effective,
        coc.Y + symmetry_axis.Y * d_effective,
        coc.Z
    )

    # Generate final offset curve
    new_cvs = project_cvs_at_displacement(
        curve, cvs, coc, symmetry_axis,
        edge1_dir, edge2_dir,
        offset_distance, d_effective
    )

    offset_crv = build_curve(curve, new_cvs)

    # Measure deviation
    dev = measure_deviation(curve, offset_crv, offset_distance)

    return offset_crv, projection_point, d_opt, dev


# =============================================================================
# RHINO COMMAND INTERFACE
# =============================================================================

def run():
    """Interactive Rhino command — prompts user and adds results to document."""

    # 1. Select curve
    curve_id = rs.GetObject(
        "Select an 8-CV corner blend curve to offset",
        rs.filter.curve,
        preselect=True
    )
    if curve_id is None:
        return

    curve_obj = rs.coercecurve(curve_id)
    if curve_obj is None:
        print("Error: could not read curve geometry.")
        return

    nurbs_curve = curve_obj.ToNurbsCurve()
    cv_count = nurbs_curve.Points.Count
    if cv_count != 8:
        print(f"Error: expected 8 control points, got {cv_count}.")
        return

    # 2. Get offset distance
    offset_distance = rs.GetReal(
        "Offset distance (positive = outward)",
        number=1.0,
        minimum=0.001
    )
    if offset_distance is None:
        return

    # 3. Get projection offset (optional)
    projection_offset = rs.GetReal(
        "Projection point offset along symmetry axis (0 = optimal)",
        number=0.0
    )
    if projection_offset is None:
        projection_offset = 0.0

    # 4. Run the algorithm
    try:
        offset_crv, proj_pt, d_opt, deviation = offset_curve_projection(
            nurbs_curve, offset_distance, projection_offset
        )
    except Exception as e:
        print(f"Error: {e}")
        return

    # 5. Add results to Rhino document
    sc.doc = Rhino.RhinoDoc.ActiveDoc

    # Add offset curve
    curve_guid = sc.doc.Objects.AddCurve(offset_crv)
    if curve_guid:
        print(f"Offset curve added.")

    # Add projection point
    point_guid = sc.doc.Objects.AddPoint(proj_pt)
    if point_guid:
        print(f"Projection point added at ({proj_pt.X:.4f}, {proj_pt.Y:.4f}, {proj_pt.Z:.4f})")

    # Report
    print(f"d_opt: {d_opt:.6f}")
    print(f"Deviation: {deviation:.6f}")

    if projection_offset != 0.0:
        print(f"Projection offset applied: {projection_offset:.6f}")

    sc.doc.Views.Redraw()


# Run the command
if __name__ == "__main__" or True:
    run()
