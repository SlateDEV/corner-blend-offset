# NURBS corner blend offset for Rhino 3D

Offset G2/G3 corner blend NURBS curves using a point projection algorithm. Works in both Grasshopper and standalone Rhino.

## What It Does

Standard curve offset tools (Rhino's `OffsetCrv`, Grasshopper's `Offset Curve`) produce precise approximations but don't preserve the control vertex structure of the original curve or degrade curve's quality. This tool offsets corner blend curves by projecting each CV through a computed center point onto offset edge lines, preserving the NURBS structure exactly.

The optimal projection center is found automatically via bisection search. The result is a clean NURBS curve with the same degree, knot structure, and weight distribution as the input, offset by a specified distance.

### Curve Requirements

- **8 control vertices** arranged as a corner blend
- CVs 0–3 collinear on one edge, CVs 4–7 collinear on the perpendicular edge
- 45-degree symmetry (the two edges are perpendicular)

## Installation

### Grasshopper (GhPython component)

1. Add a **GhPython Script** component to your canvas
2. Copy the contents of [`grasshopper/projection_offset.py`](grasshopper/projection_offset.py) into the component
3. Set up inputs and outputs as described in the script header
4. Connect a curve and offset distance slider

Or, if a `.ghuser` file is provided in Releases, drop it into your Grasshopper UserObjects folder — see [`grasshopper/CREATE_GHUSER.md`](grasshopper/CREATE_GHUSER.md) for details.

### Rhino (standalone script, no Grasshopper)

1. Open Rhino
2. Run `EditPythonScript` or `_RunPythonScript`
3. Open [`rhino/projection_offset_cmd.py`](rhino/projection_offset_cmd.py)
4. Run the script — it will prompt you to select a curve and enter values
5. The offset curve and projection point are added directly to the document

## Inputs & Outputs

### Inputs

| Name | Type | Description |
|------|------|-------------|
| `curve` | Curve | 8-CV corner blend NURBS curve |
| `offset_distance` | Float | Distance to offset (positive = outward from corner) |
| `projection_offset` | Float | Shifts the projection point along the curve's axis of symmetry. `0` = optimal. Positive = away from curve, negative = toward it. |

### Outputs

| Name | Type | Description |
|------|------|-------------|
| `offset_curve` | Curve | Resulting offset NURBS curve |
| `projection_point` | Point3d | The projection center used (for visualization) |
| `d_opt` | Float | Optimal projection center displacement found by bisection |
| `deviation` | Float | Offset deviation at curve midpoint, in document units |
| `diagnostics` | String | Diagnostic log (Grasshopper version only) |

## How It Works

1. **Extract geometry** from the input curve: edge directions, symmetry axis, and center of curvature at the midpoint.

2. **Bisection search** finds the optimal projection center displacement (`d_opt`) along the symmetry axis. This is the point where projecting CVs produces an offset curve whose midpoint lands exactly at the target distance.

3. **Project CVs**: from the projection center, cast a ray through each original CV to the corresponding offset edge line. The ray-line intersection gives the new CV position.

4. **Build the offset curve** using the original knot vector and weights with the new CV positions.

The `projection_offset` input lets you manually shift the projection center away from optimal — useful for exploring how the projection geometry affects the result, or for artistic control.

## Accuracy

Typical deviation at the curve midpoint is under 0.0002 units for a 1.0 unit offset (with `projection_offset = 0`). The bisection converges in roughly 20 iterations.

## License

MIT — see [LICENSE](LICENSE).
