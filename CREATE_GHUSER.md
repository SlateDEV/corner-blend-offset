# Creating a .ghuser Component

A `.ghuser` file is a self-contained Grasshopper component that users can install by dropping it into their UserObjects folder. This packages the Python script with pre-configured inputs and outputs so there's zero setup.

## Steps

1. **Open Grasshopper** and create a new definition.

2. **Add a GhPython Script component** (Maths > Script > GhPython Script).

3. **Configure inputs** — right-click the component, manage inputs:

   | Name              | Type Hint | Default |
   |-------------------|-----------|---------|
   | curve             | Curve     | —       |
   | offset_distance   | float     | 1.0     |
   | projection_offset | float     | 0.0     |

4. **Configure outputs** — right-click the component, manage outputs:

   | Name             | Type Hint |
   |------------------|-----------|
   | offset_curve     | Curve     |
   | projection_point | Point3d   |
   | d_opt            | float     |
   | deviation        | float     |
   | diagnostics      | str       |

5. **Paste the script** — double-click the component to open the editor, paste the entire contents of `projection_offset.py`.

6. **Set component identity** — right-click the component:
   - Name: `Projection Offset`
   - Nickname: `ProjOff`
   - Description: `Offset an 8-CV corner blend NURBS curve using the Point Projection Method`
   - Category: `Curve` (or your preferred tab)
   - Subcategory: `Offset`

7. **Save as UserObject** — right-click the component > "Create User Object..."
   - This saves a `.ghuser` file to:
     - Windows: `%AppData%\Grasshopper\UserObjects\`
     - Mac: `~/Library/Application Support/McNeel/Rhinoceros/Grasshopper/UserObjects/`

8. **Test** — restart Grasshopper. The component should appear in the toolbar under the category you chose.

## Distributing the .ghuser

Share the `.ghuser` file directly. Users install it by copying it to their UserObjects folder (same paths as above). The component will appear after restarting Grasshopper.

You can also upload it to:
- [food4Rhino](https://www.food4rhino.com/) — the main Rhino/Grasshopper plugin directory
- [Grasshopper forum](https://discourse.mcneel.com/c/grasshopper) — community sharing
- This GitHub repository's Releases page
