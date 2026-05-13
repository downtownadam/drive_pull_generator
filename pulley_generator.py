#!/usr/bin/env python3
"""
Drive Pulley STL Generator

Generates a parametric V-groove drive pulley as two separate STL halves
that can be 3D printed on their sides without supports.

The pulley profile (cross-section) looks like this:

    |         |           |         |
    |  flange |  V-groove |  flange |
    |_________|    /\     |_________|
              |   /  \    |
              |  / .. \   |      <-- valley diameter
              | /  ..  \  |
    __________|/  ....  \|__________
    |         |  bore   |         |
    |   shaft large     shaft threaded
    |         |  side   |  side   |

Split plane is at the widest point (flanges), so each half
prints flat on the split face.
"""

import cadquery as cq
import argparse
import math
import os


def create_pulley_half(
    pulley_width: float,
    shaft_dia_large: float,
    shaft_dia_threaded: float,
    threaded_side_width: float,
    valley_diameter: float,
    flange_diameter: float,
    groove_angle: float = 38.0,
    set_screw_dia: float = 0.0,
    weight_reduction_cutouts: bool = False,
    cutout_count: int = 6,
    is_large_side: bool = True,
) -> cq.Workplane:
    """
    Create one half of the drive pulley.

    The pulley is split at the midplane of the V-groove so each half
    can be printed lying on the flat split face (no supports needed).

    Parameters
    ----------
    pulley_width : float
        Total width of the assembled pulley (mm).
    shaft_dia_large : float
        Bore diameter on the large/keyed side (mm).
    shaft_dia_threaded : float
        Bore diameter on the threaded/small side (mm).
    threaded_side_width : float
        Width of shaft bore on the threaded side (mm).
        The large side gets the remainder.
    valley_diameter : float
        Diameter at the deepest point of the V-groove (mm).
    flange_diameter : float
        Outer diameter at the flanges/edges of the pulley (mm).
    groove_angle : float
        Included angle of the V-groove in degrees (default 38).
    set_screw_dia : float
        If > 0, adds a radial set screw hole of this diameter (mm).
    weight_reduction_cutouts : bool
        Whether to add triangular cutouts in the flanges.
    cutout_count : int
        Number of triangular cutouts around the circumference.
    is_large_side : bool
        True = large bore side, False = threaded bore side.
    """

    # Derived dimensions
    large_side_width = pulley_width - threaded_side_width
    half_width = large_side_width if is_large_side else threaded_side_width
    shaft_dia = shaft_dia_large if is_large_side else shaft_dia_threaded

    valley_radius = valley_diameter / 2.0
    flange_radius = flange_diameter / 2.0
    shaft_radius = shaft_dia / 2.0

    # The V-groove depth from flange edge to valley
    groove_depth = flange_radius - valley_radius

    # Half the groove angle for computing the groove wall slope
    half_angle_rad = math.radians(groove_angle / 2.0)

    # The groove wall width (horizontal extent from valley to flange OD)
    # at the split plane based on the groove angle
    groove_wall_horizontal = groove_depth * math.tan(half_angle_rad)

    # Ensure the groove wall fits within the half width
    # If the groove is wider than the half, clamp it
    groove_wall_horizontal = min(groove_wall_horizontal, half_width * 0.8)

    # Recalculate what the actual groove depth is given the clamped width
    actual_groove_depth = groove_wall_horizontal / math.tan(half_angle_rad)
    actual_valley_radius = flange_radius - actual_groove_depth

    # Build the profile to revolve (in the XZ plane, revolved around Z axis)
    # We build from the bore outward, bottom to top:
    #
    # Profile points (r, z) where r=radius from center, z=axial position
    # z=0 is the split face (groove midplane), z=half_width is the outer face

    # Hub section - the cylindrical part around the shaft
    hub_outer_radius = valley_radius - 1.0  # 1mm wall minimum around bore
    if hub_outer_radius < shaft_radius + 2.0:
        hub_outer_radius = shaft_radius + 2.0

    # Build the 2D profile for revolution
    # We'll create this as a series of points forming a closed polygon
    profile_points = []

    # Start at inner bore, at the split face (z=0)
    # Go clockwise: bore up, across top, down OD groove, across split face

    # Inner bore wall (bottom to top)
    profile_points.append((shaft_radius, 0.0))           # A: bore at split face
    profile_points.append((shaft_radius, half_width))     # B: bore at outer face

    # Outer face (top), from bore to flange OD
    profile_points.append((flange_radius, half_width))    # C: flange OD at outer face

    # Down the flange OD to the groove
    # The flange extends from z=half_width down to z=groove_wall_horizontal
    flange_flat_start = groove_wall_horizontal
    profile_points.append((flange_radius, flange_flat_start))  # D: start of groove wall

    # Groove wall slopes from flange_radius down to actual_valley_radius at z=0
    profile_points.append((actual_valley_radius, 0.0))    # E: valley at split face

    # Close back to start
    # (CadQuery polyline will close automatically)

    # Create the 2D profile and revolve it
    # Use CadQuery workplane approach - build profile in XZ, revolve around Z

    # We need to build this as a wire and revolve. CadQuery's revolve works
    # on a 2D sketch that gets revolved around an axis.

    # Build the profile using the polyline approach on a workplane
    # The workplane "XZ" means X=radial, Z=axial (Y is the revolve direction)

    result = (
        cq.Workplane("XZ")
        .polyline(profile_points)
        .close()
        .revolve(360, (0, 0, 0), (0, 0, 1))
    )

    # Add alignment features - small peg on one side, hole on the other
    peg_radius = 1.5  # mm
    peg_height = 3.0  # mm
    peg_position_radius = (valley_radius + shaft_radius) / 2.0

    if is_large_side:
        # Add pegs (2 pegs, 180 degrees apart for alignment)
        for angle in [0, 180]:
            x = peg_position_radius * math.cos(math.radians(angle))
            y = peg_position_radius * math.sin(math.radians(angle))
            peg = (
                cq.Workplane("XY")
                .transformed(offset=(x, y, 0))
                .circle(peg_radius)
                .extrude(-peg_height / 2)  # protrude into split face
            )
            result = result.union(peg)
    else:
        # Add peg holes (matching the pegs on the large side)
        for angle in [0, 180]:
            x = peg_position_radius * math.cos(math.radians(angle))
            y = peg_position_radius * math.sin(math.radians(angle))
            hole = (
                cq.Workplane("XY")
                .transformed(offset=(x, y, 0))
                .circle(peg_radius + 0.15)  # slight clearance
                .extrude(-peg_height / 2 - 0.5)  # slightly deeper for clearance
            )
            result = result.cut(hole)

    # Optional set screw hole
    if set_screw_dia > 0:
        set_screw = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, half_width / 2))
            .transformed(rotate=(0, 90, 0))
            .circle(set_screw_dia / 2.0)
            .extrude(flange_radius)
        )
        result = result.cut(set_screw)

    # Optional weight reduction cutouts
    if weight_reduction_cutouts and flange_radius > valley_radius + 5:
        cutout_inner_r = shaft_radius + 3.0  # clearance from bore
        cutout_outer_r = valley_radius - 2.0  # clearance from groove

        if cutout_outer_r > cutout_inner_r + 3.0:
            # Triangular cutouts through the flange section
            cutout_depth = half_width * 0.6  # don't go all the way through
            cutout_z_start = half_width * 0.2

            for i in range(cutout_count):
                angle = (360.0 / cutout_count) * i
                angle_rad = math.radians(angle)
                half_arc = math.radians(360.0 / cutout_count / 3.0)

                mid_r = (cutout_inner_r + cutout_outer_r) / 2.0

                # Triangle vertices in XY plane
                p1_r, p1_a = cutout_inner_r, angle_rad
                p2_r, p2_a = cutout_outer_r, angle_rad - half_arc
                p3_r, p3_a = cutout_outer_r, angle_rad + half_arc

                p1 = (p1_r * math.cos(p1_a), p1_r * math.sin(p1_a))
                p2 = (p2_r * math.cos(p2_a), p2_r * math.sin(p2_a))
                p3 = (p3_r * math.cos(p3_a), p3_r * math.sin(p3_a))

                cutout = (
                    cq.Workplane("XY")
                    .transformed(offset=(0, 0, cutout_z_start))
                    .moveTo(p1[0], p1[1])
                    .lineTo(p2[0], p2[1])
                    .lineTo(p3[0], p3[1])
                    .close()
                    .extrude(cutout_depth)
                )
                result = result.cut(cutout)

    return result


def generate_pulley(
    pulley_width: float = 25.0,
    shaft_dia_large: float = 12.0,
    shaft_dia_threaded: float = 8.0,
    threaded_side_width: float = 8.0,
    valley_diameter: float = 40.0,
    flange_diameter: float = 60.0,
    groove_angle: float = 38.0,
    set_screw_dia: float = 3.0,
    weight_reduction_cutouts: bool = False,
    cutout_count: int = 6,
    output_dir: str = "output",
):
    """Generate both halves of the pulley and export as STL files."""

    os.makedirs(output_dir, exist_ok=True)

    print(f"Generating drive pulley with parameters:")
    print(f"  Pulley width:          {pulley_width} mm")
    print(f"  Shaft dia (large):     {shaft_dia_large} mm")
    print(f"  Shaft dia (threaded):  {shaft_dia_threaded} mm")
    print(f"  Threaded side width:   {threaded_side_width} mm")
    print(f"  Large side width:      {pulley_width - threaded_side_width} mm")
    print(f"  Valley diameter:       {valley_diameter} mm")
    print(f"  Flange diameter:       {flange_diameter} mm")
    print(f"  V-groove angle:        {groove_angle} deg")
    print(f"  Set screw diameter:    {set_screw_dia} mm")
    print(f"  Weight cutouts:        {weight_reduction_cutouts}")
    if weight_reduction_cutouts:
        print(f"  Cutout count:          {cutout_count}")
    print()

    # Validation
    if valley_diameter >= flange_diameter:
        raise ValueError("Valley diameter must be less than flange diameter")
    if shaft_dia_large <= 0 or shaft_dia_threaded <= 0:
        raise ValueError("Shaft diameters must be positive")
    if threaded_side_width >= pulley_width:
        raise ValueError("Threaded side width must be less than total pulley width")
    if valley_diameter <= max(shaft_dia_large, shaft_dia_threaded) + 4:
        raise ValueError("Valley diameter must be larger than shaft diameter + 4mm wall")

    common_args = dict(
        pulley_width=pulley_width,
        shaft_dia_large=shaft_dia_large,
        shaft_dia_threaded=shaft_dia_threaded,
        threaded_side_width=threaded_side_width,
        valley_diameter=valley_diameter,
        flange_diameter=flange_diameter,
        groove_angle=groove_angle,
        set_screw_dia=set_screw_dia,
        weight_reduction_cutouts=weight_reduction_cutouts,
        cutout_count=cutout_count,
    )

    print("Generating large side half...")
    large_half = create_pulley_half(**common_args, is_large_side=True)
    large_stl = os.path.join(output_dir, "pulley_large_side.stl")
    cq.exporters.export(large_half, large_stl, cq.exporters.ExportTypes.STL)
    print(f"  Saved: {large_stl}")

    print("Generating threaded side half...")
    threaded_half = create_pulley_half(**common_args, is_large_side=False)
    threaded_stl = os.path.join(output_dir, "pulley_threaded_side.stl")
    cq.exporters.export(threaded_half, threaded_stl, cq.exporters.ExportTypes.STL)
    print(f"  Saved: {threaded_stl}")

    # Also export a STEP file of the assembled pulley for visualization
    print("Generating assembled preview (STEP)...")
    # Mirror the threaded half and combine for preview
    assembled = large_half
    # The threaded side needs to be flipped (mirrored on the split plane)
    threaded_flipped = threaded_half.mirror("XY")
    assembled = assembled.union(threaded_flipped)
    assembled_step = os.path.join(output_dir, "pulley_assembled.step")
    cq.exporters.export(assembled, assembled_step, cq.exporters.ExportTypes.STEP)
    print(f"  Saved: {assembled_step}")

    print("\nDone! Print each half lying on the flat (split) face.")
    print("Assemble with the alignment pegs and secure the shaft.")


def main():
    parser = argparse.ArgumentParser(
        description="Generate a parametric V-groove drive pulley as two printable STL halves.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--pulley-width", type=float, default=25.0,
                        help="Total width of the assembled pulley (mm)")
    parser.add_argument("--shaft-dia-large", type=float, default=12.0,
                        help="Bore diameter on the large/keyed side (mm)")
    parser.add_argument("--shaft-dia-threaded", type=float, default=8.0,
                        help="Bore diameter on the threaded/small side (mm)")
    parser.add_argument("--threaded-side-width", type=float, default=8.0,
                        help="Width of shaft bore on the threaded side (mm)")
    parser.add_argument("--valley-diameter", type=float, default=40.0,
                        help="Diameter at the bottom of the V-groove (mm)")
    parser.add_argument("--flange-diameter", type=float, default=60.0,
                        help="Outer diameter at the pulley flanges (mm)")
    parser.add_argument("--groove-angle", type=float, default=38.0,
                        help="Included angle of the V-groove (degrees)")
    parser.add_argument("--set-screw-dia", type=float, default=3.0,
                        help="Set screw hole diameter, 0 to disable (mm)")
    parser.add_argument("--weight-cutouts", action="store_true",
                        help="Add triangular weight reduction cutouts")
    parser.add_argument("--cutout-count", type=int, default=6,
                        help="Number of triangular cutouts")
    parser.add_argument("--output-dir", type=str, default="output",
                        help="Output directory for STL files")

    args = parser.parse_args()

    generate_pulley(
        pulley_width=args.pulley_width,
        shaft_dia_large=args.shaft_dia_large,
        shaft_dia_threaded=args.shaft_dia_threaded,
        threaded_side_width=args.threaded_side_width,
        valley_diameter=args.valley_diameter,
        flange_diameter=args.flange_diameter,
        groove_angle=args.groove_angle,
        set_screw_dia=args.set_screw_dia,
        weight_reduction_cutouts=args.weight_cutouts,
        cutout_count=args.cutout_count,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
