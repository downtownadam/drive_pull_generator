#!/usr/bin/env python3
"""
Drive Pulley / Spool STL Generator

Generates a parametric drive pulley (spool) as two separate STL halves
that can be 3D printed without supports.

Cross-section profile for one half:

    r (radius)
    ^
    |  flange_r  |_|  <- thin flange wall (with optional cutouts)
    |            | |
    |  valley_r  | |________________________
    |            |      solid disc          |  <- string winds on this surface
    |  shaft_r   | - - - - bore - - - - - - |
    +------------------------------------------------> z (axial)
                z=0 (split face)      z=half_width (outer)

Both halves meet flush at z=0 across the full face.
STLs are oriented with the outer face (flange) down on the build plate.
"""

import cadquery as cq
import argparse
import math
import os


def create_pulley_half(
    large_side_width: float,
    threaded_side_width: float,
    shaft_dia_large: float,
    shaft_dia_threaded: float,
    valley_diameter: float,
    flange_diameter: float,
    flange_thickness: float = 2.0,
    nut_width_af: float = 0.0,
    nut_thickness: float = 0.0,
    bolt_count: int = 4,
    heat_insert_od: float = 5.0,
    heat_insert_length: float = 4.0,
    bolt_clearance_dia: float = 3.4,
    bolt_head_dia: float = 5.5,
    bolt_head_depth: float = 3.2,
    flange_cutout_count: int = 6,
    build_plate_chamfer: float = 1.0,
    bore_compensation: float = 0.2,
    is_large_side: bool = True,
) -> cq.Workplane:
    """
    Create one half of the drive pulley / spool.

    Parameters
    ----------
    large_side_width : float
        Width of the large/keyed side half (mm).
    threaded_side_width : float
        Width of the threaded/nut side half (mm).
    shaft_dia_large : float
        Bore diameter on the large/keyed side (mm).
    shaft_dia_threaded : float
        Bore diameter on the threaded/small side (mm).
    valley_diameter : float
        Diameter at the valley floor where string winds (mm).
    flange_diameter : float
        Outer diameter at the flanges/edges of the pulley (mm).
    flange_thickness : float
        Thickness of the flange walls at the edges (mm).
    nut_width_af : float
        Nut width across-flats in mm. If > 0, a hex nut recess
        is cut into the outer face of the threaded side.
    nut_thickness : float
        Nut thickness/height in mm.
    bolt_count : int
        Number of bolts in the bolt circle (0 to disable).
    heat_insert_od : float
        Outer diameter of heat-set inserts (mm).
    heat_insert_length : float
        Length of heat-set inserts (mm).
    bolt_clearance_dia : float
        Clearance hole diameter for the bolt shank (mm).
    bolt_head_dia : float
        Counterbore diameter for the bolt head (mm).
    bolt_head_depth : float
        Counterbore depth for the bolt head (mm).
    flange_cutout_count : int
        Number of cutout windows in the flange (0 = solid flange).
    build_plate_chamfer : float
        Size of the 45-deg chamfer applied to all edges on the build-plate
        face after orientation (mm). Set to 0 to disable. Helps mask
        elephant's foot on the first layer.
    bore_compensation : float
        Amount added to each bore diameter in the model to offset typical
        print shrinkage of inner holes (mm). With the default 0.2 mm, a
        shaft_dia of 8.0 produces a modeled bore of 8.2 mm that should
        print at ~8.0 mm. Only affects the bore profile, not the bolt
        circle or pin positions.
    is_large_side : bool
        True = large bore side, False = threaded bore side.
    """

    # Derived dimensions
    half_width = large_side_width if is_large_side else threaded_side_width
    shaft_dia = shaft_dia_large if is_large_side else shaft_dia_threaded

    valley_radius = valley_diameter / 2.0
    flange_radius = flange_diameter / 2.0
    shaft_radius = shaft_dia / 2.0
    # Bore is modeled slightly larger than nominal to compensate for the
    # typical undersize of printed inner holes. shaft_radius (without
    # compensation) is still used for the bolt circle / pin positions.
    bore_radius = (shaft_dia + bore_compensation) / 2.0

    # Simple spool profile: solid disc from shaft to valley, thin flange at edge
    # Both halves meet flush at z=0 across the full face.
    #
    #   A (shaft, 0) -> B (shaft, hw) -> C (flange, hw)
    #   -> D (flange, hw-ft) -> E (valley, hw-ft) -> F (valley, 0)
    #   -> close to A
    profile_points = [
        (bore_radius, 0.0),                                 # A: bore at split face
        (bore_radius, half_width),                          # B: bore at outer face
        (flange_radius, half_width),                        # C: flange OD at outer face
        (flange_radius, half_width - flange_thickness),     # D: flange inner edge
        (valley_radius, half_width - flange_thickness),     # E: valley at flange
        (valley_radius, 0.0),                               # F: valley at split face
    ]

    # Revolve the profile around the Z axis
    result = (
        cq.Workplane("XZ")
        .polyline(profile_points)
        .close()
        .revolve()
    )

    # --- Flange cutout windows ---
    # These are slots through the thin flange wall to reduce weight
    # and since the flange is non-structural (just guides string).
    flange_height = flange_radius - valley_radius  # radial height of flange
    if flange_cutout_count > 0 and flange_height > 6.0 and flange_thickness > 0:
        cutout_inner_r = valley_radius + 2.0
        cutout_outer_r = flange_radius - 2.0
        slot_width_angle = 10.0  # degrees of arc for each rib between slots

        if cutout_outer_r > cutout_inner_r + 2.0:
            arc_per_slot = 360.0 / flange_cutout_count
            cutout_arc = arc_per_slot - slot_width_angle

            if cutout_arc > 5.0:
                for i in range(flange_cutout_count):
                    center_angle = (arc_per_slot * i) + slot_width_angle / 2.0
                    start_angle = math.radians(center_angle)
                    end_angle = math.radians(center_angle + cutout_arc)

                    n_arc_pts = 8
                    pts = []
                    # Inner arc
                    for j in range(n_arc_pts + 1):
                        a = start_angle + (end_angle - start_angle) * j / n_arc_pts
                        pts.append((cutout_inner_r * math.cos(a),
                                    cutout_inner_r * math.sin(a)))
                    # Outer arc (reverse)
                    for j in range(n_arc_pts, -1, -1):
                        a = start_angle + (end_angle - start_angle) * j / n_arc_pts
                        pts.append((cutout_outer_r * math.cos(a),
                                    cutout_outer_r * math.sin(a)))

                    cutout = (
                        cq.Workplane("XY")
                        .transformed(offset=(0, 0, half_width - flange_thickness))
                        .moveTo(pts[0][0], pts[0][1])
                        .polyline(pts[1:])
                        .close()
                        .extrude(flange_thickness)
                    )
                    result = result.cut(cutout)

    # --- Alignment pin holes in both halves ---
    pin_hole_radius = 1.75 / 2.0  # mm (sized for 1.75mm filament)
    pin_hole_depth = 4.0
    bolt_circle_radius = (valley_radius + shaft_radius) / 2.0
    pin_position_radius = bolt_circle_radius

    for angle in [0, 180]:
        x = pin_position_radius * math.cos(math.radians(angle))
        y = pin_position_radius * math.sin(math.radians(angle))
        hole = (
            cq.Workplane("XY")
            .transformed(offset=(x, y, 0))
            .circle(pin_hole_radius)
            .extrude(pin_hole_depth)
        )
        result = result.cut(hole)

    # --- Recessed hex nut pocket on outer face of threaded side ---
    if nut_width_af > 0 and nut_thickness > 0 and not is_large_side:
        af_clearance = nut_width_af + 0.3
        depth = nut_thickness + 0.4

        if depth > half_width - 2.0:
            raise ValueError(
                f"Nut thickness ({nut_thickness}mm) too large for "
                f"threaded side width ({half_width}mm) - need at least 2mm floor"
            )

        nut_pocket = (
            cq.Workplane("XY")
            .transformed(offset=(0, 0, half_width))
            .polygon(6, af_clearance / math.cos(math.radians(30)))
            .extrude(-depth)
        )
        result = result.cut(nut_pocket)

    # --- Bolt circle for joining halves with heat-set inserts ---
    if bolt_count > 0 and heat_insert_od > 0:
        # Offset from alignment pins (at 0 and 180 deg)
        bolt_start_angle = 360.0 / bolt_count / 2.0

        for i in range(bolt_count):
            angle = bolt_start_angle + (360.0 / bolt_count) * i
            angle_rad = math.radians(angle)
            x = bolt_circle_radius * math.cos(angle_rad)
            y = bolt_circle_radius * math.sin(angle_rad)

            if is_large_side:
                # Heat-set insert hole: blind hole from the split face
                insert_hole = (
                    cq.Workplane("XY")
                    .transformed(offset=(x, y, 0))
                    .circle(heat_insert_od / 2.0)
                    .extrude(heat_insert_length + 0.5)
                )
                result = result.cut(insert_hole)
            else:
                # Through-hole for bolt shank
                clearance_hole = (
                    cq.Workplane("XY")
                    .transformed(offset=(x, y, 0))
                    .circle(bolt_clearance_dia / 2.0)
                    .extrude(half_width)
                )
                result = result.cut(clearance_hole)

                # Counterbore on outer face for bolt head
                if bolt_head_dia > 0 and bolt_head_depth > 0:
                    counterbore = (
                        cq.Workplane("XY")
                        .transformed(offset=(x, y, half_width))
                        .circle(bolt_head_dia / 2.0)
                        .extrude(-bolt_head_depth)
                    )
                    result = result.cut(counterbore)

    # --- Orient for printing: flip so outer face (z=half_width) is at z=0 ---
    result = result.mirror("XY").translate((0, 0, half_width))

    # --- Chamfer build-plate edges to mask elephant's foot ---
    # Only chamfer closed circular edges (outer perimeter, bore, counterbores).
    # The cutout-window arcs meet at sharp corners that OCC refuses to chamfer,
    # and the hex nut pocket wants a sharp socket, not a lead-in.
    if build_plate_chamfer > 0:
        circle_edges = []
        for edge in result.faces("<Z").edges().vals():
            if edge.geomType() != "CIRCLE":
                continue
            s, e = edge.startPoint(), edge.endPoint()
            if abs(s.x - e.x) + abs(s.y - e.y) + abs(s.z - e.z) < 1e-6:
                circle_edges.append(edge)
        if circle_edges:
            result = result.newObject(circle_edges).chamfer(build_plate_chamfer)

    return result


def generate_pulley(
    large_side_width: float = 9.0,
    threaded_side_width: float = 10.9,
    shaft_dia_large: float = 8.0,
    shaft_dia_threaded: float = 6.0,
    valley_diameter: float = 125.0,
    flange_diameter: float = 150.0,
    flange_thickness: float = 2.0,
    nut_width_af: float = 10.0,
    nut_thickness: float = 5.0,
    bolt_count: int = 4,
    heat_insert_od: float = 5.0,
    heat_insert_length: float = 4.0,
    bolt_clearance_dia: float = 3.4,
    bolt_head_dia: float = 5.5,
    bolt_head_depth: float = 3.2,
    flange_cutout_count: int = 6,
    build_plate_chamfer: float = 1.0,
    bore_compensation: float = 0.2,
    output_dir: str = "output",
):
    """Generate both halves of the pulley and export as STL files."""

    os.makedirs(output_dir, exist_ok=True)

    print(f"Generating drive pulley with parameters:")
    print(f"  Large side width:      {large_side_width} mm")
    print(f"  Threaded side width:   {threaded_side_width} mm")
    print(f"  Total pulley width:    {large_side_width + threaded_side_width} mm")
    print(f"  Shaft dia (large):     {shaft_dia_large} mm (target printed)")
    print(f"  Shaft dia (threaded):  {shaft_dia_threaded} mm (target printed)")
    print(f"  Bore compensation:     +{bore_compensation} mm (added to model)")
    print(f"  Valley diameter:       {valley_diameter} mm")
    print(f"  Flange diameter:       {flange_diameter} mm")
    print(f"  Flange thickness:      {flange_thickness} mm")
    if nut_width_af > 0 and nut_thickness > 0:
        print(f"  Nut recess (AF):       {nut_width_af} mm")
        print(f"  Nut recess (thick):    {nut_thickness} mm")
    else:
        print(f"  Nut recess:            disabled")
    if bolt_count > 0:
        print(f"  Bolt count:            {bolt_count}")
        print(f"  Heat insert OD:        {heat_insert_od} mm")
        print(f"  Heat insert length:    {heat_insert_length} mm")
        print(f"  Bolt clearance dia:    {bolt_clearance_dia} mm")
        print(f"  Bolt head dia:         {bolt_head_dia} mm")
        print(f"  Bolt head depth:       {bolt_head_depth} mm")
    else:
        print(f"  Bolt circle:           disabled")
    print(f"  Flange cutouts:        {flange_cutout_count} (0 = solid flange)")
    if build_plate_chamfer > 0:
        print(f"  Build-plate chamfer:   {build_plate_chamfer} mm (45 deg)")
    else:
        print(f"  Build-plate chamfer:   disabled")
    print()

    # Validation
    if valley_diameter >= flange_diameter:
        raise ValueError("Valley diameter must be less than flange diameter")
    if shaft_dia_large <= 0 or shaft_dia_threaded <= 0:
        raise ValueError("Shaft diameters must be positive")
    if large_side_width <= 0 or threaded_side_width <= 0:
        raise ValueError("Half widths must be positive")
    if valley_diameter <= max(shaft_dia_large, shaft_dia_threaded) + 4:
        raise ValueError("Valley diameter must be larger than shaft diameter + 4mm wall")

    common_args = dict(
        large_side_width=large_side_width,
        threaded_side_width=threaded_side_width,
        shaft_dia_large=shaft_dia_large,
        shaft_dia_threaded=shaft_dia_threaded,
        valley_diameter=valley_diameter,
        flange_diameter=flange_diameter,
        flange_thickness=flange_thickness,
        nut_width_af=nut_width_af,
        nut_thickness=nut_thickness,
        bolt_count=bolt_count,
        heat_insert_od=heat_insert_od,
        heat_insert_length=heat_insert_length,
        bolt_clearance_dia=bolt_clearance_dia,
        bolt_head_dia=bolt_head_dia,
        bolt_head_depth=bolt_head_depth,
        flange_cutout_count=flange_cutout_count,
        build_plate_chamfer=build_plate_chamfer,
        bore_compensation=bore_compensation,
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
    assembled = large_half
    threaded_flipped = threaded_half.mirror("XY")
    assembled = assembled.union(threaded_flipped)
    assembled_step = os.path.join(output_dir, "pulley_assembled.step")
    cq.exporters.export(assembled, assembled_step, cq.exporters.ExportTypes.STEP)
    print(f"  Saved: {assembled_step}")

    # Recommend bolt length
    if bolt_count > 0:
        grip_length = threaded_side_width - bolt_head_depth + heat_insert_length
        common_lengths = [6, 8, 10, 12, 16, 20, 25, 30]
        recommended = next((l for l in common_lengths if l >= grip_length), common_lengths[-1])
        print(f"\n  Bolt grip needed:      {grip_length:.1f} mm")
        print(f"  Recommended M3 SHCS:   {recommended} mm")

    print("\nDone! STLs are oriented with the outer face down on the build plate.")
    print("Assemble with alignment pins and M3 bolts through the bolt circle.")


def main():
    parser = argparse.ArgumentParser(
        description="Generate a parametric drive pulley/spool as two printable STL halves.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--large-side-width", type=float, default=9.0,
                        help="Width of the large/keyed side half (mm)")
    parser.add_argument("--threaded-side-width", type=float, default=10.9,
                        help="Width of the threaded/nut side half (mm)")
    parser.add_argument("--shaft-dia-large", type=float, default=8.0,
                        help="Target printed bore diameter on the large/keyed side (mm)")
    parser.add_argument("--shaft-dia-threaded", type=float, default=6.0,
                        help="Target printed bore diameter on the threaded/small side (mm)")
    parser.add_argument("--valley-diameter", type=float, default=125.0,
                        help="Diameter at the valley floor (mm)")
    parser.add_argument("--flange-diameter", type=float, default=150.0,
                        help="Outer diameter at the pulley flanges (mm)")
    parser.add_argument("--flange-thickness", type=float, default=2.0,
                        help="Thickness of the flange walls (mm)")
    parser.add_argument("--nut-width-af", type=float, default=10.0,
                        help="Nut width across-flats for hex recess, 0 to disable (mm)")
    parser.add_argument("--nut-thickness", type=float, default=5.0,
                        help="Nut thickness for hex recess, 0 to disable (mm)")
    parser.add_argument("--bolt-count", type=int, default=4,
                        help="Number of bolts in bolt circle, 0 to disable")
    parser.add_argument("--heat-insert-od", type=float, default=5.0,
                        help="Heat-set insert outer diameter (mm)")
    parser.add_argument("--heat-insert-length", type=float, default=4.0,
                        help="Heat-set insert length (mm)")
    parser.add_argument("--bolt-clearance-dia", type=float, default=3.4,
                        help="Bolt shank clearance hole diameter (mm)")
    parser.add_argument("--bolt-head-dia", type=float, default=5.5,
                        help="Socket head cap screw head diameter for counterbore (mm)")
    parser.add_argument("--bolt-head-depth", type=float, default=3.2,
                        help="Counterbore depth for bolt head (mm)")
    parser.add_argument("--flange-cutout-count", type=int, default=6,
                        help="Number of cutout windows in the flange (0 = solid)")
    parser.add_argument("--build-plate-chamfer", type=float, default=1.0,
                        help="45-deg chamfer size on build-plate edges, 0 to disable (mm)")
    parser.add_argument("--bore-compensation", type=float, default=0.2,
                        help="Amount added to bore diameter to offset print shrinkage, 0 to disable (mm)")
    parser.add_argument("--output-dir", type=str, default="output",
                        help="Output directory for STL files")

    args = parser.parse_args()

    generate_pulley(
        large_side_width=args.large_side_width,
        threaded_side_width=args.threaded_side_width,
        shaft_dia_large=args.shaft_dia_large,
        shaft_dia_threaded=args.shaft_dia_threaded,
        valley_diameter=args.valley_diameter,
        flange_diameter=args.flange_diameter,
        flange_thickness=args.flange_thickness,
        nut_width_af=args.nut_width_af,
        nut_thickness=args.nut_thickness,
        bolt_count=args.bolt_count,
        heat_insert_od=args.heat_insert_od,
        heat_insert_length=args.heat_insert_length,
        bolt_clearance_dia=args.bolt_clearance_dia,
        bolt_head_dia=args.bolt_head_dia,
        bolt_head_depth=args.bolt_head_depth,
        flange_cutout_count=args.flange_cutout_count,
        build_plate_chamfer=args.build_plate_chamfer,
        bore_compensation=args.bore_compensation,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
