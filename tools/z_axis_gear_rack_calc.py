#!/usr/bin/env python3
"""Z-axis rack-and-pinion design calculator.

All dimensions are millimetres unless noted otherwise.
The model is a standard metric involute pinion (pressure angle 20 deg)
meshing with a straight-toothed rack. It is intended for a first-pass
design check, not for replacing a CAD mesh/print test.

Examples
--------
python tools/z_axis_gear_rack_calc.py
python tools/z_axis_gear_rack_calc.py --module 1.5 --teeth 30 --angle 170
"""

from __future__ import annotations

import argparse
import math

PRESSURE_ANGLE_DEG = 20.0
GRAVITY = 9.80665
NM_TO_KGCM = 10.197162


def stroke_mm(module: float, teeth: int, angle_deg: float) -> float:
    """Rack travel for a pinion rotation angle."""
    pitch_diameter = module * teeth
    return angle_deg / 360.0 * math.pi * pitch_diameter


def design(
    module: float = 1.5,
    teeth: int = 30,
    angle_deg: float = 170.0,
    moving_mass_kg: float = 0.5,
    guide_friction_n: float = 3.0,
    efficiency: float = 0.85,
    face_width_mm: float = 10.0,
    sigma_allow_mpa: float = 25.0,
    lewis_y: float = 0.32,
) -> dict:
    """Return the main geometry, torque and strength numbers."""
    pitch_diameter = module * teeth
    outside_diameter = module * (teeth + 2)
    root_diameter = module * (teeth - 2.5)
    circular_pitch = math.pi * module
    pitch_radius = pitch_diameter / 2.0

    full_stroke = stroke_mm(module, teeth, 180.0)
    usable_stroke = stroke_mm(module, teeth, angle_deg)

    # Centre distance from the pinion axis to the rack reference lines.
    # Rack pitch line is d/2 below the pinion axis; rack tip is module above
    # the pitch line (towards the pinion), rack root is 1.25m below it.
    centre_to_pitch_line = pitch_radius
    centre_to_tip = pitch_radius - module
    centre_to_root = pitch_radius + 1.25 * module

    tangential_load_n = moving_mass_kg * GRAVITY + guide_friction_n
    torque_nm = tangential_load_n * (pitch_radius / 1000.0) / efficiency

    # Lewis bending estimate for a printed spur gear.
    # Wt_allow = sigma * face_width * module * Lewis form factor
    wt_allow_n = (
        sigma_allow_mpa
        * (face_width_mm / 1000.0)
        * (module / 1000.0)
        * lewis_y
        * 1_000_000.0
    )
    tooth_safety_factor = wt_allow_n / tangential_load_n if tangential_load_n else 0.0

    # The rack tooth strip must cover the full stroke plus at least three
    # pitches of engagement margin at each end.
    teeth_in_mesh = math.ceil(full_stroke / circular_pitch)
    rack_teeth = teeth_in_mesh + 6
    rack_strip = rack_teeth * circular_pitch

    # Backlash increase caused by moving the pinion away from the rack.
    # For a 20 deg pressure angle, j = 2 * delta_a * tan(alpha).
    centre_offset_for_015_backlash = 0.15 / (
        2.0 * math.tan(math.radians(PRESSURE_ANGLE_DEG))
    )

    # Example pulse mappings. They are NOT valid until the servo is measured.
    ppm_500_2500 = full_stroke / 2000.0  # mm per us if 500..2500 us = 180 deg
    ppm_1000_2000 = full_stroke / 1000.0  # mm per us if 1000..2000 us = 180 deg

    return {
        "module": module,
        "teeth": teeth,
        "angle_deg": angle_deg,
        "pitch_diameter": pitch_diameter,
        "outside_diameter": outside_diameter,
        "root_diameter": root_diameter,
        "circular_pitch": circular_pitch,
        "full_stroke": full_stroke,
        "usable_stroke": usable_stroke,
        "centre_to_pitch_line": centre_to_pitch_line,
        "centre_to_tip": centre_to_tip,
        "centre_to_root": centre_to_root,
        "tangential_load_n": tangential_load_n,
        "torque_nm": torque_nm,
        "torque_kgcm": torque_nm * NM_TO_KGCM,
        "torque_with_sf3_kgcm": torque_nm * NM_TO_KGCM * 3.0,
        "wt_allow_n": wt_allow_n,
        "tooth_safety_factor": tooth_safety_factor,
        "rack_teeth": rack_teeth,
        "rack_strip": rack_strip,
        "centre_offset_for_015_backlash": centre_offset_for_015_backlash,
        "mm_per_us_500_2500": ppm_500_2500,
        "mm_per_us_1000_2000": ppm_1000_2000,
    }


def print_result(r: dict) -> None:
    print("== Z-axis rack and pinion ==")
    print(f"module m                : {r['module']:.2f} mm")
    print(f"pinion teeth z          : {r['teeth']}")
    print(f"pitch diameter d        : {r['pitch_diameter']:.2f} mm")
    print(f"outside diameter da     : {r['outside_diameter']:.2f} mm")
    print(f"root diameter df        : {r['root_diameter']:.2f} mm")
    print(f"circular pitch p        : {r['circular_pitch']:.3f} mm")
    print(f"stroke at 180 deg       : {r['full_stroke']:.2f} mm")
    print(f"stroke at {r['angle_deg']:.0f} deg       : {r['usable_stroke']:.2f} mm")
    print(f"centre to rack pitch    : {r['centre_to_pitch_line']:.2f} mm")
    print(f"centre to rack tip      : {r['centre_to_tip']:.2f} mm")
    print(f"centre to rack root     : {r['centre_to_root']:.2f} mm")
    print(f"tangential load         : {r['tangential_load_n']:.2f} N")
    print(f"servo torque            : {r['torque_nm']:.3f} N.m = {r['torque_kgcm']:.2f} kg.cm")
    print(f"torque with SF=3        : {r['torque_with_sf3_kgcm']:.2f} kg.cm")
    print(f"Lewis allowable load    : {r['wt_allow_n']:.1f} N")
    print(f"tooth safety factor     : {r['tooth_safety_factor']:.1f}")
    print(f"rack teeth / strip      : {r['rack_teeth']} T / {r['rack_strip']:.1f} mm")
    print(f"centre offset for 0.15  : {r['centre_offset_for_015_backlash']:.3f} mm")
    print(f"mm/us if 500..2500 us   : {r['mm_per_us_500_2500']:.5f}")
    print(f"mm/us if 1000..2000 us  : {r['mm_per_us_1000_2000']:.5f}")


def print_candidates() -> None:
    print("\n== pinion candidates (m=1.5) ==")
    print(" z |   d   |  da   | 160 deg | 170 deg | 180 deg")
    print("---+-------+-------+---------+---------+--------")
    for z in range(26, 39, 2):
        print(
            f"{z:2d} | {1.5 * z:5.1f} | {1.5 * (z + 2):5.1f} | "
            f"{stroke_mm(1.5, z, 160):7.2f} | {stroke_mm(1.5, z, 170):7.2f} | "
            f"{stroke_mm(1.5, z, 180):7.2f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", type=float, default=1.5)
    parser.add_argument("--teeth", type=int, default=30)
    parser.add_argument("--angle", type=float, default=170.0)
    parser.add_argument("--mass", type=float, default=0.5, help="moving mass in kg")
    parser.add_argument("--friction", type=float, default=3.0, help="guide friction in N")
    parser.add_argument("--face", type=float, default=10.0, help="face width in mm")
    parser.add_argument("--sigma", type=float, default=25.0, help="allowable bending stress in MPa")
    args = parser.parse_args()

    result = design(
        module=args.module,
        teeth=args.teeth,
        angle_deg=args.angle,
        moving_mass_kg=args.mass,
        guide_friction_n=args.friction,
        face_width_mm=args.face,
        sigma_allow_mpa=args.sigma,
    )
    print_result(result)
    print_candidates()


if __name__ == "__main__":
    main()
