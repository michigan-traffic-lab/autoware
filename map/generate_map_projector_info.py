#!/usr/bin/env python3
"""
Generate map_projector_info.yaml from a lanelet2 OSM file or SUMO network.

Usage:
    # From OSM file (uses first node as origin):
    python generate_map_projector_info.py <osm_file>

    # From SUMO network file (uses netOffset for precise alignment):
    python generate_map_projector_info.py --sumo <net.xml>

    # Both (recommended - verifies alignment):
    python generate_map_projector_info.py <osm_file> --sumo <net.xml>

Examples:
    python generate_map_projector_info.py gandy2herbana_depth2_lanelet2.osm
    python generate_map_projector_info.py --sumo gandy2herbana_depth2.net.xml
    python generate_map_projector_info.py my_map.osm --sumo my_map.net.xml
"""

import argparse
import os
import re
import sys
from pathlib import Path

try:
    from pyproj import Proj
    HAS_PYPROJ = True
except ImportError:
    HAS_PYPROJ = False


def extract_origin_from_osm(osm_path: str) -> tuple[float, float]:
    """
    Extract the map origin (first node's lat/lon) from a lanelet2 OSM file.

    Args:
        osm_path: Path to the lanelet2 OSM file

    Returns:
        Tuple of (latitude, longitude)
    """
    # Pattern to match node with lat/lon attributes
    node_pattern = re.compile(
        r'<node[^>]+lat="([^"]+)"[^>]+lon="([^"]+)"'
    )

    with open(osm_path, 'r') as f:
        for line in f:
            match = node_pattern.search(line)
            if match:
                lat = float(match.group(1))
                lon = float(match.group(2))
                return lat, lon

    raise ValueError(f"No node with lat/lon found in {osm_path}")


def extract_origin_from_sumo(net_path: str) -> tuple[float, float]:
    """
    Extract the map origin from SUMO network file's netOffset.

    The netOffset in SUMO is the negated UTM coordinates of the origin.
    We convert it back to lat/lon.

    Args:
        net_path: Path to the SUMO .net.xml file

    Returns:
        Tuple of (latitude, longitude)
    """
    if not HAS_PYPROJ:
        raise ImportError("pyproj is required for SUMO netOffset conversion. Install with: pip install pyproj")

    # Pattern to match location element with netOffset and projParameter
    location_pattern = re.compile(
        r'<location[^>]+netOffset="([^"]+)"[^>]+projParameter="([^"]+)"'
    )

    with open(net_path, 'r') as f:
        content = f.read()

    match = location_pattern.search(content)
    if not match:
        raise ValueError(f"No location element with netOffset found in {net_path}")

    net_offset = match.group(1)
    proj_param = match.group(2)

    # Parse netOffset (format: "x,y")
    offset_x, offset_y = map(float, net_offset.split(','))

    # Parse UTM zone from projParameter
    zone_match = re.search(r'\+zone=(\d+)', proj_param)
    if not zone_match:
        raise ValueError(f"Could not parse UTM zone from projParameter: {proj_param}")

    zone = int(zone_match.group(1))

    # The netOffset is negated UTM coordinates, so negate again to get real UTM
    utm_x = -offset_x
    utm_y = -offset_y

    # Convert UTM to lat/lon
    proj = Proj(proj='utm', zone=zone, ellps='WGS84', datum='WGS84')
    lon, lat = proj(utm_x, utm_y, inverse=True)

    return lat, lon


def generate_yaml(lat: float, lon: float, altitude: float = 0.0) -> str:
    """Generate the map_projector_info.yaml content."""
    return f"""projector_type: LocalCartesianUTM
vertical_datum: WGS84
map_origin:
  latitude: {lat}
  longitude: {lon}
  altitude: {altitude}
"""


def main():
    parser = argparse.ArgumentParser(
        description="Generate map_projector_info.yaml from lanelet2 OSM or SUMO network file"
    )
    parser.add_argument(
        "osm_file",
        nargs="?",
        help="Path to the lanelet2 OSM file (optional if --sumo is provided)"
    )
    parser.add_argument(
        "--sumo", "-s",
        help="Path to SUMO .net.xml file (recommended for precise alignment)"
    )
    parser.add_argument(
        "--output", "-o",
        help="Output directory (default: same directory as input file)"
    )
    parser.add_argument(
        "--altitude", "-a",
        type=float,
        default=0.0,
        help="Altitude in meters (default: 0.0)"
    )
    parser.add_argument(
        "--dry-run", "-n",
        action="store_true",
        help="Print the yaml content without writing to file"
    )
    parser.add_argument(
        "--force", "-f",
        action="store_true",
        help="Overwrite existing file without prompting"
    )

    args = parser.parse_args()

    if not args.osm_file and not args.sumo:
        parser.error("Either osm_file or --sumo must be provided")

    lat, lon = None, None
    osm_lat, osm_lon = None, None
    sumo_lat, sumo_lon = None, None
    input_path = None

    # Extract from OSM if provided
    if args.osm_file:
        osm_path = Path(args.osm_file)
        if not osm_path.exists():
            print(f"Error: OSM file not found: {osm_path}", file=sys.stderr)
            sys.exit(1)
        input_path = osm_path

        try:
            osm_lat, osm_lon = extract_origin_from_osm(str(osm_path))
            print(f"Extracted origin from OSM ({osm_path.name}):")
            print(f"  latitude:  {osm_lat}")
            print(f"  longitude: {osm_lon}")
        except ValueError as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)

    # Extract from SUMO if provided
    if args.sumo:
        sumo_path = Path(args.sumo)
        if not sumo_path.exists():
            print(f"Error: SUMO file not found: {sumo_path}", file=sys.stderr)
            sys.exit(1)
        if input_path is None:
            input_path = sumo_path

        try:
            sumo_lat, sumo_lon = extract_origin_from_sumo(str(sumo_path))
            print(f"\nExtracted origin from SUMO netOffset ({sumo_path.name}):")
            print(f"  latitude:  {sumo_lat}")
            print(f"  longitude: {sumo_lon}")
        except (ValueError, ImportError) as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)

    # If both provided, check alignment and prefer SUMO
    if osm_lat is not None and sumo_lat is not None:
        # Calculate distance between origins
        from math import radians, cos, sin, sqrt, atan2
        R = 6371000  # Earth radius in meters

        lat1, lon1 = radians(osm_lat), radians(osm_lon)
        lat2, lon2 = radians(sumo_lat), radians(sumo_lon)

        dlat = lat2 - lat1
        dlon = lon2 - lon1

        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        distance = R * c

        print(f"\nAlignment check:")
        print(f"  Distance between OSM and SUMO origins: {distance:.2f} m")

        if distance > 10:
            print(f"  WARNING: Origins differ by {distance:.1f}m!")
            print(f"  Using SUMO netOffset for precise co-simulation alignment.")
        else:
            print(f"  OK: Origins are well-aligned.")

        # Prefer SUMO origin for co-simulation
        lat, lon = sumo_lat, sumo_lon
        print(f"\nUsing SUMO-derived origin for output.")
    elif sumo_lat is not None:
        lat, lon = sumo_lat, sumo_lon
    else:
        lat, lon = osm_lat, osm_lon

    # Generate yaml content
    yaml_content = generate_yaml(lat, lon, args.altitude)

    if args.dry_run:
        print("\nGenerated map_projector_info.yaml:")
        print("-" * 40)
        print(yaml_content)
        return

    # Determine output path
    if args.output:
        output_dir = Path(args.output)
    else:
        output_dir = input_path.parent

    output_path = output_dir / "map_projector_info.yaml"

    # Check if file exists
    if output_path.exists() and not args.force:
        print(f"\nWarning: {output_path} already exists.")
        response = input("Overwrite? [y/N] ").strip().lower()
        if response != 'y':
            print("Aborted.")
            sys.exit(0)

    # Write the file
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(yaml_content)

    print(f"\nGenerated: {output_path}")


if __name__ == "__main__":
    main()
