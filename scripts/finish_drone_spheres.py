"""Finish foam's sphere model of PHANTOM's drone into ``drone_spherized.urdf``.

Usage: finish_drone_spheres.py FOAM_URDF SOURCE_URDF OUTPUT_URDF

foam (``scripts/spherize_robot.sh drone``) spherizes the body box of
``drone.urdf`` (SOURCE_URDF) into FOAM_URDF.  Its spheres cover foam's sample
points on the box surface, not the whole box: points between the samples lie
up to a few millimetres outside every sphere.  This script

1. grows every foam sphere by the same smallest amount, rounded up to 0.1 mm,
   that puts every point of the box inside a sphere.  The amount is the
   largest distance of a box point outside the spheres: the worst points of a
   dense lattice of the box, refined by local search;
2. adds one sphere for the camera, fixed to the body at the gimbal pivot (the
   origin of ``camera_pan``, through which the tilt axis passes too).  Its
   radius is the distance from the pivot to the camera box's farthest corner,
   rounded up to 0.1 mm, so the sphere holds the camera at every pan and tilt
   and the collision model does not depend on the gimbal angles;
3. writes OUTPUT_URDF: the joints of SOURCE_URDF, these spheres on
   ``base_link`` and no visual geometry.
"""

import math
import sys
import textwrap
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import minimize

LATTICE = (93, 93, 25)  # lattice points along x, y and z of the box
REFINED = 60  # worst lattice points refined by local search
STEP = 1e-4  # rounding (m) of the growth and of the camera sphere's radius


def floats(text):
    return np.array([float(value) for value in text.split()])


def origin_xyz(element):
    origin = element.find("origin")
    if origin is not None and floats(origin.get("rpy", "0 0 0")).any():
        raise ValueError("finish_drone_spheres expects origins without rotation")
    return np.zeros(3) if origin is None else floats(origin.get("xyz", "0 0 0"))


def link(root, name):
    return root.find(f"link[@name='{name}']")


def joint(root, name):
    return root.find(f"joint[@name='{name}']")


def foam_spheres(path):
    """``(n, 4)`` centres and radii of the spheres foam put on base_link."""
    spheres = [
        [*origin_xyz(collision), float(collision.find("geometry/sphere").get("radius"))]
        for collision in link(ET.parse(path).getroot(), "base_link").findall(
            "collision"
        )
    ]
    if not spheres:
        raise ValueError(f"{path} has no spheres on base_link")
    return np.array(spheres)


def outside(points, spheres):
    """Distance of each point outside the nearest sphere (negative inside)."""
    gaps = (
        np.linalg.norm(points[..., None, :] - spheres[:, :3], axis=-1) - spheres[:, 3]
    )
    return gaps.min(-1)


def largest_gap(spheres, half):
    """Largest distance of a point of the box ``[-half, half]`` outside every sphere."""
    axes = [np.linspace(-h, h, n) for h, n in zip(half, LATTICE)]
    points = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    gaps = outside(points, spheres)
    largest = float(gaps.max())
    bounds = list(zip(-half, half))
    for start in points[np.argsort(gaps)[-REFINED:]]:
        result = minimize(
            lambda p: -outside(p, spheres), start, method="L-BFGS-B", bounds=bounds
        )
        largest = max(largest, float(-result.fun))
    return largest


def round_up(value):
    return math.ceil(value / STEP - 1e-9) * STEP


def camera_sphere(root):
    """Centre and radius of the sphere at the gimbal pivot that holds the camera box."""
    if origin_xyz(joint(root, "camera_tilt")).any():
        raise ValueError("The tilt axis must pass through the pan joint origin")
    pivot = origin_xyz(joint(root, "camera_pan"))
    visual = link(root, "camera").find("visual")
    center = origin_xyz(visual)
    half = floats(visual.find("geometry/box").get("size")) / 2
    corners = np.array(np.meshgrid(*zip(center - half, center + half))).reshape(3, -1).T
    return pivot, round_up(float(np.linalg.norm(corners, axis=1).max()))


def sphere_element(center, radius):
    collision = ET.Element("collision")
    ET.SubElement(
        collision, "origin", xyz=" ".join(f"{v:.6f}" for v in center + 0.0), rpy="0 0 0"
    )
    geometry = ET.SubElement(collision, "geometry")
    # Radii are rounded up, never down, so the cover stays whole.
    ET.SubElement(
        geometry, "sphere", radius=f"{math.ceil(radius * 1e6 - 1e-6) / 1e6:.6f}"
    )
    return collision


def main(foam_urdf, source_urdf, output_urdf):
    root = ET.parse(source_urdf).getroot()  # comments are dropped
    body = link(root, "base_link")
    box = body.find("collision/geometry/box")
    if box is None or origin_xyz(body.find("collision")).any():
        raise ValueError("drone.urdf must give base_link a centred collision box")
    half = floats(box.get("size")) / 2
    spheres = foam_spheres(foam_urdf)
    gap = largest_gap(spheres, half)
    growth = round_up(max(gap, 0.0))
    spheres[:, 3] += growth
    pivot, camera_radius = camera_sphere(root)

    for element in root.iter("link"):
        for child in element.findall("visual") + element.findall("collision"):
            element.remove(child)
    for sphere in spheres:
        body.append(sphere_element(sphere[:3], sphere[3]))
    body.append(sphere_element(pivot, camera_radius))
    note = (
        "The collision model of drone.urdf, the source of the VAMP model (cricket, "
        "scripts/generate_fk.sh drone), generated by scripts/spherize_robot.sh drone: "
        f"foam's medial fit of the body box ({len(spheres)} spheres), each grown by "
        f"{growth * 1000:.1f} mm, the largest distance of a box point outside foam's "
        f"spheres ({gap * 1000:.2f} mm) rounded up; then one sphere at the gimbal pivot "
        f"that holds the camera at every pan and tilt ({camera_radius * 1000:.1f} mm). "
        "Every sphere is on base_link, so the gimbal angles do not change whether the "
        "drone collides."
    )
    text = textwrap.fill(note, 72, initial_indent=" " * 7, subsequent_indent=" " * 7)
    root.insert(0, ET.Comment(" " + text.lstrip() + " "))
    for element in root.iter():  # the emptied links keep no whitespace
        if element.text is not None and not element.text.strip():
            element.text = None
    ET.indent(root, space="  ")
    with open(output_urdf, "w") as stream:
        stream.write(
            '<?xml version="1.0" ?>\n' + ET.tostring(root, encoding="unicode") + "\n"
        )
    print(
        f"{len(spheres)} foam spheres, largest gap {gap * 1000:.3f} mm, growth "
        f"{growth * 1000:.1f} mm; camera sphere {camera_radius * 1000:.1f} mm at {pivot}"
    )


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
