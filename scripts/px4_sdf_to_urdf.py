"""Convert PX4's ``x500_gimbal`` Gazebo model (SDF) to the planning URDF.

    python scripts/px4_sdf_to_urdf.py <PX4-gazebo-models>/models <output dir>

``x500_gimbal`` merges ``x500`` (which merges ``x500_base``) with the
``gimbal`` model, attached to ``base_link``.  The conversion keeps every
link, joint, pose, mesh and inertial of those SDFs and changes only what a
planning model needs:

* Four virtual joints ``base_x_joint`` / ``base_y_joint`` / ``base_z_joint``
  (prismatic) and ``base_yaw_joint`` (revolute) carry ``base_link`` from
  ``world``: the flight controller holds roll and pitch.
* The rotor joints and the gimbal roll joint become fixed; the gimbal pan
  joint, unlimited in the SDF, gets the range of PX4's ``MNT_RANGE_YAW``
  (360 deg) and the virtual joints the planner's default workspace.
* URDF puts a child link's frame at its parent joint, so link contents are
  re-expressed in that frame (the SDF gives link poses in the model frame
  and joint poses in the child link frame; axes are in the joint frame).
* Meshes are written as binary STL in their own frame, in metres (the SDF
  ``<scale>`` is applied to the vertices: foam would apply a collision
  origin before the scale).  COLLADA meshes are flattened with their node
  transforms; a mesh above the repository's 1 MiB file limit is decimated
  (quadric edge collapse) to fit.  Textures and the four flat decal planes
  on the frame are dropped, and faces wound inwards are flipped.
* Collision geometry is the visual meshes (foam spherizes them), except on
  the rotors: a spinning rotor (propeller + motor bell) sweeps a solid of
  revolution, so its collision is the cylinder that bounds it about the
  rotor axis.
* A fixed ``camera_sensor_link`` marks the SDF camera sensor pose (x along
  the optical axis, Gazebo's camera convention).
"""

import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pymeshlab
import trimesh
from trimesh.transformations import euler_from_matrix, euler_matrix

MAX_FILE_BYTES = 1024 * 1024  # pre-commit check-added-large-files limit
STL_BYTES_PER_FACE = 50  # binary STL: 84-byte header, 50 bytes per face

PAN_RANGE = math.radians(360.0) / 2  # PX4 MNT_RANGE_YAW = 360 deg
# Virtual base joints: (name, type, axis, lower, upper).  The workspace box
# is the default planning range; MotionPlanner.set_joint_limits replaces it.
VIRTUAL_JOINTS = [
    ("base_x_joint", "prismatic", "1 0 0", -10.0, 10.0),
    ("base_y_joint", "prismatic", "0 1 0", -10.0, 10.0),
    ("base_z_joint", "prismatic", "0 0 1", 0.0, 10.0),
    ("base_yaw_joint", "revolute", "0 0 1", -math.pi, math.pi),
]
ROTORS = {"rotor_0", "rotor_1", "rotor_2", "rotor_3"}  # spin about their z axis
FIXED_JOINTS = {f"{rotor}_joint" for rotor in ROTORS} | {
    "cgo3_horizontal_arm_joint",  # gimbal roll, held level
}
JOINT_LIMITS = {"cgo3_vertical_arm_joint": (-PAN_RANGE, PAN_RANGE)}
COLORS = {
    "base_link": "0.15 0.15 0.15 1",
    "rotor_0": "0.3 0.3 0.3 1",
    "rotor_1": "0.3 0.3 0.3 1",
    "rotor_2": "0.3 0.3 0.3 1",
    "rotor_3": "0.3 0.3 0.3 1",
    "cgo3_mount_link": "0.175 0.175 0.175 1",
    "cgo3_vertical_arm_link": "0.175 0.175 0.175 1",
    "cgo3_horizontal_arm_link": "0.175 0.175 0.175 1",
    "camera_link": "0.175 0.175 0.175 1",
}


def pose(element) -> np.ndarray:
    """4x4 transform of an element's ``<pose>`` (x y z roll pitch yaw)."""
    x, y, z, roll, pitch, yaw = map(
        float, element.findtext("pose", "0 0 0 0 0 0").split()
    )
    T = euler_matrix(roll, pitch, yaw, "sxyz")
    T[:3, 3] = x, y, z
    return T


def load_model(
    models: Path, name: str, placement: np.ndarray, links: dict, joints: list
):
    """Collect links (model-frame pose, element) and joints of a model and
    its merged includes.  ``placement`` is the model frame in the top model.
    """
    model = ET.parse(models / name / "model.sdf").getroot().find("model")
    for include in model.findall("include"):
        child = include.findtext("uri").removeprefix("model://")
        child_model = ET.parse(models / child / "model.sdf").getroot().find("model")
        # The include's <pose> replaces the included model's own <pose>.
        frame = pose(include) if include.find("pose") is not None else pose(child_model)
        load_model(models, child, placement @ frame, links, joints)
    for link in model.findall("link"):
        links[link.get("name")] = (placement @ pose(link), link)
    joints.extend(model.findall("joint"))


def load_mesh(path: Path, scale: np.ndarray) -> trimesh.Trimesh:
    """Mesh in its own frame, in metres.  COLLADA scenes are flattened with
    their node transforms; faces wound inwards (a negative enclosed volume:
    three gimbal meshes and the propellers) are flipped outwards."""
    loaded = trimesh.load(path, process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.to_geometry()
    mesh = trimesh.Trimesh(loaded.vertices * scale, loaded.faces, process=False)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def write_stl(mesh: trimesh.Trimesh, path: Path) -> None:
    """Binary STL, decimated to the file-size limit when it is above it."""
    faces = (MAX_FILE_BYTES - 84) // STL_BYTES_PER_FACE
    if len(mesh.faces) > faces:
        meshset = pymeshlab.MeshSet()
        meshset.add_mesh(
            pymeshlab.Mesh(
                np.asarray(mesh.vertices, np.float64), np.asarray(mesh.faces, np.int32)
            )
        )
        # COLLADA stores every face with its own vertices; weld them first.
        meshset.meshing_merge_close_vertices()
        meshset.meshing_decimation_quadric_edge_collapse(
            targetfacenum=faces, preservenormal=True, planarquadric=True
        )
        result = meshset.current_mesh()
        mesh = trimesh.Trimesh(
            result.vertex_matrix(), result.face_matrix(), process=False
        )
    mesh.export(path)


def numbers(values) -> str:
    # Rounded to 1e-9: drops the round-off of composing SDF poses.
    return " ".join(f"{round(v, 9) + 0.0:.9g}" for v in values)


def origin(element, T: np.ndarray) -> None:
    ET.SubElement(
        element,
        "origin",
        xyz=numbers(T[:3, 3]),
        rpy=numbers(euler_from_matrix(T, "sxyz")),
    )


def mesh_geometry(element, filename: str) -> None:
    geometry = ET.SubElement(element, "geometry")
    ET.SubElement(geometry, "mesh", filename=f"package://meshes/{filename}")


def limit(element, lower: float, upper: float) -> None:
    ET.SubElement(
        element,
        "limit",
        lower=numbers([lower]),
        upper=numbers([upper]),
        effort="0",
        velocity="0",
    )


def main(models: str, output: str) -> None:
    models, output = Path(models), Path(output)
    meshes_dir = output / "meshes"
    if meshes_dir.exists():
        shutil.rmtree(meshes_dir)
    meshes_dir.mkdir(parents=True)

    links, joints = {}, []
    load_model(models, "x500_gimbal", np.eye(4), links, joints)
    joints_by_child = {j.findtext("child"): j for j in joints}

    robot = ET.Element("robot", name="x500")
    robot.append(
        ET.Comment(
            " Generated by scripts/px4_sdf_to_urdf.py from PX4-gazebo-models x500_gimbal. "
            "Velocity and acceleration limits live in drone_planning/x500.py. "
        )
    )
    ET.SubElement(robot, "link", name="world")
    parent = "world"
    for name, kind, axis, lower, upper in VIRTUAL_JOINTS:
        child = (
            "base_link" if name == "base_yaw_joint" else name.replace("_joint", "_link")
        )
        joint = ET.SubElement(robot, "joint", name=name, type=kind)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "axis", xyz=axis)
        limit(joint, lower, upper)
        if child != "base_link":
            ET.SubElement(robot, "link", name=child)
        parent = child

    # URDF link frames: base_link keeps its SDF frame, every other link sits
    # at its parent joint.  Links are emitted parent-first.
    frames = {"base_link": links["base_link"][0]}
    order = ["base_link"]
    for name in order:
        for joint in joints:
            if joint.findtext("parent") == name:
                child = joint.findtext("child")
                frames[child] = links[child][0] @ pose(joint)
                order.append(child)

    for name in order:
        X, sdf_link = links[name]
        F = frames[name]
        link = ET.SubElement(robot, "link", name=name)
        inertial = sdf_link.find("inertial")
        if inertial is not None:
            element = ET.SubElement(link, "inertial")
            origin(element, np.linalg.inv(F) @ X @ pose(inertial))
            ET.SubElement(element, "mass", value=inertial.findtext("mass"))
            inertia = inertial.find("inertia")
            ET.SubElement(
                element,
                "inertia",
                **{
                    k: inertia.findtext(k, "0")
                    for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                },
            )
        swept = []  # rotor mesh vertices in the link frame
        for visual in sdf_link.findall("visual"):
            sdf_mesh = visual.find("geometry/mesh")
            if sdf_mesh is None:
                continue  # flat decal planes
            source = models / sdf_mesh.findtext("uri").removeprefix("model://")
            filename = source.stem + ".stl"
            mesh = load_mesh(
                source, np.array(sdf_mesh.findtext("scale", "1 1 1").split(), float)
            )
            if not (meshes_dir / filename).exists():
                write_stl(mesh, meshes_dir / filename)
            T = np.linalg.inv(F) @ X @ pose(visual)
            element = ET.SubElement(link, "visual")
            origin(element, T)
            mesh_geometry(element, filename)
            material = ET.SubElement(element, "material", name=f"{name}_color")
            ET.SubElement(material, "color", rgba=COLORS[name])
            if name in ROTORS:
                swept.append(trimesh.transform_points(mesh.vertices, T))
            else:
                element = ET.SubElement(link, "collision")
                origin(element, T)
                mesh_geometry(element, filename)
        if swept:
            points = np.concatenate(swept)
            low, high = points[:, 2].min(), points[:, 2].max()
            C = np.eye(4)
            C[2, 3] = 0.5 * (low + high)
            element = ET.SubElement(link, "collision")
            origin(element, C)
            geometry = ET.SubElement(element, "geometry")
            radius = np.hypot(points[:, 0], points[:, 1]).max()
            ET.SubElement(
                geometry,
                "cylinder",
                radius=numbers([radius]),
                length=numbers([high - low]),
            )

        joint = joints_by_child.get(name)
        if joint is None:
            continue  # base_link, reached through the virtual joints
        parent_name = joint.findtext("parent")
        kind = "fixed" if joint.get("name") in FIXED_JOINTS else joint.get("type")
        element = ET.SubElement(robot, "joint", name=joint.get("name"), type=kind)
        origin(element, np.linalg.inv(frames[parent_name]) @ F)
        ET.SubElement(element, "parent", link=parent_name)
        ET.SubElement(element, "child", link=name)
        if kind != "fixed":
            ET.SubElement(element, "axis", xyz=joint.findtext("axis/xyz"))
            limit(
                element,
                *JOINT_LIMITS.get(
                    joint.get("name"),
                    (
                        float(joint.findtext("axis/limit/lower")),
                        float(joint.findtext("axis/limit/upper")),
                    ),
                ),
            )

    # The camera sensor frame (SDF sensor pose in camera_link).
    X, camera = links["camera_link"]
    sensor = camera.find("sensor[@type='camera']")
    joint = ET.SubElement(robot, "joint", name="camera_sensor_joint", type="fixed")
    origin(joint, np.linalg.inv(frames["camera_link"]) @ X @ pose(sensor))
    ET.SubElement(joint, "parent", link="camera_link")
    ET.SubElement(joint, "child", link="camera_sensor_link")
    ET.SubElement(robot, "link", name="camera_sensor_link")

    ET.indent(robot, "  ")
    ET.ElementTree(robot).write(
        output / "x500.urdf", encoding="utf-8", xml_declaration=True
    )


if __name__ == "__main__":
    main(*sys.argv[1:])
