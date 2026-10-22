"""Shared configuration and SDF generation for the manipulation scene."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def load_scene(path):
    config = yaml.safe_load(Path(path).read_text())
    for name in object_names(config):
        item = config[name]
        for field in ("size", "position"):
            if len(item[field]) != 3 or not all(math.isfinite(v) for v in item[field]):
                raise ValueError(f"Invalid {name}.{field}")
        if min(item["size"]) <= 0:
            raise ValueError(f"Invalid {name} dimensions")
        if name != "table" and (not math.isfinite(item["mass"]) or item["mass"] <= 0):
            raise ValueError(f"{name} mass must be positive")
        if "friction" in item and (not math.isfinite(item["friction"]) or item["friction"] < 0):
            raise ValueError(f"Invalid {name} friction")
    if not math.isfinite(config["approach_clearance"]) or config["approach_clearance"] <= 0:
        raise ValueError("Approach clearance must be positive")
    if not math.isfinite(config["surface_clearance"]) or config["surface_clearance"] <= 0:
        raise ValueError("Surface clearance must be positive")
    if config["approach_clearance"] <= config["cube"]["size"][2]/2 + config["surface_clearance"]:
        raise ValueError("Pre-grasp must be above the approach endpoint")
    return config


def object_names(config):
    return ["table", "cube"] + (["support_cube"] if "support_cube" in config else [])


def model_sdf(name, item):
    root = ET.Element("sdf", version="1.9")
    model = ET.SubElement(root, "model", name=name)
    ET.SubElement(model, "static").text = "true" if name == "table" else "false"
    ET.SubElement(model, "pose").text = " ".join(map(str, item["position"])) + " 0 0 0"
    link = ET.SubElement(model, "link", name="body")
    if name != "table":
        mass = item["mass"]
        x, y, z = item["size"]
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass").text = str(mass)
        inertia = ET.SubElement(inertial, "inertia")
        for key, value in dict(ixx=mass*(y*y+z*z)/12, iyy=mass*(x*x+z*z)/12,
                               izz=mass*(x*x+y*y)/12, ixy=0, ixz=0, iyz=0).items():
            ET.SubElement(inertia, key).text = str(value)
        plugin = ET.SubElement(model, "plugin", filename="gz-sim-pose-publisher-system",
                               name="gz::sim::systems::PosePublisher")
        for key, value in dict(publish_model_pose="true", publish_link_pose="false",
                               use_pose_vector_msg="true", update_frequency="30",
                               topic=f"/{name}/pose").items():
            ET.SubElement(plugin, key).text = value
    for kind in ("collision", "visual"):
        shape = ET.SubElement(link, kind, name=name)
        box = ET.SubElement(ET.SubElement(shape, "geometry"), "box")
        ET.SubElement(box, "size").text = " ".join(map(str, item["size"]))
        if kind == "collision" and "friction" in item:
            friction = ET.SubElement(ET.SubElement(shape, "surface"), "friction")
            for engine, keys in (("ode", ("mu", "mu2")), ("bullet", ("friction", "friction2"))):
                parameters = ET.SubElement(friction, engine)
                for key in keys:
                    ET.SubElement(parameters, key).text = str(item["friction"])
        if kind == "visual":
            material = ET.SubElement(shape, "material")
            color = {"cube": "0.8 0.15 0.05 1", "support_cube": "0.1 0.3 0.8 1"}.get(name, "0.4 0.45 0.5 1")
            ET.SubElement(material, "diffuse").text = color
            ET.SubElement(material, "ambient").text = color
    return ET.tostring(root, encoding="unicode")
