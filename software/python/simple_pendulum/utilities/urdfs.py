import re
import xml.etree.ElementTree as ET
from lxml import etree


def remove_namespaces(tree):
    # for el in tree.getiterator():
    for el in tree.iter():
        match = re.match("^(?:\{.*?\})?(.*)$", el.tag)
        if match:
            el.tag = match.group(1)


def set_parameters_in_urdf(
    urdf_in, urdf_out, mass, inertia, length, damping, coulomb_friction, torque_limit
):
    # ET.register_namespace("xsi", "http://www.w3.org/2001/XMLSchema-instance")
    tree = ET.parse(urdf_in, parser=etree.XMLParser())
    remove_namespaces(tree)
    root = tree.getroot()

    for joint in root.iter("joint"):
        if joint.attrib["name"] == "shoulder":
            for a in joint.iter("dynamics"):
                a.attrib["damping"] = str(damping)
                a.attrib["friction"] = str(coulomb_friction)
            for a in joint.iter("limit"):
                a.attrib["effort"] = str(torque_limit)
            for a in joint.iter("origin"):
                a.attrib["xyz"] = "0 " + str(length) + " 0"

    for link in root.iter("link"):
        if link.attrib["name"] == "upper_link":
            for a in link.iter("inertial"):
                for aa in a.iter("mass"):
                    aa.attrib["value"] = str(mass)
                for aa in a.iter("origin"):
                    aa.attrib["xyz"] = "0 0 " + str(-length)
                for aa in a.iter("inertia"):
                    aa.attrib["iyy"] = str(inertia)
            for a in link.iter("visual"):
                for aa in a.iter("geometry"):
                    for aaa in aa.iter("cylinder"):
                        aaa.attrib["length"] = str(length)

    tree.write(urdf_out)
