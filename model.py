import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from lxml import etree

from onshape import MassProperties, Mate, Part

def create_sdf(name: str, parts: dict[str, Part], mates: dict[str, Mate]) -> etree._ElementTree:
    if len(mates) == 0:
        trunk = next(iter(parts.keys()))
        print('No mates!')
    else:
        trunk = find_trunk(mates)

    print(f'Root part: {parts[trunk].identifier}')

    grouped_parts = collect_part_groups(trunk, parts, mates)

    for i, group in enumerate(grouped_parts):
        print(f'Group {i}: {", ".join(parts[id].identifier for id in group)}')

    link_groups = create_link_groups(grouped_parts, parts)

    print('Creating SDF')
    sdf_root = etree.Element('sdf', version='1.6')
    model_element = etree.SubElement(sdf_root, 'model', name=name)

    insert_link_groups(model_element, link_groups, name)
    insert_joints(model_element, link_groups, parts, mates)
    etree.SubElement(model_element, 'self_collide').text = 'false'

    return etree.ElementTree(sdf_root)

@dataclass
class LinkGroup:
    parts: dict[str, Part]
    mass_props: MassProperties

def find_trunk(mates: dict[str, Mate]):
    mated_graph = {mate.child: mate.parent for mate in mates.values()}
    current = next(iter(mated_graph.values()))

    while current in mated_graph:
        current = mated_graph[current]

    return current

def collect_part_groups(trunk: str, parts: dict[str, Part], mates: dict[str, Mate]) -> list[list[str]]:
    groups = [[trunk]]

    for mate in mates.values():
        if mate.parent in groups[0]:
            if mate.kind == 'FASTENED':
                groups[0].append(mate.child)
            else:
                # NOTE: this assumes mate graph is acyclic! (maybe worth fixing this?)
                groups.extend(collect_part_groups(mate.child, parts, mates))

    return groups

def create_link_groups(grouped_parts: list[list[str]], parts: dict[str, Part]) -> list[LinkGroup]:
    link_groups = []
    for group in grouped_parts:
        link_frame_mass_props = [parts[part].mass_props.apply_transform(parts[part].transform) for part in group]

        group_mass = sum(mass_props.mass for mass_props in link_frame_mass_props)
        group_com = sum(mass_props.mass * mass_props.com for mass_props in link_frame_mass_props) / len(link_frame_mass_props) / group_mass
        group_inertia = sum(mass_props.inertia_at_point(group_com) for mass_props in link_frame_mass_props)

        link_groups.append(LinkGroup({part: parts[part] for part in group}, MassProperties(group_mass, group_com, group_inertia)))
    
    return link_groups

def insert_link_groups(model_element: etree._Element, link_groups: list[LinkGroup], model_name: str):
    for i, group in enumerate(link_groups):
        link = etree.SubElement(model_element, 'link', name=f'group_{i}')

        for part in group.parts.values():
            (x, y, z) = part.transform[:3, 3]
            (roll, pitch, yaw) = Rotation.from_matrix(part.transform[:3, :3]).as_euler('xyz')
            mesh_path = f'model://{model_name}/meshes/{part.identifier}.stl'

            visual = etree.SubElement(link, 'visual', name=part.identifier)
            etree.SubElement(visual, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
            geometry = etree.SubElement(visual, 'geometry')
            mesh = etree.SubElement(geometry, 'mesh')
            etree.SubElement(mesh, 'uri').text = mesh_path

            collision = etree.SubElement(link, 'collision', name=part.identifier)
            etree.SubElement(collision, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
            geometry = etree.SubElement(collision, 'geometry')
            mesh = etree.SubElement(geometry, 'mesh')
            etree.SubElement(mesh, 'uri').text = mesh_path

        inertial = etree.SubElement(link, 'inertial')

        (x, y, z) = group.mass_props.com
        etree.SubElement(inertial, 'pose').text = f'{x} {y} {z} 0 0 0'
        etree.SubElement(inertial, 'mass').text = str(group.mass_props.mass)

        inertia = etree.SubElement(inertial, 'inertia')
        etree.SubElement(inertia, 'ixx').text = str(group.mass_props.inertia[0, 0])
        etree.SubElement(inertia, 'ixy').text = str(group.mass_props.inertia[0, 1])
        etree.SubElement(inertia, 'ixz').text = str(group.mass_props.inertia[0, 2])
        etree.SubElement(inertia, 'iyy').text = str(group.mass_props.inertia[1, 1])
        etree.SubElement(inertia, 'iyz').text = str(group.mass_props.inertia[1, 2])
        etree.SubElement(inertia, 'izz').text = str(group.mass_props.inertia[2, 2])

def insert_joints(model_element: etree._Element, link_groups: list[LinkGroup], parts: dict[str, Part], mates: dict[str, Mate]):
    def find_group(part: str) -> int:
        for i, group in enumerate(link_groups):
            if part in group.parts:
                return i
        
        return None

    for mate in mates.values():
        if mate.kind != 'FASTENED':
            print(f'Joint {mate.kind}: {parts[mate.parent].identifier} -> {parts[mate.child].identifier}')

            parent_identifier = parts[mate.parent].identifier
            child_identifier = parts[mate.child].identifier

            parent_group = find_group(mate.parent)
            child_group = find_group(mate.child)

            joint = etree.SubElement(model_element, 'joint', name=f'{child_identifier}_joint', type=mate.kind.lower())

            local_transform = np.identity(4)
            local_transform[:3, 3] = mate.origin
            transform = np.dot(link_groups[parent_group].parts[mate.parent].transform, local_transform)

            joint_to_part = np.eye(4)
            joint_to_part[:3, :3] = mate.rotation

            transform = np.dot(transform, joint_to_part)

            (x, y, z) = transform[:3, 3]
            (roll, pitch, yaw) = Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')
            etree.SubElement(joint, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'

            etree.SubElement(joint, 'parent').text = f'group_{parent_group}'
            etree.SubElement(joint, 'child').text = f'group_{child_group}'

            axis = etree.SubElement(joint, 'axis')
            etree.SubElement(axis, 'xyz').text = '0 0 1'

            limit = etree.SubElement(axis, 'limit')
            etree.SubElement(limit, 'effort').text = '1'
            etree.SubElement(limit, 'velocity').text = '20'
