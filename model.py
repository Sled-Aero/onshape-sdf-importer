import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from lxml import etree

from onshape import MassProperties, Mate, Part

def create_sdf(name: str, parts: dict[str, Part], mates: dict[str, Mate]) -> etree._ElementTree:

    # sp = ['MqoJdm+1Ylb3JuY8T', 'M5iddUD56a6Tvjfui', 'M6wJi1A1p+H1YdBrd', 'MuW8F+gplOtCagK37', 'M0L9+5pBbdjB/ZUUg', 'M58/TZ9JuxatR0eGA', 'Minc79k0tRAKdHMpQ', 'MvbH5joNpoFd/nKov', 'MBn23zO+hH4HRbBSn', 'MA53Z7dbJXzUjrO0O', 'MAo6Z7+9vmtU0c+pQ', 'MoYwIH3aTLOjV2Dwq', 'Mo9JwCHdBGAVDta0q']
    # sp = ['Mc2DFPuKuLiyW0yHc', 'MCgCab6AbGjLwcEiu', 'M3ClM1oe3fCYublOF', 'MkmKoS0po50ZsMVp8', 'MX6h9pnDIt1dXKUyD', 'MhEu+tBIA66ANBzdQ']
    # parts = {key: parts[key] for key in sp}
    # print(list(parts.keys()))

    grouped_parts = collect_part_groups(parts, mates)

    for group in grouped_parts:
        # for part in group:
        #     if mate_child := next((mate.child for mate in mates.values() if mate.kind == 'FASTENED' and mate.parent == part), None):
        #         print('mate_child', parts[mate_child].identifier)


        members = ", ".join(parts[id].identifier for id in group[1:])
        print(f'Group: {parts[group[0]].identifier} {"(" + members + ")" if members else ""}')

        # print(list(parts.keys()))
        # print(list(group1_parts.keys()))
        # print(group1_parts)
        # exit()

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
    # pose: tuple[float, float, float, float, float, float]

def collect_part_groups(parts: dict[str, Part], mates: dict[str, Mate]) -> list[list[str]]:
    groups = list([part] for part in parts.keys())

    def part_group_index(part: str) -> int:
        return next(i for i, group in enumerate(groups) if part in group)

    for mate in mates.values():
        # we only group together fastened parts
        if mate.kind == 'FASTENED':
            # NOTE: try except for parts with no next?
            try:
                parent_group = part_group_index(mate.parent)
                child_group = part_group_index(mate.child)
            except:
                continue
            # merge the two groups
            if parent_group != child_group:
                groups[parent_group].extend(groups[child_group])
                groups.pop(child_group)

    return groups

def create_link_groups(grouped_parts: list[list[str]], parts: dict[str, Part]) -> list[LinkGroup]:
    link_groups = []
    for group in grouped_parts:
        link_frame_mass_props = [parts[part].mass_props.apply_transform(parts[part].transform) for part in group]

        group_mass = sum(mass_props.mass for mass_props in link_frame_mass_props)
        group_com = sum(mass_props.mass * mass_props.com for mass_props in link_frame_mass_props) / group_mass
        group_inertia = sum(mass_props.inertia_at_point(group_com) for mass_props in link_frame_mass_props)

        link_groups.append(LinkGroup({part: parts[part] for part in group}, MassProperties(group_mass, group_com, group_inertia)))
    
    return link_groups

def insert_link_groups(model_element: etree._Element, link_groups: list[LinkGroup], model_name: str):
    for i, group in enumerate(link_groups):
        link = etree.SubElement(model_element, 'link', name=f'group_{i}')
        print(group, group.mass_props.com)
        com_x, com_y, com_z = group.mass_props.com
        etree.SubElement(link, 'pose').text = f'{com_x} {com_y} {com_z} 0 0 0'

        for part in group.parts.values():
            # HACK: does link transform always have no rotation?
            (x, y, z) = part.transform[:3, 3] - group.mass_props.com
            (roll, pitch, yaw) = Rotation.from_matrix(part.transform[:3, :3]).as_euler('xyz')
            mesh_path = f'model://{model_name}/meshes/{part.identifier}.stl'

            visual = etree.SubElement(link, 'visual', name=part.identifier)
            etree.SubElement(visual, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
            geometry = etree.SubElement(visual, 'geometry')
            mesh = etree.SubElement(geometry, 'mesh')
            etree.SubElement(mesh, 'uri').text = mesh_path

            collision = etree.SubElement(link, 'collision', name=f'{part.identifier}_collision')
            etree.SubElement(collision, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
            geometry = etree.SubElement(collision, 'geometry')
            mesh = etree.SubElement(geometry, 'mesh')
            etree.SubElement(mesh, 'uri').text = mesh_path

        inertial = etree.SubElement(link, 'inertial')
        etree.SubElement(inertial, 'pose').text = f'0 0 0 0 0 0'
        etree.SubElement(inertial, 'mass').text = str(group.mass_props.mass)

        inertia = etree.SubElement(inertial, 'inertia')
        etree.SubElement(inertia, 'ixx').text = str(max(0, group.mass_props.inertia[0, 0]))
        etree.SubElement(inertia, 'ixy').text = str(max(0, group.mass_props.inertia[0, 1]))
        etree.SubElement(inertia, 'ixz').text = str(max(0, group.mass_props.inertia[0, 2]))
        etree.SubElement(inertia, 'iyy').text = str(max(0, group.mass_props.inertia[1, 1]))
        etree.SubElement(inertia, 'iyz').text = str(max(0, group.mass_props.inertia[1, 2]))
        etree.SubElement(inertia, 'izz').text = str(max(0, group.mass_props.inertia[2, 2]))

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

            # print('mate:', mate.origin, link_groups[child_group].parts[mate.child].transform[:3, 3] - link_groups[child_group].mass_props.com)

            joint = etree.SubElement(model_element, 'joint', name=f'{child_identifier}_joint', type=mate.kind.lower())

            # local_transform = np.identity(4)
            # local_transform[:3, 3] = mate.origin
            # # transform = np.dot(link_groups[parent_group].parts[mate.parent].transform, local_transform)

            # link_group_transform = np.identity(4)
            # link_group_transform[:3, 3] = link_groups[parent_group].mass_props.com
            # transform = np.dot(link_group_transform, local_transform)

            # joint_to_part = np.eye(4)
            # joint_to_part[:3, :3] = mate.rotation

            # transform = np.dot(transform, joint_to_part)

            # HACK: skipping this for now, how to fix?
            # we are trying to find where to connect the mate to given that the link frame pose is now at the link com
            # transform = np.identity(4)


            # print("\n\n\n              --------------------")

            # print(np.round(mate.rotation))
            # # vals, vecs = np.linalg.eig(np.round(mate.rotation))
            # # index = np.argmin(np.abs(vals - 1))
            # # print(vecs)

            # print("              --------------------\n\n\n")
            

            # relative_to_global = np.dot(link_groups[parent_group].parts[mate.parent].transform, np.append(mate.origin, 1))
            # (x, y, z) = relative_to_global[:3]
            # times by the transform THEN take away th ecom of child?

            print(link_groups[child_group].parts[mate.child].transform[:3, 3] - link_groups[child_group].mass_props.com)
            # print(np.dot(link_groups[child_group].parts[mate.child].transform[:3, 3], mate.origin) - link_groups[child_group].mass_props.com)
            print(mate.origin, link_groups[parent_group].parts[mate.parent].transform[:3, 3] + np.dot(link_groups[parent_group].parts[mate.parent].transform[:3, :3], mate.origin) - link_groups[child_group].mass_props.com)

            # (x, y, z) = transform[:3, 3] # same as (0, 0, 0)
            # (x, y, z) = (0, 0, 0) # same as (0, 0, 0)
            (x, y, z) = (link_groups[parent_group].parts[mate.parent].transform[:3, 3] + np.dot(link_groups[parent_group].parts[mate.parent].transform[:3, :3], mate.origin)) - link_groups[child_group].mass_props.com
            # (x, y, z) = np.dot(link_groups[child_group].parts[mate.child].transform[:3, 3], mate.origin) - link_groups[child_group].mass_props.com
            # (x, y, z) = link_groups[child_group].parts[mate.child].transform[:3, 3] - link_groups[child_group].mass_props.com
            # (x, y, z) = link_groups[child_group].parts[mate.child].transform[:3, 3] - link_groups[child_group].mass_props.com # working one for now
            # (x, y, z) = link_groups[child_group].parts[mate.child].transform[:3, 3] + np.dot(link_groups[child_group].parts[mate.child].transform[:3, :3], link_groups[child_group].mass_props.com) 
            # (x, y, z) = np.dot(np.dot(link_groups[child_group].parts[mate.child].transform[:3, :3], mate.rotation), mate.origin)
            # (x, y, z) = np.dot(np.dot(link_groups[child_group].parts[mate.child].transform[:3, :3], mate.rotation), mate.origin)
            # print(mate.origin, link_groups[child_group].parts[mate.child].transform[:3, 3] - link_groups[child_group].mass_props.com)

            # print('rpw:', Rotation.from_matrix(link_groups[child_group].parts[mate.child].transform[:3, :3]).as_euler('xyz'), Rotation.from_matrix(transform[:3, :3]).as_euler('xyz'))
            # (roll, pitch, yaw) = Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')
            (roll, pitch, yaw) = Rotation.from_matrix(mate.rotation).as_euler('xyz')
            
            # (roll, pitch, yaw) = Rotation.from_matrix(np.dot(link_groups[child_group].parts[mate.child].transform[:3, :3], mate.rotation)).as_euler('xyz')
            # (roll, pitch, yaw) = Rotation.from_matrix(np.dot(link_groups[child_group].parts[mate.child].transform[:3, :3], np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]))).as_euler('xyz')
            # (roll, pitch, yaw) = Rotation.from_matrix(link_groups[child_group].parts[mate.child].transform[:3, :3]).as_euler('xyz')
            
            # (roll, pitch, yaw) = Rotation.from_matrix(mate.rotation).as_euler('xyz')
            etree.SubElement(joint, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'

            etree.SubElement(joint, 'parent').text = f'group_{parent_group}'
            etree.SubElement(joint, 'child').text = f'group_{child_group}'

            axis = etree.SubElement(joint, 'axis')
            # (rx, ry, rz) = np.dot(mate.rotation, np.array([0, 0, 1])) # default (without transformation?) is 0 0 1??
            # (rx, ry, rz) = np.dot(np.dot(mate.rotation, link_groups[child_group].parts[mate.child].transform[:3, :3]), np.array([0, 0, 1])) # same as above??
            # (rx, ry, rz) = np.dot(np.dot(mate.rotation, np.array([[0, 0, -1], [0, 1, 0], [-1, 0, 0]])), np.array([0, 0, 1])) # same as above??
            (rx, ry, rz) = (0, 0, 1) # ALL ONSHAPE REVOLUTE AXIS ARE ABOUT THE Z
            # print(f'rotation axis in sdf: {rx} {ry} {rz}')
            etree.SubElement(axis, 'xyz').text = f'{rx} {ry} {rz}' 
            # etree.SubElement(axis, 'use_parent_model_frame').text = 'true'
            # etree.SubElement(axis, 'initial_position').text = f'{50.57}' 

            limit = etree.SubElement(axis, 'limit')
            etree.SubElement(limit, 'effort').text = '1'
            etree.SubElement(limit, 'velocity').text = '20'

            dynamics = etree.SubElement(axis, 'dynamics')
            etree.SubElement(dynamics, 'friction').text = '1'
