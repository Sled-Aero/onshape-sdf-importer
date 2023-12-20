from argparse import ArgumentParser
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from lxml import etree
import numpy as np
import requests
import json
import os

HEADERS = {'Accept': 'application/json;charset=UTF-8;qs=0.09', 'Content-Type': 'application/json'}

class Client:
    def __init__(self, base_url: str, access_key: str, secret_key: str):
        self._session = requests.Session()
        self._session.auth = (access_key, secret_key)
        self._base_url = base_url

    def _api_request(self, method: str, path: str, params={}, body={}) -> requests.Response:
        return self._session.request(method, self._base_url + path, params=params, headers=HEADERS, data=body, allow_redirects=False)
    
    def get_assembly(self, did: str, wid: str, eid: str) -> dict:
        return self._api_request('get', f'/assemblies/d/{did}/w/{wid}/e/{eid}', params={'includeMateFeatures': 'true', 'includeNonSolids': 'true', 'includeMateConnectors': 'true'}).json()
    
    def get_assembly_features(self, did: str, wid: str, eid: str) -> dict:
        return self._api_request('get', f'/assemblies/d/{did}/w/{wid}/e/{eid}/features').json()
    
    def get_parts_stl(self, did: str, mid: str, eid: str, pid: str) -> bytes:
        redirect = self._api_request('get', f'/parts/d/{did}/m/{mid}/e/{eid}/partid/{pid}/stl', params={'mode': 'binary', 'units': 'meter'}).headers['Location']
        return self._session.get(redirect).content
    
    def get_mass_properties(self, did: str, mid: str, eid: str, pid: str) -> dict:
        return self._api_request('get', f'/parts/d/{did}/m/{mid}/e/{eid}/partid/{pid}/massproperties', params={'useMassPropertyOverrides': 'true'}).json()

def main():
    parser = ArgumentParser(description='Onshape To Gazebo SDF Importing Tool')
    parser.add_argument('name', type=str, help='Model name')
    parser.add_argument('models_path', type=str, help='Models directory')
    parser.add_argument('document', type=str, help='Onshape document id')
    parser.add_argument('workspace', type=str, help='Onshape workspace id')
    parser.add_argument('element', type=str, help='Onshape element id')
    args = parser.parse_args()

    model_path = Path(args.models_path, args.name)
    model_path.mkdir(exist_ok=True, parents=True)

    meshes_path = Path(model_path, 'meshes/')
    meshes_path.mkdir(exist_ok=True)

    client = Client('https://cad.onshape.com/api/v6', os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY'])
    assembly = client.get_assembly(args.document, args.workspace, args.element)

    parts = extract_parts(assembly)
    mates = extract_mates(assembly)
    part_mass_properties = get_mass_properties(parts, client)

    # download_part_meshes(client, parts, meshes_path)

    trunk = find_trunk(mates)
    print(f'Trunk is: {parts[trunk].identifier}')

    grouped_parts = collect_part_groups(trunk, parts, mates)

    print('Fixed part groups:')
    for group in grouped_parts:
        print(', '.join(parts[id].identifier for id in group))

    fixed_groups = []
    for group in grouped_parts:
        masses = [part_mass_properties[part].mass for part in group]
        group_mass = sum(masses)

        center_of_masses = [np.dot(parts[part].transform, np.append(part_mass_properties[part].center_of_mass, 1))[:3] for part in group]
        group_center_of_mass = sum(masses[i] * center_of_masses[i] for i in range(len(masses))) / len(masses) / group_mass

        # for p in group: print(part_mass_properties[p].inertia)

        group_inertia = sum(part_mass_properties[part].inertia for part in group)

        fixed_groups.append(FixedGroup({part: parts[part] for part in group}, MassProperties(group_mass, group_center_of_mass, group_inertia)))

    sdf = create_sdf(fixed_groups, parts, mates, args.name)
    sdf.write(Path(model_path, f'{args.name}.sdf'), pretty_print=True, xml_declaration=True, encoding='utf-8')

    config = create_config('thing')
    config.write(Path(model_path, 'model.config'), pretty_print=True, xml_declaration=True, encoding='utf-8')

def part_identifier(name: str):
    clean = name.replace('<', '').replace('>', '')
    words = clean.split(' ')
    return '_'.join(words).lower()

@dataclass
class Part:
    identifier: str
    transform: np.ndarray
    did: str
    mid: str
    eid: str
    pid: str

    @classmethod
    def from_data(cls, data, transform: np.ndarray):
        return cls(part_identifier(data['name']), transform, data['documentId'], data['documentMicroversion'], data['elementId'], data['partId'])

def extract_parts(assembly: dict) -> dict[str, Part]:
    part_transforms = {data['path'][0]: np.reshape(data['transform'], (4, 4)) for data in assembly['rootAssembly']['occurrences']}
    return {data['id']: Part.from_data(data, part_transforms[data['id']]) for data in assembly['rootAssembly']['instances']}

@dataclass
class Mate:
    parent: str
    child: str
    kind: str
    origin: np.ndarray
    x_axis: np.ndarray
    y_axis: np.ndarray
    z_axis: np.ndarray

    @classmethod
    def from_data(cls, data):
        return cls(data['matedEntities'][1]['matedOccurrence'][0], data['matedEntities'][0]['matedOccurrence'][0], data['mateType'], data['matedEntities'][1]['matedCS']['origin'], data['matedEntities'][1]['matedCS']['xAxis'], data['matedEntities'][1]['matedCS']['yAxis'], data['matedEntities'][1]['matedCS']['zAxis'])
    
def extract_mates(assembly: dict) -> dict[str, Mate]:
    return {data['id']: Mate.from_data(data['featureData']) for data in assembly['rootAssembly']['features']}

@dataclass
class MassProperties:
    mass: float
    center_of_mass: np.ndarray
    inertia: np.ndarray

    @classmethod
    def from_data(cls, data):
        return cls(data['mass'][0], np.reshape(data['centroid'][:3], 3), np.reshape(data['inertia'][:9], (3, 3)))

def get_mass_properties(parts: dict[str, Part], client: Client) -> dict[str, MassProperties]:
    mass_properties = {}

    for id, part in parts.items():
        data = client.get_mass_properties(part.did, part.mid, part.eid, part.pid)
        mass_properties[id] = MassProperties.from_data(next(iter(data['bodies'].values())))

    return mass_properties

@dataclass
class FixedGroup:
    parts: dict[str, Part]
    mass_properties: MassProperties

def download_part_meshes(client: Client, parts: dict[str, Part], meshes_path: Path):
    for part in parts.values():
        mesh = client.get_parts_stl(part.did, part.mid, part.eid, part.pid)

        with open(Path(meshes_path, part.identifier).with_suffix('.stl'), 'wb') as f:
            f.write(mesh)

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

def create_sdf(fixed_groups: list[FixedGroup], parts: dict[str, Part], mates: dict[str, Mate], model_name: str) -> etree._ElementTree:
    sdf_root = etree.Element('sdf', version='1.6')
    model_element = etree.SubElement(sdf_root, 'model', name='blank_model')

    for i, group in enumerate(fixed_groups):
        # print('g: ', [part.identifier for part in group.parts.values()])
        # link_name = next(iter(group.parts.values())).identifier
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

        (x, y, z) = group.mass_properties.center_of_mass
        inertial = etree.SubElement(link, 'inertial')
        etree.SubElement(inertial, 'pose').text = f'{x} {y} {z} 0 0 0'
        etree.SubElement(inertial, 'mass').text = str(group.mass_properties.mass)
        # etree.SubElement(inertial, 'inertia').text = str(group.mass_properties.mass)

    def find_group(part: str) -> int:
        for i, group in enumerate(fixed_groups):
            if part in group.parts:
                return i
        
        return None

    for mate in mates.values():
        if mate.kind != 'FASTENED':
            # print(f'{parts[mate.parent].identifier} -> {parts[mate.child].identifier}')

            parent_identifier = parts[mate.parent].identifier
            child_identifier = parts[mate.child].identifier

            parent_group = find_group(mate.parent)
            child_group = find_group(mate.child)

            joint = etree.SubElement(model_element, 'joint', name=f'{parent_identifier}:{child_identifier}', type=mate.kind.lower())

            local_transform = np.identity(4)
            local_transform[:3, 3] = mate.origin
            transform = np.dot(fixed_groups[parent_group].parts[mate.parent].transform, local_transform)

            # print('local_transform:', local_transform[:3, 3])
            # print('parent:', fixed_groups[parent_group].parts[mate.parent].transform[:3, 3])
            # print('new transform:', transform[:3, 3])

            joint_to_part = np.eye(4)
            joint_to_part[:3, :3] = np.stack((
                np.array(mate.x_axis),
                np.array(mate.y_axis),
                np.array(mate.z_axis)
            )).T

            transform = np.dot(transform, joint_to_part)

            (x, y, z) = transform[:3, 3]
            (roll, pitch, yaw) = Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')
            etree.SubElement(joint, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'

            etree.SubElement(joint, 'parent').text = f'group_{parent_group}'
            etree.SubElement(joint, 'child').text = f'group_{child_group}'

            axis = etree.SubElement(joint, 'axis')
            etree.SubElement(axis, 'xyz').text = '0 0 1'

    return etree.ElementTree(sdf_root)

def skew_symmetric_matrix(r):
    return np.ndarray([
        [0, -r[2], r[1]],
        [r[2], 0, -r[0]],
        [-r[1], r[0], 0]
    ])

def combine_moment_of_inertia_matrices(inertial_1: np.ndarray, inertial_2: np.ndarray, mass_1: float, mass_2: float, offset: np.ndarray):
    skew_matrix = skew_symmetric_matrix(offset)
    return inertial_1 + inertial_2 + mass_1 * mass_2 * np.dot(skew_matrix, skew_matrix)

def create_config(name: str) -> etree._ElementTree:
    model = etree.Element('model')

    etree.SubElement(model, 'name').text = name
    etree.SubElement(model, 'version').text = '1.0'
    etree.SubElement(model, 'sdf', version='1.6').text = f'{name}.sdf'

    return etree.ElementTree(model)

if __name__ == '__main__':
    main()
