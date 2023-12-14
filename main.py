from argparse import ArgumentParser
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

    # download_part_meshes(client, parts, meshes_path)

    trunk = find_trunk(mates)
    print(f'trunk is: {parts[trunk].identifier}')

    part_groups = collect_part_groups(trunk, parts, mates)

    for group in part_groups:
        names = ', '.join(parts[id].identifier for id in group)
        print(f'found group: {names}')

    sdf = create_sdf(part_groups, parts, args.name)
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
    transform: np.matrix
    did: str
    mid: str
    eid: str
    pid: str

    @classmethod
    def from_data(cls, data, transform: np.matrix):
        return cls(part_identifier(data['name']), transform, data['documentId'], data['documentMicroversion'], data['elementId'], data['partId'])

def extract_parts(assembly: dict) -> dict[str, Part]:
    part_transforms = {data['path'][0]: np.matrix(np.reshape(data['transform'], (4, 4))) for data in assembly['rootAssembly']['occurrences']}
    return {data['id']: Part.from_data(data, part_transforms[data['id']]) for data in assembly['rootAssembly']['instances']}

@dataclass
class Mate:
    parent: str
    child: str
    kind: str

    @classmethod
    def from_data(cls, data):
        return cls(data['matedEntities'][1]['matedOccurrence'][0], data['matedEntities'][0]['matedOccurrence'][0], data['mateType'])
    
def extract_mates(assembly: dict) -> dict[str, Mate]:
    return {data['id']: Mate.from_data(data['featureData']) for data in assembly['rootAssembly']['features']}

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

# r = client._request('get', f'/assemblies/d/{document}/w/{workspace}/e/{element}', params={'includeMateFeatures': False, 'includeNonSolids': False, 'includeMateConnectors': False, 'excludeSuppressed': False})
# print(r.json())

# client = Client('https://cad.onshape.com/api/v6', os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY'])
# print(client.get_assembly(document, workspace, element))
# print(json.dumps(client.get_assembly(document, workspace, element), indent=4))

# assembly_features = client.get_assembly_features(document, workspace, element)

# with open('assembly_features.json', 'w') as f:
#     f.write(json.dumps(assembly_features, indent=4))

# assembly = client.get_assembly(document, workspace, element)

# with open('assembly_parts.json', 'w') as f:
#     f.write(json.dumps(assembly, indent=4))

# print([instance['name'] for instance in assembly_parts['rootAssembly']['instances']])

# my_part = assembly_parts['rootAssembly']['instances'][0]
# my_stl = client.get_parts_stl(my_part['documentId'], my_part['documentMicroversion'], my_part['elementId'], my_part['partId'])

# with open('my_part.stl', 'wb') as f:
#     f.write(my_stl)

# for part in assembly_parts['rootAssembly']['instances']:
#     stl = client.get_parts_stl(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'])

#     with open(f'{part["name"].replace(">", "").replace("<", "")}.stl', 'wb') as f:
#         f.write(stl)

# for part in parts.values():
#     stl = client.get_parts_stl(part.did, part.mid, part.eid, part.pid)

#     with open(f'{part.identifier}.stl', 'wb') as f:
#         f.write(stl)

# for id, part in parts.items():
#     print(f'{part.identifier} ({id}) has translation: {part.transform[0, 3]} {part.transform[1, 3]} {part.transform[2, 3]}')

def create_sdf(part_groups: list[list[str]], parts: dict[str, Part], model_name: str) -> etree._ElementTree:
    sdf_root = etree.Element('sdf', version='1.6')
    model_element = etree.SubElement(sdf_root, 'model', name='blank_model')

    for group in part_groups:
        link_name = parts[group[0]].identifier
        link = etree.SubElement(model_element, 'link', name=link_name)

        for id in group:
            mesh_path = f'model://{model_name}/meshes/{parts[id].identifier}.stl'

            x = parts[id].transform[0, 3]
            y = parts[id].transform[1, 3]
            z = parts[id].transform[2, 3]
            (roll, pitch, yaw) = transform_matrix_to_euler_angles(parts[id].transform)

            visual = etree.SubElement(link, 'visual', name=parts[id].identifier)
            etree.SubElement(visual, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
            geometry = etree.SubElement(visual, 'geometry')
            mesh = etree.SubElement(geometry, 'mesh')
            etree.SubElement(mesh, 'uri').text = mesh_path

            collision = etree.SubElement(link, 'collision', name=parts[id].identifier)
            etree.SubElement(collision, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
            geometry = etree.SubElement(collision, 'geometry')
            mesh = etree.SubElement(geometry, 'mesh')
            etree.SubElement(mesh, 'uri').text = mesh_path

            inertial = etree.SubElement(link, 'inertial', name=parts[id].identifier)
            etree.SubElement(collision, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'
            geometry = etree.SubElement(collision, 'geometry')
            mesh = etree.SubElement(geometry, 'mesh')
            etree.SubElement(mesh, 'uri').text = mesh_path

    return etree.ElementTree(sdf_root)

def transform_matrix_to_euler_angles(matrix: np.matrix):
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])

    if not sy < 1e-6:
        x = math.atan2(matrix[2, 1], matrix[2, 2])
        y = math.atan2(-matrix[2, 0], sy)
        z = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        x = math.atan2(-matrix[1, 2], matrix[1, 1])
        y = math.atan2(-matrix[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def create_config(name: str) -> etree._ElementTree:
    model = etree.Element('model')

    etree.SubElement(model, 'name').text = name
    etree.SubElement(model, 'version').text = '1.0'
    etree.SubElement(model, 'sdf', version='1.6').text = f'{name}.sdf'

    return etree.ElementTree(model)

if __name__ == '__main__':
    main()
