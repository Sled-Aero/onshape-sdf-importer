from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import numpy as np
import requests
import json
import os

auth=(os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY'])
headers = {'Accept': 'application/json;charset=UTF-8;qs=0.09', 'Content-Type': 'application/json'}

document = 'f1acb5938f411c5a47db608c'
workspace = 'd20d9753a184585e00e13e9f'
element = '34860e26f5a79791793441a0'
# url = f'https://cad.onshape.com/api/v6/assemblies/d/{document}/w/{workspace}/e/{element}?includeMateFeatures=false&includeNonSolids=false&includeMateConnectors=false&excludeSuppressed=false'
# url = f'https://cad.onshape.com/api/v6/assemblies/d/{document}/w/{workspace}/e/{element}/features'
# url = f'https://cad.onshape.com/api/v6/parts/d/{document}/w/{workspace}/e/{element}'

# response = requests.get(url, auth=auth, headers=headers)
# out = json.dumps(response.json(), indent=4)

# # print(json.dumps(response.json(), indent=4))

# with open('out.json', 'w') as f:
#     f.write(out)

class Client:
    def __init__(self, base_url: str, access_key: str, secret_key: str):
        self._session = requests.Session()
        self._session.auth = (access_key, secret_key)
        self._base_url = base_url

    def _api_request(self, method: str, path: str, params={}, headers={}, body={}) -> requests.Response:
        return self._session.request(method, self._base_url + path, params=params, headers=headers, data=body, allow_redirects=False)
    
    def get_assembly(self, did: str, wid: str, eid: str) -> dict:
        return self._api_request('get', f'/assemblies/d/{did}/w/{wid}/e/{eid}', params={'includeMateFeatures': 'true', 'includeNonSolids': 'true', 'includeMateConnectors': 'true'}).json()
    
    def get_assembly_features(self, did: str, wid: str, eid: str) -> dict:
        return self._api_request('get', f'/assemblies/d/{did}/w/{wid}/e/{eid}/features').json()
    
    def get_parts_stl(self, did: str, mid: str, eid: str, pid: str) -> bytes:
        # hasher = hashlib.sha1()
        # hasher.update(pid.encode('utf-8'))
        # encoded_pid = hasher.hexdigest()
        # escaped_pid = encoded_pid.replace('/', '%2f')

        redirect = self._api_request('get', f'/parts/d/{did}/m/{mid}/e/{eid}/partid/{pid}/stl', params={'mode': 'binary', 'units': 'meter'}).headers['Location']
        return self._session.get(redirect).content
    
# r = client._request('get', f'/assemblies/d/{document}/w/{workspace}/e/{element}', params={'includeMateFeatures': False, 'includeNonSolids': False, 'includeMateConnectors': False, 'excludeSuppressed': False})
# print(r.json())

client = Client('https://cad.onshape.com/api/v6', os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY'])
# print(client.get_assembly(document, workspace, element))
# print(json.dumps(client.get_assembly(document, workspace, element), indent=4))

# assembly_features = client.get_assembly_features(document, workspace, element)

# with open('assembly_features.json', 'w') as f:
#     f.write(json.dumps(assembly_features, indent=4))

assembly = client.get_assembly(document, workspace, element)

with open('assembly_parts.json', 'w') as f:
    f.write(json.dumps(assembly, indent=4))

# print([instance['name'] for instance in assembly_parts['rootAssembly']['instances']])

# my_part = assembly_parts['rootAssembly']['instances'][0]
# my_stl = client.get_parts_stl(my_part['documentId'], my_part['documentMicroversion'], my_part['elementId'], my_part['partId'])

# with open('my_part.stl', 'wb') as f:
#     f.write(my_stl)

# for part in assembly_parts['rootAssembly']['instances']:
#     stl = client.get_parts_stl(part['documentId'], part['documentMicroversion'], part['elementId'], part['partId'])

#     with open(f'{part["name"].replace(">", "").replace("<", "")}.stl', 'wb') as f:
#         f.write(stl)
    
def get_part_identifier(name: str):
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
        return cls(get_part_identifier(data['name']), transform, data['documentId'], data['documentMicroversion'], data['elementId'], data['partId'])

part_transforms = {data['path'][0]: np.matrix(np.reshape(data['transform'], (4, 4))) for data in assembly['rootAssembly']['occurrences']}
parts = {data['id']: Part.from_data(data, part_transforms[data['id']]) for data in assembly['rootAssembly']['instances']}

@dataclass
class Mate:
    parent: str
    child: str

    @classmethod
    def from_data(cls, data):
        return cls(data['matedEntities'][1]['matedOccurrence'][0], data['matedEntities'][0]['matedOccurrence'][0])

mates = {data['id']: Mate.from_data(data['featureData']) for data in assembly['rootAssembly']['features']}

def find_trunk(mates: dict[str, Mate]):
    mated_graph = {mate.child: mate.parent for mate in mates.values()}
    current = next(iter(mated_graph.values()))

    while current in mated_graph:
        current = mated_graph[current]

    return current

trunk_id = find_trunk(mates)
print('trunk is:', parts[trunk_id].identifier)

# for part in parts.values():
#     stl = client.get_parts_stl(part.did, part.mid, part.eid, part.pid)

#     with open(f'{part.identifier}.stl', 'wb') as f:
#         f.write(stl)

def transform_matrix_to_euler_angles(matrix: np.matrix):
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(matrix[2, 1], matrix[2, 2])
        y = math.atan2(-matrix[2, 0], sy)
        z = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        x = math.atan2(-matrix[1, 2], matrix[1, 1])
        y = math.atan2(-matrix[2, 0], sy)
        z = 0

    return np.array([x, y, z])


from lxml import etree

sdf_root = etree.Element('sdf', version='1.6')
model_element = etree.SubElement(sdf_root, 'model', name='blank_model')

base_link = etree.SubElement(model_element, 'link', name='base')

for part in parts.values():
    visual = etree.SubElement(base_link, 'visual', name=part.identifier)

    x = part.transform[0, 3]
    y = part.transform[1, 3]
    z = part.transform[2, 3]
    (roll, pitch, yaw) = transform_matrix_to_euler_angles(part.transform)
    pose = etree.SubElement(visual, 'pose').text = f'{x} {y} {z} {roll} {pitch} {yaw}'

    geometry = etree.SubElement(visual, 'geometry')
    mesh = etree.SubElement(geometry, 'mesh')
    uri = etree.SubElement(mesh, 'uri').text = f'file:///Users/liam/src/sled/onshape-sdf-importer/thing/{part.identifier}.stl'

for id, part in parts.items():
    print(f'{part.identifier} ({id}) has translation: {part.transform[0, 3]} {part.transform[1, 3]} {part.transform[2, 3]}')

model_path = Path('./thing/')
model_path.mkdir(parents=True, exist_ok=True)

sdf_tree = etree.ElementTree(sdf_root)
sdf_tree.write(Path(model_path, 'thing.sdf'), pretty_print=True, xml_declaration=True, encoding='utf-8')

def create_config(name: str) -> etree._ElementTree:
    model = etree.Element('model')

    etree.SubElement(model, 'name').text = name
    etree.SubElement(model, 'version').text = '1.0'
    etree.SubElement(model, 'sdf', version='1.6').text = f'{name}.sdf'

    return etree.ElementTree(model)

config = create_config('thing')
config.write(Path(model_path, 'model.config'), pretty_print=True, xml_declaration=True, encoding='utf-8')
