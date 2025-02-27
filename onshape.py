from dataclasses import dataclass
from pathlib import Path

import numpy as np
from api import Client

import pprint as pp

def fetch_assembly(client: Client, document_id: str, workspace_id: str, element_id: str, meshes_path: Path) -> tuple[dict[str, 'Part'], dict[str, 'Mate']]:    
    print('Fetching assembly')
    assembly = client.get_assembly(document_id, workspace_id, element_id)
    assembly_features = client.get_assembly_features(document_id, workspace_id, element_id)
    print('Fetching mass properties')
    mass_props = get_mass_properties(assembly, client)

    parts = extract_parts(assembly, mass_props)
    mates = extract_mates(assembly)

    # print("\n\n\n                  --------------\n\ngetting assembly features")
    for feature in assembly_features['features']:

        # get joint limits (could also hardcode the index since it should always be the same from the api) 
        # TODO: review, should we rely on onshape api? and can limits ever be 0.0?
        min_z_index = next((i for i, e in enumerate(feature['parameters']) if e['parameterId'] == 'limitAxialZMin'), None)
        if min_z_index:
            lower = float(feature['parameters'][min_z_index]['expression'][:-3]) * (np.pi / 180)
            if lower != 0.0:
                mates[feature['featureId']].limits['lower'] = lower
        
        max_z_index = next((i for i, e in enumerate(feature['parameters']) if e['parameterId'] == 'limitAxialZMax'), None)
        if max_z_index:
            upper = float(feature['parameters'][max_z_index]['expression'][:-3]) * (np.pi / 180)
            if upper != 0.0:
                mates[feature['featureId']].limits['upper'] = upper


    # uncomment when making changes in onshape
    print('Downloading part meshes')
    # download_part_meshes(client, assembly, meshes_path)

    return parts, mates

def part_identifier(name: str):
    clean = name.replace('<', '').replace('>', '')
    words = clean.split(' ')
    return '_'.join(words).lower()

@dataclass
class MassProperties:
    mass: float
    com: np.ndarray
    inertia: np.ndarray

    def apply_transform(self, transform: np.ndarray) -> 'MassProperties':
        rotation = transform[:3, :3]
        # print('com+1', np.append(self.com, 1))
        return MassProperties(self.mass, np.dot(transform, np.append(self.com, 1))[:3], np.dot(np.dot(rotation, self.inertia), rotation.T))
    
    def inertia_at_point(self, point: np.ndarray) -> np.ndarray:
        offset = self.com - point
        return self.inertia + (np.dot(offset, offset) * np.identity(3) - np.outer(offset, offset)) * self.mass

    @classmethod
    def from_data(cls, data):
        return cls(data['mass'][0], np.reshape(data['centroid'][:3], 3), np.reshape(data['inertia'][:9], (3, 3)))

def get_mass_properties(assembly: dict, client: Client) -> dict[str, MassProperties]:
    mass_props = {}

    for data in assembly['rootAssembly']['instances']:
        mass_data = client.get_mass_properties(data['documentId'], data['documentMicroversion'], data['elementId'], data['partId'])
        mass_props[data['id']] = MassProperties.from_data(next(iter(mass_data['bodies'].values())))

    return mass_props

def download_part_meshes(client: Client, assembly: dict, meshes_path: Path):
    for data in assembly['rootAssembly']['instances']:
        mesh = client.get_parts_stl(data['documentId'], data['documentMicroversion'], data['elementId'], data['partId'])

        with open(Path(meshes_path, part_identifier(data['name'])).with_suffix('.stl'), 'wb') as f:
            f.write(mesh)

@dataclass
class Part:
    identifier: str
    transform: np.ndarray
    mass_props: MassProperties

def extract_parts(assembly: dict, mass_props: dict[str, MassProperties]) -> dict[str, Part]:
    # for data in assembly['rootAssembly']['occurrences']:
    #     print('\n\n\n\nhello', data['transform'], np.reshape(data['transform'], (4, 4)))
    part_transforms = {data['path'][0]: np.reshape(data['transform'], (4, 4)) for data in assembly['rootAssembly']['occurrences']}
    return {data['id']: Part(part_identifier(data['name']), part_transforms[data['id']], mass_props[data['id']]) for data in assembly['rootAssembly']['instances']}

@dataclass
class Mate:
    parent: str
    child: str
    kind: str
    origin: np.ndarray
    rotation: np.ndarray
    limits: dict[str, float]

    @classmethod
    def from_data(cls, data):
        features = data['matedEntities'][1]['matedCS']
        return cls(data['matedEntities'][1]['matedOccurrence'][0], data['matedEntities'][0]['matedOccurrence'][0], data['mateType'], features['origin'], np.stack((features['xAxis'], features['yAxis'], features['zAxis'])).T, {'lower': -7.0, 'upper': 7.0})
    
def extract_mates(assembly: dict) -> dict[str, Mate]:
    return {data['id']: Mate.from_data(data['featureData']) for data in assembly['rootAssembly']['features'] if 'matedEntities' in data['featureData']}
