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
    # pp.pprint(assembly)
    parts = extract_parts(assembly, mass_props)
    mates = extract_mates(assembly)

    pp.pprint(assembly.keys())
    pp.pprint(assembly['partStudioFeatures'])
    # pp.pprint(assembly['rootAssembly'])

    # print("\n\n\n                  --------------\n\ngetting assembly features")
    # for feature in assembly_features['features']:
    #     # pp.pprint(feature.keys())
    #     if 'Revolute' not in feature['name']:
    #         continue
    #     print('name', feature['name'], 'featureId', feature['featureId'])
    #     print([e['parameterId'] for e in feature['parameters']])
    #     # for p in feature['parameters']:
    #     #     if p['parameterId'] in ['rotation', 'rotationType', 'secondaryAxisAlignment', 'primaryAxisAlignment']:
    #     #         print(p['parameterId'], p['value'])
    #     pp.pprint(feature['parameters'])
    #     print('\n')
    #     break

        # pp.pprint(feature['parameters'][6])
        # pp.pprint(feature['mateConnectors'])
        # for p in feature['parameters']:
        #     if p['parameterId'] in ['rotation', 'rotationType', 'secondaryAxisAlignment', 'primaryAxisAlignment']:
        #         print(p['parameterId'], p['value'])
        # print(feature['mateConnectors'][0])

        # for mc in feature['mateConnectors']:
        #     print([e['parameterId'] for e in mc['parameters']])
        #     for p in mc['parameters']:
        #         # find out the rotation axis for mate connectors... those seems to be more correct than the mates themselves (aboutz instead of aboutx)
        #         if p['parameterId'] in ['rotation', 'rotationType', 'flipPrimary', 'transform', 'realign', 'secondaryAxisType']:
        #             print(p['parameterId'], p['value'])
        #     print('-----------')
        
        # pp.pprint(feature['mateConnectors'][0]['parameters'][10])

        # print('\n')
        # break
    # print("\n\n                  --------------\n\n")
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

    @classmethod
    def from_data(cls, data):
        features = data['matedEntities'][1]['matedCS']
        return cls(data['matedEntities'][1]['matedOccurrence'][0], data['matedEntities'][0]['matedOccurrence'][0], data['mateType'], features['origin'], np.stack((features['xAxis'], features['yAxis'], features['zAxis'])).T)
    
def extract_mates(assembly: dict) -> dict[str, Mate]:
    return {data['id']: Mate.from_data(data['featureData']) for data in assembly['rootAssembly']['features'] if 'matedEntities' in data['featureData']}
