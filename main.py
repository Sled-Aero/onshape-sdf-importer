from argparse import ArgumentParser
from pathlib import Path
import numpy as np
import os
from airframe import create_airframe_config
import pickle

from api import Client
from gazebo import create_gazebo_config, insert_gazebo_plugins
from onshape import fetch_assembly
from model import create_sdf

def main():
    parser = ArgumentParser(description='Onshape To Gazebo SDF Importing Tool')
    parser.add_argument('name', type=str, help='Model name')
    parser.add_argument('models_path', type=str, help='Models directory')
    parser.add_argument('document', type=str, help='Onshape document id')
    parser.add_argument('workspace', type=str, help='Onshape workspace id')
    parser.add_argument('element', type=str, help='Onshape element id')
    parser.add_argument('-t', '--testing', action='store_true', help='Used previously fetched assembly, prevents unnecessary api calls')
    args = parser.parse_args()

    model_path = Path(args.models_path, args.name)
    model_path.mkdir(exist_ok=True, parents=True)

    meshes_path = Path(model_path, 'meshes/')
    meshes_path.mkdir(exist_ok=True)

    parts_mates = Path(os.path.join(os.getcwd(), 'parts_mates'))
    if not args.testing:
        client = Client('https://cad.onshape.com/api/v6', os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY'])
        parts, mates = fetch_assembly(client, args.document, args.workspace, args.element, meshes_path)

        parts_mates.mkdir(exist_ok=True)

        with open(os.path.join(parts_mates, 'parts.pickle'), 'wb') as handle:
            pickle.dump(parts, handle, protocol=pickle.HIGHEST_PROTOCOL)

        with open(os.path.join(parts_mates, 'mates.pickle'), 'wb') as handle:
            pickle.dump(mates, handle, protocol=pickle.HIGHEST_PROTOCOL)

    else:
        print('Using stored assembly')
        # NOTE: might want to add a check for whether the files or the directory exits or not
        with open(os.path.join(parts_mates, 'parts.pickle'), 'rb') as handle:
            parts = pickle.load(handle)

        with open(os.path.join(parts_mates, 'mates.pickle'), 'rb') as handle:
            mates = pickle.load(handle)

    sdf = create_sdf(args.name, parts, mates)
    insert_gazebo_plugins(sdf)
    sdf.write(Path(model_path, f'{args.name}.sdf'), pretty_print=True, xml_declaration=True, encoding='utf-8')

    config = create_gazebo_config(args.name)
    config.write(Path(model_path, 'model.config'), pretty_print=True, xml_declaration=True, encoding='utf-8')

    # airframe = create_airframe_config(args.name, parts)
    # open('airframe', 'w').write(airframe)

if __name__ == '__main__':
    main()

    # model_path = Path('/Users/liam/src/sled/PX4-SITL_gazebo/models', 'test_model')
    # model_path.mkdir(exist_ok=True, parents=True)

    # meshes_path = Path(model_path, 'meshes/')
    # meshes_path.mkdir(exist_ok=True)

    # client = Client('https://cad.onshape.com/api/v6', os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY'])
    # parts, mates = fetch_assembly(client, 'adb51efa6f82da6d1426ed1d', 'bd05296ab9c04e1ed0040cc4', '7ea38088a4a7139634cb0761', meshes_path)
    # # parts, mates = fetch_assembly(client, 'adb51efa6f82da6d1426ed1d', 'bd05296ab9c04e1ed0040cc4', 'd6712ead4eea90f6aa35a552', meshes_path)

    # sdf = create_sdf('test_model', parts, mates)
    # sdf.write(Path(model_path, f'test_model.sdf'), pretty_print=True, xml_declaration=True, encoding='utf-8')

    # config = create_gazebo_config('test_model')
    # config.write(Path(model_path, 'model.config'), pretty_print=True, xml_declaration=True, encoding='utf-8')
