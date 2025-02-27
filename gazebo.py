import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
from pathlib import Path
from lxml import etree

def insert_gazebo_plugins(sdf: etree._ElementTree):
    print('Inserting Gazebo plugins')

    # insert_rotor_plugins(sdf, RotorParameters(time_constant_up=0.1, time_constant_down=0.2, max_rot_velocity=1100, motor_constant=9.0e-06, moment_constant=0.08, rotor_drag_coefficient=0.000175, rolling_moment_coefficient=0.8e-06, rotor_velocity_slowdown_sim=10))
    insert_rotor_plugins(sdf, RotorParameters(time_constant_up=0.1, time_constant_down=0.2, max_rot_velocity=600, motor_constant=5.0e-06, moment_constant=0.06, rotor_drag_coefficient=0.000175, rolling_moment_coefficient=0.8e-06, rotor_velocity_slowdown_sim=50))
    # include_sensor(sdf, 'model://gps')
    insert_imu_plugin(sdf)
    insert_barometer_plugin(sdf)
    insert_magnetometer_plugin(sdf)
    insert_mavlink_plugin(sdf, ChannelParameters(input_offset=0, input_scaling=1000, zero_position_disarmed=0, zero_position_armed=100))

def create_gazebo_config(name: str) -> etree._ElementTree:
    model = etree.Element('model')

    etree.SubElement(model, 'name').text = name
    etree.SubElement(model, 'version').text = '1.0'
    etree.SubElement(model, 'sdf', version='1.6').text = f'{name}.sdf'

    return etree.ElementTree(model)

@dataclass
class RotorParameters:
    time_constant_up: float
    time_constant_down: float
    max_rot_velocity: float
    motor_constant: float
    moment_constant: float
    rotor_drag_coefficient: float
    rolling_moment_coefficient: float
    rotor_velocity_slowdown_sim: int

def insert_rotor_plugins(tree: etree._ElementTree, parameters: RotorParameters):
    model = tree.getroot()[0]
    base_link = next(link.attrib['name'] for link in model.findall('link'))

    rosbag = etree.SubElement(model, 'plugin', { 'name': 'rosbag', 'filename': 'libgazebo_multirotor_base_plugin.so' })
    etree.SubElement(rosbag, 'robotNamespace')
    etree.SubElement(rosbag, 'linkName').text = base_link
    etree.SubElement(rosbag, 'rotorVelocitySlowdownSim').text = str(parameters.rotor_velocity_slowdown_sim)

    # for i, name in enumerate(['8x4x3_propeller_ccw_1_joint', '8x4x3_propeller_ccw_2_joint', '8x4x3_propeller_cw_1_joint', '8x4x3_propeller_cw_2_joint']): # FIXME: hardcoding!!
    #     link_name = next(joint.find('child').text for joint in model.findall('joint') if joint.attrib['name'] == name)

    #     motor = etree.SubElement(model, 'plugin', { 'name': f'{name}_model', 'filename': 'libgazebo_motor_model.so' })
    #     etree.SubElement(motor, 'robotNamespace')
    #     etree.SubElement(motor, 'jointName').text = name
    #     etree.SubElement(motor, 'linkName').text = link_name
    #     etree.SubElement(motor, 'turningDirection').text = 'ccw' if i // 2 == 0 else 'cw'
    #     etree.SubElement(motor, 'timeConstantUp').text = str(parameters.time_constant_up)
    #     etree.SubElement(motor, 'timeConstantDown').text = str(parameters.time_constant_down)
    #     etree.SubElement(motor, 'maxRotVelocity').text = str(parameters.max_rot_velocity)
    #     etree.SubElement(motor, 'motorConstant').text = str(parameters.motor_constant)
    #     etree.SubElement(motor, 'momentConstant').text = str(parameters.moment_constant)
    #     etree.SubElement(motor, 'commandSubTopic').text = '/gazebo/command/motor_speed'
    #     etree.SubElement(motor, 'motorNumber').text = str(i)
    #     etree.SubElement(motor, 'rotorDragCoefficient').text = str(parameters.rotor_drag_coefficient)
    #     etree.SubElement(motor, 'rollingMomentCoefficient').text = str(parameters.rolling_moment_coefficient)
    #     etree.SubElement(motor, 'motorSpeedPubTopic').text = f'/motor_speed/{i}'
    #     etree.SubElement(motor, 'rotorVelocitySlowdownSim').text = str(parameters.rotor_velocity_slowdown_sim)

    # below is for dragonfly, above is for grasshopper (original)
    # for i, name in enumerate(['7x4x3_propeller_ccw_1_joint', '7x4x3_propeller_ccw_2_joint', '7x4x3_propeller_cw_1_joint', '7x4x3_propeller_cw_2_joint']): # FIXME: hardcoding!!
    for i, name in enumerate(['back_l_foot_1_joint', 'front_r_foot_1_joint', 'back_r_foot_1_joint', 'front_l_foot_1_joint']): # FIXME: hardcoding!!
        link_name = next(joint.find('child').text for joint in model.findall('joint') if joint.attrib['name'] == name)

        motor = etree.SubElement(model, 'plugin', { 'name': f'{name}_model', 'filename': 'libgazebo_motor_model.so' })
        etree.SubElement(motor, 'robotNamespace')
        etree.SubElement(motor, 'jointName').text = name
        etree.SubElement(motor, 'linkName').text = link_name
        etree.SubElement(motor, 'turningDirection').text = 'cw' if i // 2 == 0 else 'ccw' # TODO: double check this
        etree.SubElement(motor, 'timeConstantUp').text = str(parameters.time_constant_up)
        etree.SubElement(motor, 'timeConstantDown').text = str(parameters.time_constant_down)
        etree.SubElement(motor, 'maxRotVelocity').text = str(parameters.max_rot_velocity)
        etree.SubElement(motor, 'motorConstant').text = str(parameters.motor_constant)
        etree.SubElement(motor, 'momentConstant').text = str(parameters.moment_constant)
        etree.SubElement(motor, 'commandSubTopic').text = '/gazebo/command/motor_speed'
        etree.SubElement(motor, 'motorNumber').text = str(i)
        etree.SubElement(motor, 'rotorDragCoefficient').text = str(parameters.rotor_drag_coefficient)
        etree.SubElement(motor, 'rollingMomentCoefficient').text = str(parameters.rolling_moment_coefficient)
        etree.SubElement(motor, 'motorSpeedPubTopic').text = f'/motor_speed/{i}'
        etree.SubElement(motor, 'rotorVelocitySlowdownSim').text = str(parameters.rotor_velocity_slowdown_sim)

def include_sensor(tree: etree._ElementTree, sensor_path: str):
    model = tree.getroot()[0]
    base_link = next(link.attrib['name'] for link in model.findall('link'))
    sensor_name = sensor_path.split('/')[-1]

    include = etree.SubElement(model, 'include')
    etree.SubElement(include, 'uri').text = sensor_path
    etree.SubElement(include, 'pose').text = '0 0 0 0 0 0'
    etree.SubElement(include, 'name').text = sensor_name

    joint = etree.SubElement(model, 'joint', {'name': f'{sensor_name}_joint', 'type': 'fixed'})
    etree.SubElement(joint, 'child').text = f'{sensor_name}::link'
    etree.SubElement(joint, 'parent').text = base_link

def insert_imu_plugin(tree: etree._ElementTree):
    model = tree.getroot()[0]
    base_link = next(link.attrib['name'] for link in model.findall('link'))

    imu = etree.SubElement(model, 'plugin', {'name': 'rotors_gazebo_imu_plugin', 'filename': 'libgazebo_imu_plugin.so'})
    etree.SubElement(imu, 'robotNamespace')
    etree.SubElement(imu, 'linkName').text = base_link
    etree.SubElement(imu, 'imuTopic').text = '/imu'
    etree.SubElement(imu, 'gyroscopeNoiseDensity').text = '0.00018665'
    etree.SubElement(imu, 'gyroscopeRandomWalk').text = '3.8785e-05'
    etree.SubElement(imu, 'gyroscopeBiasCorrelationTime').text = '1000.0'
    etree.SubElement(imu, 'gyroscopeTurnOnBiasSigma').text = '0.0087'
    etree.SubElement(imu, 'accelerometerNoiseDensity').text = '0.00186'
    etree.SubElement(imu, 'accelerometerRandomWalk').text = '0.006'
    etree.SubElement(imu, 'accelerometerBiasCorrelationTime').text = '300.0'
    etree.SubElement(imu, 'accelerometerTurnOnBiasSigma').text = '0.196'

def insert_barometer_plugin(tree: etree._ElementTree):
    model = tree.getroot()[0]

    barometer = etree.SubElement(model, 'plugin', {'name': 'barometer_plugin', 'filename': 'libgazebo_barometer_plugin.so'})
    etree.SubElement(barometer, 'robotNamespace')
    etree.SubElement(barometer, 'pubRate').text = '50'
    etree.SubElement(barometer, 'baroTopic').text = '/baro'
    etree.SubElement(barometer, 'baroDriftPaPerSec').text = '0'

def insert_magnetometer_plugin(tree: etree._ElementTree):
    model = tree.getroot()[0]

    magnetometer = etree.SubElement(model, 'plugin', {'name': 'magnetometer_plugin', 'filename': 'libgazebo_magnetometer_plugin.so'})
    etree.SubElement(magnetometer, 'robotNamespace')
    etree.SubElement(magnetometer, 'pubRate').text = '100'
    etree.SubElement(magnetometer, 'noiseDensity').text = '0.0004'
    etree.SubElement(magnetometer, 'randomWalk').text = '6.4e-06'
    etree.SubElement(magnetometer, 'biasCorrelationTime').text = '600'
    etree.SubElement(magnetometer, 'magTopic').text = '/mag'

@dataclass
class ChannelParameters:
    input_offset: float
    input_scaling: float
    zero_position_disarmed: float
    zero_position_armed: float

def insert_mavlink_plugin(tree: etree._ElementTree, parameters: ChannelParameters):
    model = tree.getroot()[0]

    ground_truth = etree.SubElement(model, 'plugin', {'name': 'groundtruth_plugin', 'filename': 'libgazebo_groundtruth_plugin.so'})
    etree.SubElement(ground_truth, 'robotNamespace')
    
    mavlink = etree.SubElement(model, 'plugin', {'name': 'mavlink_interface', 'filename': 'libgazebo_mavlink_interface.so'})
    etree.SubElement(mavlink, 'robotNamespace')
    etree.SubElement(mavlink, 'imuSubTopic').text = '/imu'
    etree.SubElement(mavlink, 'magSubTopic').text = '/mag'
    etree.SubElement(mavlink, 'baroSubTopic').text = '/baro'
    etree.SubElement(mavlink, 'mavlink_addr').text = 'INADDR_ANY'
    etree.SubElement(mavlink, 'mavlink_tcp_port').text = '4560'
    etree.SubElement(mavlink, 'mavlink_udp_port').text = '14560'
    etree.SubElement(mavlink, 'serialEnabled').text = '0'
    etree.SubElement(mavlink, 'serialDevice').text = '/dev/ttyACM0'
    etree.SubElement(mavlink, 'baudRate').text = '921600'
    etree.SubElement(mavlink, 'qgc_addr').text = 'INADDR_ANY'
    etree.SubElement(mavlink, 'qgc_udp_port').text = '14550'
    etree.SubElement(mavlink, 'sdk_addr').text = 'INADDR_ANY'
    etree.SubElement(mavlink, 'sdk_udp_port').text = '14540'
    etree.SubElement(mavlink, 'hil_mode').text = '0'
    etree.SubElement(mavlink, 'hil_state_level').text = '0'
    etree.SubElement(mavlink, 'send_vision_estimation').text = '0'
    etree.SubElement(mavlink, 'send_odometry').text = '1'
    etree.SubElement(mavlink, 'enable_lockstep').text = '1'
    etree.SubElement(mavlink, 'use_tcp').text = '1'
    etree.SubElement(mavlink, 'motorSpeedCommandPubTopic').text = '/gazebo/command/motor_speed'

    channels = etree.SubElement(mavlink, 'control_channels')
    for i in range(8):
        channel = etree.SubElement(channels, 'channel', {'name': f'rotor{i}'})
        etree.SubElement(channel, 'input_index').text = str(i)
        etree.SubElement(channel, 'input_offset').text = str(parameters.input_offset)
        etree.SubElement(channel, 'input_scaling').text = str(parameters.input_scaling)
        etree.SubElement(channel, 'zero_position_disarmed').text = str(parameters.zero_position_disarmed)
        etree.SubElement(channel, 'zero_position_armed').text = str(parameters.zero_position_armed)
        etree.SubElement(channel, 'joint_control_type').text = 'velocity'
