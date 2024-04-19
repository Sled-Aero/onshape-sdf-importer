from onshape import Part
from textwrap import dedent

def create_airframe_config(name: str, parts: dict[str, Part]) -> str:
    header = dedent(f"""\
        #!/bin/sh
        #
        # @name {name}
        #
        # @type Quadrotor Wide
        #
        # @maintainer Sled Aero
        #
    """)

    rotor_positions = {part.identifier: part.transform[:3, 3] for part in parts.values() if 'propeller' in part.identifier}

    lines = [
        ". ${R}etc/init.d/rc.mc_defaults",
        default_param("CA_AIRFRAME", 0),
        default_param("CA_ROTOR_COUNT", 4),
        default_param("CA_ROTOR0_PX", rotor_positions["8x4x3_propeller_ccw_1"][0]),
        default_param("CA_ROTOR0_PY", -rotor_positions["8x4x3_propeller_ccw_1"][1]),
        default_param("CA_ROTOR0_PZ", rotor_positions["8x4x3_propeller_ccw_1"][2]),
        default_param("CA_ROTOR0_KM", 0.05),
        default_param("CA_ROTOR1_PX", rotor_positions["8x4x3_propeller_ccw_2"][0]),
        default_param("CA_ROTOR1_PY", -rotor_positions["8x4x3_propeller_ccw_2"][1]),
        default_param("CA_ROTOR1_PZ", rotor_positions["8x4x3_propeller_ccw_2"][2]),
        default_param("CA_ROTOR1_KM", 0.05),
        default_param("CA_ROTOR2_PX", rotor_positions["8x4x3_propeller_cw_1"][0]),
        default_param("CA_ROTOR2_PY", -rotor_positions["8x4x3_propeller_cw_1"][1]),
        default_param("CA_ROTOR2_PZ", rotor_positions["8x4x3_propeller_cw_1"][2]),
        default_param("CA_ROTOR2_KM", -0.05),
        default_param("CA_ROTOR3_PX", rotor_positions["8x4x3_propeller_cw_2"][0]),
        default_param("CA_ROTOR3_PY", -rotor_positions["8x4x3_propeller_cw_2"][1]),
        default_param("CA_ROTOR3_PZ", rotor_positions["8x4x3_propeller_cw_2"][2]),
        default_param("CA_ROTOR3_KM", -0.05),
        default_param("PWM_MAIN_FUNC1", 101),
        default_param("PWM_MAIN_FUNC2", 102),
        default_param("PWM_MAIN_FUNC3", 103),
        default_param("PWM_MAIN_FUNC4", 104),
        default_param("MC_ROLLRATE_P", 0.150),
        default_param("MC_ROLLRATE_I", 0.2),
        default_param("MC_ROLLRATE_D", 0.003),
        default_param("MC_PITCHRATE_P", 0.150),
        default_param("MC_PITCHRATE_I", 0.2),
        default_param("MC_PITCHRATE_D", 0.003),
        default_param("MC_YAWRATE_P", 0.150),
        default_param("MC_YAWRATE_I", 0.1),
        default_param("MC_YAWRATE_D", 0.0),
    ]

    return header + "\n" + "\n".join(lines)

def default_param(name: str, value) -> str:
    return f"param set-default {name} {value}"
