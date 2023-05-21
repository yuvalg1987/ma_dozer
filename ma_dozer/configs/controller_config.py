from ma_dozer.configs.pydantic_config import BaseModel


class ControllerConfig(BaseModel):

    controller_debug_mode: bool = False
    controller_debug_x_increment: float = 1
    controller_debug_angle_increment: float = 2

    eps_delta_translation: float = 1
    eps_delta_yaw: float = 1
    eps_delta_pitch: float = 5
    eps_delta_roll: float = 5
    eps_delta_planner_xyz: float = 3
    eps_delta_planner_pqr: float = 3

    max_angle_per_step: float = 10
    max_distance_per_step: float = 10
    backward_bias: float = 35

    print_control_mod: float = 5
    eps_bound_distance: float = 0
<<<<<<< HEAD
    use_ekf: bool = True
