"""
LEAP hand configs file for IsaacLab.

Modified template from https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_assets/isaaclab_assets/robots/allegro.py
"""

import math
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators.actuator_cfg import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

# 当前使用的 LEAP USD（相对本文件目录）
# 左手网格；关节 prim 名 a_0…a_15 与 ``leap_hand_left_urdf/leap_hand_left.urdf`` 一致，仿真/真机均以此为语义基准。
_LEAP_HAND_USD_PATH = f"{Path(__file__).parent}/leap_hand_v1_left/leap_hand_left.usd"

# 右手 USD 根位置 / 根四元数（env 局部）
_RIGHT_HAND_INIT_POS = (0.0, 0.0, 0.5)
_RIGHT_HAND_INIT_ROT_WXYZ = (0.5, 0.5, -0.5, 0.5)

# 左手：根位置 / 根姿态均为**绝对量**（env 局部系，与右手无「基准 + 偏移」关系）。
# +X 左、+Y 前
LEAP_LEFT_HAND_INIT_POS = (0.04, -0.1, 0.5)
# 根四元数 (w,x,y,z)，掌心朝世界 +Z；直接改此元组即可。
_SQRT2_OVER_2 = math.sqrt(2.0) / 2.0
LEAP_LEFT_HAND_INIT_ROT_WXYZ = (0.0, _SQRT2_OVER_2, -_SQRT2_OVER_2, 0.0)

# 与真机「拇指掌骨躺下」对齐的可选仿真参考偏置（rad）；默认 −π/2（见 README 与 ``LeapHandEnvCfg.udp_mirror_thumb_a12_offset_rad``）。
LEAP_LEFT_THUMB_A12_STRAIGHT_REF_RAD = -math.pi / 2

def _is_left_hand_usd_path(usd_path: str) -> bool:
    return "left" in usd_path.replace("\\", "/").lower()


# 右手：物体 / 目标 marker 的 env 局部位置（历史配置）
OBJECT_INIT_POS_RIGHT_HAND = (-0.00, -0.1, 0.56)
GOAL_MARKER_POS_RIGHT_HAND = (-0.2, -0.45, 0.68)

# 左手：物体与目标可视化的**绝对** env 局部位置（米）；调参只改下列两元组，勿相对右手推算。
# 若物体落在「左前方」相对掌心：略增 X（向手中心线）、略增 Y（向掌根/手腕收），必要时再调 LEAP_LEFT_HAND_INIT_POS。
OBJECT_INIT_POS_LEFT_HAND = (0.11, -0.03, 0.53)
GOAL_MARKER_POS_LEFT_HAND = (-0.09, -0.38, 0.65)

# ``play.py --udp-mirror-port``：物体抬到掌面上方（同 XY，抬高 Z），spawn 侧 ``disable_gravity``，
# 保持动态刚体（勿用 kinematic，避免部分版本在 reset 写根状态时不响应）。减轻与手纠缠与「隐藏 USD」负担。
OBJECT_INIT_POS_LEFT_HAND_UDP_TELEOP = (0.11, -0.03, 1.08)
GOAL_MARKER_POS_LEFT_HAND_UDP_TELEOP = (-0.09, -0.38, 1.18)


def object_init_pos_for_current_asset() -> tuple[float, float, float]:
    """左手 ``OBJECT_INIT_POS_LEFT_HAND``；右手 ``OBJECT_INIT_POS_RIGHT_HAND``。"""
    if not _is_left_hand_usd_path(_LEAP_HAND_USD_PATH):
        return OBJECT_INIT_POS_RIGHT_HAND
    return OBJECT_INIT_POS_LEFT_HAND


def goal_marker_pos_for_current_asset() -> tuple[float, float, float]:
    """左手 ``GOAL_MARKER_POS_LEFT_HAND``；右手 ``GOAL_MARKER_POS_RIGHT_HAND``。"""
    if not _is_left_hand_usd_path(_LEAP_HAND_USD_PATH):
        return GOAL_MARKER_POS_RIGHT_HAND
    return GOAL_MARKER_POS_LEFT_HAND


def udp_teleop_object_and_goal_positions() -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    """``play.py --udp-mirror-port``：左手资产时返回（物体根位置, 目标 marker 位置），否则 ``None``。"""
    if not _is_left_hand_usd_path(_LEAP_HAND_USD_PATH):
        return None
    return (OBJECT_INIT_POS_LEFT_HAND_UDP_TELEOP, GOAL_MARKER_POS_LEFT_HAND_UDP_TELEOP)


def _init_root_rot_wxyz_for_usd(usd_path: str) -> tuple[float, float, float, float]:
    """左手：``LEAP_LEFT_HAND_INIT_ROT_WXYZ``；右手：``_RIGHT_HAND_INIT_ROT_WXYZ``。"""
    if not _is_left_hand_usd_path(usd_path):
        return _RIGHT_HAND_INIT_ROT_WXYZ
    return LEAP_LEFT_HAND_INIT_ROT_WXYZ


def _init_hand_root_pos_for_usd(usd_path: str) -> tuple[float, float, float]:
    """左手：``LEAP_LEFT_HAND_INIT_POS``；右手：``_RIGHT_HAND_INIT_POS``。"""
    if not _is_left_hand_usd_path(usd_path):
        return _RIGHT_HAND_INIT_POS
    return LEAP_LEFT_HAND_INIT_POS


def _left_hand_initial_joint_pos_straight_ref() -> dict[str, float]:
    """Spawn 与任务「伸直躺下」参考一致：除 a_12 外为 0（见 ``LEAP_LEFT_THUMB_A12_STRAIGHT_REF_RAD``）。"""
    pos = {f"a_{i}": 0.0 for i in range(16)}
    pos["a_12"] = LEAP_LEFT_THUMB_A12_STRAIGHT_REF_RAD
    return pos


LEAP_HAND_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=_LEAP_HAND_USD_PATH,
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=False,
            disable_gravity=True,
            retain_accelerations=False,
            enable_gyroscopic_forces=False,
            angular_damping=0.01,
            max_linear_velocity=1000.0,
            max_angular_velocity=64 / math.pi * 180.0,
            max_depenetration_velocity=1000.0,
            max_contact_impulse=1e32,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
            sleep_threshold=0.005,
            stabilization_threshold=0.0005,
            fix_root_link=True
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=_init_hand_root_pos_for_usd(_LEAP_HAND_USD_PATH),
        rot=_init_root_rot_wxyz_for_usd(_LEAP_HAND_USD_PATH),
        joint_pos=(
            _left_hand_initial_joint_pos_straight_ref()
            if _is_left_hand_usd_path(_LEAP_HAND_USD_PATH)
            else {"a_.*": 0.0}
        ),
    ),
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit_sim=0.5,
            velocity_limit_sim=100.0,
            stiffness=4.0,
            damping=0.35,
            friction=0.02,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
