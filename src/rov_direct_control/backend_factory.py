"""Backend  — 根据参数创建 (actuator_backend, mode_interface) 二元组。

使用延迟 import 避免编译期依赖：
  Gazebo 模式不 import PX4 消息；
  PX4 模式不 import Gazebo transport。
"""


def create_backend(backend_type: str, node):
    """根据 backend_type 创建执行后端和模式接口。

    Args:
        backend_type: 'gazebo' 或 'px4'
        node: ROS 2 Node 实例（PX4 backend 需要用来创建 publisher）

    Returns:
        (actuator_backend, mode_interface) 二元组。
        Gazebo 模式下 mode_interface = None。
    """
    if backend_type == 'gazebo':
        from sim.gz_thruster import GzThrusterInterface
        backend = GzThrusterInterface(model_name='ustrov_0')
        return backend, None

    elif backend_type == 'px4':
        from px4_actuator import PX4ActuatorInterface
        from px4_interface import PX4Interface
        backend = PX4ActuatorInterface(node)
        mode_interface = PX4Interface(node)
        return backend, mode_interface

    else:
        raise ValueError(
            f"Unknown actuator_backend: '{backend_type}'. "
            f"Supported: 'gazebo', 'px4'."
        )
