"""Implement the organizer's model/SDK here once its specification is available.

The callable runs in the ROS container. Install required dependencies in the
Dockerfile, mount official assets separately, and keep credentials out of Git.
Use bounded timeouts for SDK/network calls. This is NOT an official API schema.
"""


def solve_task(*, question: str, context: dict, config: dict) -> dict:
    """Return red/blue counts, first color, red_zone and blue_zone.

    Example normalized output:
        {"red": 2, "blue": 3, "first": "red",
         "red_zone": "B", "blue_zone": "C"}

    Replace this body with official inference, parsing, and semantic validation
    against the received question. Raising an exception stops the mission.
    """
    raise RuntimeError('官方模型适配器尚未实现：请按组委会 SDK/协议实现 '
                       'ros_ws/official_model_adapter.py 的 solve_task；禁止回退本地 Qwen')
