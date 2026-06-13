import os
from glob import glob

from setuptools import setup

package_name = "go2_utils"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "sounds"), glob("sounds/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "urdf", "dae"), glob("urdf/dae/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mattias Tofte",
    maintainer_email="mattiastofte@gmail.com",
    description="Go2 utility nodes for TF, perception, audio, and visualization.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_to_tf_bridge = go2_utils.odom_to_tf_bridge:main",
            "cloud_throttle = go2_utils.cloud_throttle:main",
            "cmd_vel_bridge_node = go2_utils.cmd_vel_bridge_node:main",
            "front_video_bridge_node = go2_utils.front_video_bridge_node:main",
            "webrtc_camera_node = go2_utils.webrtc_camera_node:main",
            "person_tracker_node = go2_utils.person_tracker_node:main",
            "audiohub_player_node = go2_utils.audiohub_player_node:main",
            "whisper_node = go2_utils.whisper_node:main",
        ],
    },
)
