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
            "front_scan_logger = go2_utils.front_scan_logger:main",
            "go2_whisper_node = go2_utils.go2_whisper_node:main",
            "audio_dump_node = go2_utils.audio_dump_node:main",
            "go2_deepfilter_test_node = go2_utils.go2_deepfilter_test_node:main",
            "go2_faster_whisper_test_node = go2_utils.go2_faster_whisper_test_node:main",
        ],
    },
)