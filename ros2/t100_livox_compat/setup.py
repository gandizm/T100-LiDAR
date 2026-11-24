import os
from pathlib import Path

from setuptools import setup


package_name = "t100_livox_compat"
here = Path(__file__).resolve().parent
repo_root = here.parents[1]
core_package = os.path.relpath(repo_root / "src" / "t100_lidar", here)

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, "t100_lidar"],
    package_dir={
        package_name: package_name,
        "t100_lidar": core_package,
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), [
            "launch/t100_mid360_compat.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gandizm",
    maintainer_email="58645849+gandizm@users.noreply.github.com",
    description="T100 UDP to Livox ROS Driver 2 CustomMsg/IMU compatibility bridge",
    license="T100 Non-Commercial License 1.0",
    entry_points={
        "console_scripts": [
            "t100_livox_compat = t100_livox_compat.node:main",
        ],
    },
)
