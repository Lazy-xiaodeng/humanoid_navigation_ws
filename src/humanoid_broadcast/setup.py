from glob import glob
import os

from setuptools import setup

package_name = "humanoid_broadcast"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@ubuntu.com",
    description="Broadcast playback service with automatic speaker selection.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "broadcast_service_node = humanoid_broadcast.broadcast_service_node:main",
            "call_broadcast_service = humanoid_broadcast.call_broadcast_service:main",
        ],
    },
)
