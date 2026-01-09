import os
import glob
from setuptools import find_packages, setup

package_name = "autoware_cosim"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhijie Qiao",
    maintainer_email="zhijieq@umich.edu",
    description="Autoware Co-simulation package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_vehicle_plugin = autoware_cosim.autoware_vehicle_plugin:main",
            "autoware_vehicle_plugin_http = autoware_cosim.autoware_vehicle_plugin_http:main",
            "autoware_vehicle_report = autoware_cosim.autoware_vehicle_report:main",
            "autoware_tls_plugin = autoware_cosim.autoware_tls_plugin:main",
            "autoware_dummy_grid = autoware_cosim.autoware_dummy_grid:main",
            "autoware_planning = autoware_cosim.autoware_planning:main",
        ],
    },
)
