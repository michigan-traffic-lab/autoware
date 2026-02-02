import os
from glob import glob
from setuptools import find_packages, setup

package_name = "autoware_cosim_http"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "requests", "pyyaml", "pyproj"],
    zip_safe=True,
    maintainer="Zhijie Qiao",
    maintainer_email="zhijieq@umich.edu",
    description="Autoware Co-simulation package (HTTP-based, no Redis)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_vehicle_plugin = autoware_cosim_http.autoware_vehicle_plugin:main",
            "autoware_tls_plugin = autoware_cosim_http.autoware_tls_plugin:main",
            "autoware_dummy_grid = autoware_cosim_http.autoware_dummy_grid:main",
            "terasim_tick_driver = autoware_cosim_http.terasim_tick_driver:main",
        ],
    },
)
