from setuptools import find_packages, setup

package_name = "autoware_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhijie Qiao",
    maintainer_email="zhijieq@umich.edu",
    description="Receive planning from autoware and convert to preview control format",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_planning = autoware_planning.autoware_planning:main",
        ],
    },
)
