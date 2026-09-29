from glob import glob
from setuptools import find_packages, setup


package_name = "ur_llm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/srdf", glob("srdf/*.xacro")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Phuc",
    maintainer_email="phuc@example.com",
    description="Control a simulated UR3e from natural language using validated robot skills.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "llm_skill_node = ur_llm_control.node:main",
            "test_llm = ur_llm_control.llm_test:main",
        ],
    },
)
