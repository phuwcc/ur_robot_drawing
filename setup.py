from glob import glob
from setuptools import find_packages, setup


package_name = "ur_llm_camera"

setup(
    name=package_name,
    version="0.2.0",
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
    description="Camera-grounded LLM planning with occupied-zone recovery for a simulated UR3e.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "llm_skill_node = ur_llm_camera.node:main",
        ],
    },
)
