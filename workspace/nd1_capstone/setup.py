import os
from glob import glob
from setuptools import find_packages, setup

package_name = "nd1_capstone"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models"), glob("models/*.sdf")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ND1",
    maintainer_email="instructor@nd1.local",
    description="ND1 캡스톤 — Node A(LLM) + Coordinator(FSM) + Node B(Nav2) + Node C(IK)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "node_a_llm = nd1_capstone.node_a_llm:main",
            "node_b_nav = nd1_capstone.node_b_nav:main",
            "node_c_grasp = nd1_capstone.node_c_grasp:main",
            "coordinator_fsm = nd1_capstone.coordinator_fsm:main",
            "linear_orchestrator = nd1_capstone.linear_orchestrator:main",
            "llm_planner = nd1_capstone.llm_planner:main",
        ],
    },
)
