from glob import glob
from setuptools import find_packages, setup


package_name = "ee3033_search_policy"


setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.json")),
        (
            "share/" + package_name + "/calibration",
            ["results/updated_calibration/calibration.npz"],
        ),
        (
            "share/" + package_name + "/models",
            ["models/tiny_search_policy_weights.npz"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Liu Hanyu",
    maintainer_email="liuhanyu0602@users.noreply.github.com",
    description="Learned viewpoint selection for EE3033 target search.",
    license="MIT",
    entry_points={
        "console_scripts": [
            (
                "search_policy_dry_run = "
                "ee3033_search_policy.search_policy_node:main"
            ),
            "planner_test = ee3033_search_policy.planner_test_node:main",
            "target_checker = ee3033_search_policy.target_checker:main",
            "target_localizer = ee3033_search_policy.target_localizer_node:main",
            "mission_manager = ee3033_search_policy.mission_manager_node:main",
            "search_manager = ee3033_search_policy.search_manager_node:main",
            "yolo_detection_adapter = ee3033_search_policy.yolo_detection_adapter_node:main",
        ],
    },
)
