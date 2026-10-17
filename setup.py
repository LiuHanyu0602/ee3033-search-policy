from glob import glob
from setuptools import find_packages, setup


package_name = "ee3033_search_policy"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
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
        ],
    },
)
