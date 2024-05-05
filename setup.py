from setuptools import find_packages, setup

package_name = "viva_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/camera_override.yaml"]),
        ("share/" + package_name + "/launch", ["launch/inference.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mhubii",
    maintainer_email="martin.huber@kcl.ac.uk",
    description="Demo for PhD viva.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "inference = viva_demo.inference:main",
        ],
    },
)
