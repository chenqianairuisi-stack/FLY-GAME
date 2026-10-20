from setuptools import setup
from glob import glob

package_name = "ican_delivery_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="iCAN Delivery Sim",
    maintainer_email="todo@example.com",
    description="Full perception, flight, and ballistic delivery demo for the iCAN task 2 simulator.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "minimal_drop_demo = ican_delivery_examples.minimal_drop_demo:main",
            "full_mission = ican_delivery_examples.full_mission:main",
            "window_detector = ican_delivery_examples.window_detector:main",
        ],
    },
)
