from setuptools import find_packages, setup


package_name = "radar_parse_em_wave"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=["rmuc", "rmuc.*"]),
    package_data={"rmuc": ["web/index.html"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Radar Parse EM Wave Maintainers",
    maintainer_email="rmuc@example.com",
    description="Radar electromagnetic wave parser protocol node and ROS2 bridge.",
    license="Proprietary",
)
