from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "blind_navigation_system"


setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(exclude=("tests",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md", "LICENSE"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/maps", glob("maps/*.yaml") + glob("maps/*.pgm")),
    ],
    install_requires=[
        "setuptools",
        "numpy>=1.21,<3",
        "PyAudio>=0.2.13,<1",
        "webrtcvad-wheels>=2.0.14,<3",
        "websockets>=11,<14",
        'Jetson.GPIO>=2.1; platform_machine == "aarch64"',
    ],
    zip_safe=False,
    maintainer="Blind Navigation System Contributors",
    maintainer_email="maintainers@example.com",
    description="面向盲人用户的 ROS 2 室内导航、触觉引导和多模态感知系统",
    license="MIT",
    url="https://github.com/GuoGuo614/Blind-Navigation-System",
    project_urls={
        "Source": "https://github.com/GuoGuo614/Blind-Navigation-System",
        "Issues": "https://github.com/GuoGuo614/Blind-Navigation-System/issues",
    },
    entry_points={
        "console_scripts": [
            "blind-navigation = blind_navigation.navigation.main:main",
            "blind-perception = blind_navigation.perception.app:main",
        ]
    },
)
