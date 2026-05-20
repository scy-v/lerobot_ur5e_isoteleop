from setuptools import setup, find_packages

setup(
    name="lerobot_teleoperator_ur5e",
    version="0.0.1",
    description="LeRobot teleoperator integration",
    author="Chenyu Su",
    author_email="suchenyu@mail.ustc.edu.cn",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=[
        "dynamixel_sdk",
        "pin==3.8.0",
        "scipy",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)
