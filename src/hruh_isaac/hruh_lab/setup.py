"""HRUH tasks for Isaac Lab 2.3 (install: isaac-python -m pip install -e .)."""
from setuptools import find_packages, setup

setup(
    name="hruh_lab",
    version="0.1.0",
    description="Isaac Lab RL tasks for the HRUH humanoid: locomotion, motion imitation, reaching",
    packages=find_packages(),
    python_requires=">=3.10",
    install_requires=["numpy"],
    zip_safe=False,
)
