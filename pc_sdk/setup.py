from setuptools import find_packages, setup


setup(
    name="sanpo-st-app-control",
    version="1.0.0",
    packages=find_packages(),
    install_requires=["pyserial>=3.5"],
    python_requires=">=3.9",
)
