from setuptools import setup, find_packages

setup(
    name="g1_project",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "isaaclab",  # Assuming this is available in the env
    ],
)
