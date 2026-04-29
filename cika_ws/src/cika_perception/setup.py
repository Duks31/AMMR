from setuptools import setup, find_packages

setup(
    name="cika_perception",
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chidubem",
    maintainer_email="chidubem@nileuniversity.edu.ng",
    description="Perception package for cika AMR",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "detector_node   = cika_perception.cika_core.detector_node:main",
            "classifier_node = cika_perception.cika_core.classifier_node:main",
            "camera_node = cika_perception.cika_core.camera_node:main",
        ],
    },
)
