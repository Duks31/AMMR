from setuptools import setup, find_packages
 
setup(
    name="cika_task_manager",
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chidubem",
    maintainer_email="chidubem@nileuniversity.edu.ng",
    description="Task manager for cika AMR — perception to Nav2 to arm sequencing",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task_manager_node = cika_task_manager_core.task_manager_node:main",
        ],
    },
)
 