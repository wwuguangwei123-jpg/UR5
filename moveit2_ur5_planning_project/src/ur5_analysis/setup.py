from setuptools import setup

package_name = "ur5_analysis"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="guangwei",
    maintainer_email="guangwei@example.com",
    description="Analysis scripts for UR5 MoveIt planning results.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "plot_trajectory.py = ur5_analysis.plot_trajectory:main",
            "fk_verify.py = ur5_analysis.fk_verify:main",
        ],
    },
)
