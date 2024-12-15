from setuptools import setup

package_name = "test_map_parse"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rocotics",
    maintainer_email="598062@stud.hvl.no",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_map_parse = test_map_parse.test_map_parse:main",
            "test_nav = test_map_parse.test_nav:main",
            "gotopoint = test_map_parse.gotopoint:main",
            "move_service_server = test_map_parse.move_service_server:main",
        ],
    },
)
