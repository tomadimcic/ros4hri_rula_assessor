# Copyright 2026 Entanglenet GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
from setuptools import find_packages, setup

package_name = "ros4hri_rula_assessor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tomislav Dimcic",
    maintainer_email="toma.dimcic@gmail.com",
    description="ARISE Tele-Craft Module B: real-time RULA from REP-155 bodies",
    license="LGPL-3.0-or-later",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "assessor = ros4hri_rula_assessor.assessor_node:main",
            "body_sim = ros4hri_rula_assessor.body_sim:main",
            "openxr_body = ros4hri_rula_assessor.openxr_body:main",
            "prompter = ros4hri_rula_assessor.prompter:main"
        ],
    },
)
