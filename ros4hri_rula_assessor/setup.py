from setuptools import find_packages, setup

package_name = 'ros4hri_rula_assessor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tomislav Dimcic',
    maintainer_email='toma.dimcic@gmail.com',
    description='ARISE Tele-Craft Module B: real-time RULA from REP-155 bodies',
    license='LGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'assessor = ros4hri_rula_assessor.assessor_node:main',
            'body_sim = ros4hri_rula_assessor.body_sim:main',
            'openxr_body = ros4hri_rula_assessor.openxr_body:main',
            'prompter = ros4hri_rula_assessor.prompter:main'
        ],
    },
)
