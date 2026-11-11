from setuptools import setup

package_name = 'fk_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FK Team',
    maintainer_email='dev@example.com',
    description='fk-system 試験用の偽の th',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_th = fk_sim.fake_th:main',
        ],
    },
)
