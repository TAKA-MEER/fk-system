from setuptools import setup

package_name = 'fk_auto'

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
    description='fk-system 自律測定の移動',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_runner = fk_auto.auto_runner:main',
        ],
    },
)
