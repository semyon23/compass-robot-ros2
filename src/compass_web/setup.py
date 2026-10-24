from setuptools import setup

package_name = 'compass_web'

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
    maintainer='semen',
    maintainer_email='semen@example.com',
    description='Веб-сервер Compass на роботе',
    license='Proprietary',
    entry_points={
        'console_scripts': ['compass_web = compass_web.server:main'],
    },
)
