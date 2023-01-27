from setuptools import setup

package_name = 'rqt_scora'
setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/RqtScora.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('lib/' + package_name, ['scripts/rqt_scora']),
        ('lib/' + package_name, ['scripts/read'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Sebastian Muñoz',
    maintainer='Sebastian Muñoz',
    maintainer_email='semurcdc@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'rqt_scora provides a GUI plugin for the control of the robotic arm scora-er 14'
    ),
    license='BSD',
    scripts=['scripts/rqt_scora','scripts/read'],
)
