from setuptools import find_packages, setup

package_name = 'row_following_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='milos',
    maintainer_email='nikolicmilos3610@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = row_following_py_pkg.image_subscriber:main',
            'randomize_node = row_following_py_pkg.randomize_node:main',
            'semantic_image_subscriber = row_following_py_pkg.semantic_image_subscriber:main',
            'main_node = row_following_py_pkg.main_node:main'

        ],
    },
)
