from setuptools import setup

package_name = 'carrito_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uriel',
    maintainer_email='aguilaruriel368@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'nodo_imagen=carrito_pkg.imagen_hough:main',
	'nodo_centroide=carrito_pkg.imagen_centroide:main',
	'nodo_pub_imagen=carrito_pkg.nodo_imagen:main',
	'nodo_principal=carrito_pkg.nodo_principal:main',
        ],
    },
)
