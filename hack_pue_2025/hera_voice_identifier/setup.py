from setuptools import find_packages, setup

package_name = 'hera_voice_identifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'speechbrain',
        'sounddevice',
        'numpy',
        'torch',
        'torchaudio',
    ],
    zip_safe=True,
    maintainer='santiago_gomez',
    maintainer_email='santiago_gomez@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grabador_referencia = hera_voice_identifier.grabador_referencia:main',
            'voz_identifier_node = hera_voice_identifier.voz_identifier_node:main'
        ],
    },
)
