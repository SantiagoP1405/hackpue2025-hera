from setuptools import setup

package_name = 'voz_identifier'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
    maintainer='Dario',
    maintainer_email='dario5feb6b@outlook.com',
    description='Identifica personas por su voz usando SpeechBrain',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grabador_referencia = voz_identifier.grabador_referencia:main',
            'voz_identifier_node = voz_identifier.voz_identifier_node:main'
        ],
    },
)