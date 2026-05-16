from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'catchrobo_pc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # static ファイルをインストール先にコピー
        (os.path.join('lib', package_name, 'static', 'css'),
         glob('catchrobo_pc/static/css/*')),
        (os.path.join('lib', package_name, 'static', 'js'),
         glob('catchrobo_pc/static/js/*')),
        (os.path.join('lib', package_name, 'static'),
         glob('catchrobo_pc/static/*.html')),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'pyserial'],
    zip_safe=True,
    maintainer='Altair',
    maintainer_email='altair@example.com',
    description='Catchrobo 2026 PC側パッケージ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gui_node = catchrobo_pc.web_gui_node:main',
            'debug_node = catchrobo_pc.debug_node:main',
        ],
    },
)
