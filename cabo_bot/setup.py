from setuptools import setup, find_packages

package_name = 'cabo_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='scripts'),
    package_dir={'': 'scripts'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cabo_launch.py']),
        ('lib/' + package_name, [
            
            'scripts/cabo_bot/cabo_motion_node.py',
            'scripts/cabo_bot/cabo_state_node.py',
            'scripts/cabo_bot/cabo_vision_node.py',
            'scripts/cabo_bot/cabo_vision_display.py',
            'scripts/cabo_bot/cabo_voice_node.py',

            'scripts/cabo_bot/cabo_motion_diablo.py',

            'scripts/cabo_bot/cabo_voice_silent_test_node.py',
            'scripts/cabo_bot/cabo_vision_source.py',

        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clara',
    maintainer_email='clara@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
 
    entry_points={
        'console_scripts': [

            'cabo_motion_diablo = cabo_bot.cabo_motion_diablo:main',
            
            'cabo_motion_node = cabo_bot.cabo_motion_node:main',
            'cabo_state_node = cabo_bot.cabo_state_node:main',
            'cabo_vision_node = cabo_bot.cabo_vision_node:main',
            'cabo_vision_display = cabo_bot.cabo_vision_display:main',
            'cabo_voice_node = cabo_bot.cabo_voice_node:main',

            'cabo_voice_silent_test_node = cabo_bot.cabo_voice_silent_test_node:main',
            'cabo_vision_source = cabo_bot.cabo_vision_source:main',

     
        ],
    }

)
