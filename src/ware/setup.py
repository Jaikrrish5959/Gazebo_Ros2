from setuptools import find_packages, setup

package_name = 'ware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/warehouse.launch.py']),
        ('share/' + package_name + '/config', ['config/bridge.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/tugbot.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/warehouse.rviz']),
        ('share/' + package_name + '/models/tugbot', ['models/tugbot/model.config', 'models/tugbot/model.sdf']),
        ('share/' + package_name + '/models/tugbot/meshes', ['models/tugbot/meshes/VLP16_scan.dae', 'models/tugbot/meshes/VLP16_base_1.dae', 'models/tugbot/meshes/VLP16_base_2.dae']),
        ('share/' + package_name + '/models/tugbot/meshes/base', ['models/tugbot/meshes/base/logo-new.png', 'models/tugbot/meshes/base/movai_logo.dae', 'models/tugbot/meshes/base/movai-logo.png', 'models/tugbot/meshes/base/tugbot_simp.dae', 'models/tugbot/meshes/base/tugbot_simp.stl']),
        ('share/' + package_name + '/models/tugbot/thumbnails', ['models/tugbot/thumbnails/1.png']),
        ('share/' + package_name + '/models/tugbot/meshes/gripper2', ['models/tugbot/meshes/gripper2/gripper2.dae', 'models/tugbot/meshes/gripper2/gripper_hand.stl']),
        ('share/' + package_name + '/models/tugbot/meshes/light_link', ['models/tugbot/meshes/light_link/light_led.stl', 'models/tugbot/meshes/light_link/light.stl']),
        ('share/' + package_name + '/models/tugbot/meshes/wheel', ['models/tugbot/meshes/wheel/wheel.dae']),
        ('share/' + package_name + '/models/warehouse', ['models/warehouse/model.config', 'models/warehouse/model.sdf']),
        ('share/' + package_name + '/models/warehouse/meshes', ['models/warehouse/meshes/Terrazzo005_1K_Color.jpg', 'models/warehouse/meshes/Tape001_1K_Color.png', 'models/warehouse/meshes/warehouse.dae', 'models/warehouse/meshes/Rough_Square_Concrete_Block.jpg', 'models/warehouse/meshes/Asphalt010_1K_Color.jpg', 'models/warehouse/meshes/ground.png', 'models/warehouse/meshes/warehouse_colision.stl']),
        ('share/' + package_name + '/models/warehouse/thumbnails', ['models/warehouse/thumbnails/1.png', 'models/warehouse/thumbnails/2.png', 'models/warehouse/thumbnails/3.png', 'models/warehouse/thumbnails/4.png']),
        ('share/' + package_name + '/models/tugbot_warehouse', ['models/tugbot_warehouse/model.config', 'models/tugbot_warehouse/model.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krish',
    maintainer_email='krishanth.k2006@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = ware.controller:main',
        ],
    },
)
