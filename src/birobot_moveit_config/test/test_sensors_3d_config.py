import os
import yaml


def test_sensors_3d_yaml_configuration():
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
    sensors_3d_path = os.path.join(config_dir, 'sensors_3d.yaml')

    assert os.path.exists(sensors_3d_path), f"sensors_3d.yaml not found at {sensors_3d_path}"

    with open(sensors_3d_path, 'r') as f:
        config = yaml.safe_load(f)

    assert 'sensors' in config, "sensors list missing from sensors_3d.yaml"
    sensors = config['sensors']
    assert len(sensors) > 0, "No sensor plugins defined in sensors_3d.yaml"

    camera_sensor = sensors[0]
    assert camera_sensor.get('sensor_plugin') == 'occupancy_map_monitor/PointCloudOctomapUpdater'
    assert camera_sensor.get('point_cloud_topic') == '/birobot/depth_camera/points/points'
    assert camera_sensor.get('max_range') == 3.0
    assert camera_sensor.get('padding_offset') == 0.1
    assert camera_sensor.get('padding_scale') == 1.0
    assert camera_sensor.get('max_update_rate') == 1.0


def test_move_group_launch_perception_parameters():
    launch_dir = os.path.join(os.path.dirname(__file__), '..', 'launch')
    move_group_launch_path = os.path.join(launch_dir, 'move_group.launch.py')

    assert os.path.exists(move_group_launch_path), f"move_group.launch.py not found at {move_group_launch_path}"

    with open(move_group_launch_path, 'r') as f:
        content = f.read()

    assert 'sensors_3d.yaml' in content, "move_group.launch.py does not load sensors_3d.yaml"
    assert "'octomap_resolution': 0.05" in content or '0.05' in content, "Octomap resolution 0.05 missing"
    assert "'octomap_frame': 'world'" in content or "'world'" in content, "Octomap frame 'world' missing"


if __name__ == '__main__':
    test_sensors_3d_yaml_configuration()
    test_move_group_launch_perception_parameters()
