import yaml
import argparse
import os

def kalibr_to_ros2_yaml(kalibr_yaml_path, output_yaml_path, camera_name='camera'):
    with open(kalibr_yaml_path, 'r') as f:
        kalibr_data = yaml.safe_load(f)

    # Kalibr files can be nested under 'cam0', 'cam1', etc.
    cam_key = next(iter(kalibr_data.keys()))
    cam_data = kalibr_data[cam_key]

    fx, fy, cx, cy = cam_data['intrinsics']
    width, height = cam_data['resolution']
    distortion_model = cam_data['distortion_model']
    distortion_coeffs = cam_data['distortion_coeffs']

    # Ensure we have 5 coefficients for ROS
    if len(distortion_coeffs) == 4:
        distortion_coeffs.append(0.0)

    ros2_yaml = {
        'image_width': width,
        'image_height': height,
        'camera_name': camera_name,
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0
            ]
        },
        'distortion_model': 'plumb_bob',  # ROS uses plumb_bob for radtan
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': distortion_coeffs
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
        }
    }

    with open(output_yaml_path, 'w') as f:
        yaml.dump(ros2_yaml, f, sort_keys=False)

    print(f"Converted {kalibr_yaml_path} to {output_yaml_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert Kalibr YAML to ROS 2 camera_info YAML')
    parser.add_argument('kalibr_yaml', help='Path to Kalibr camera YAML file (e.g. camchain.yaml or cam0.yaml)')
    parser.add_argument('output_yaml', help='Path to output ROS 2-compatible YAML file')
    parser.add_argument('--camera-name', default='camera', help='Camera name for ROS (default: camera)')
    args = parser.parse_args()

    kalibr_to_ros2_yaml(args.kalibr_yaml, args.output_yaml, args.camera_name)

