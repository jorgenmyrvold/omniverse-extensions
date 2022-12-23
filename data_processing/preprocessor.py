import os
import json
import numpy as np


FILEPATHS = {
    'drive_circle': f'{os.path.dirname(__file__)}/data/drive_data/drive_circle/drive_circle', 
    'drive_diagonal_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_diagonal_square/drive_diagonal_square', 
    'drive_rotate': f'{os.path.dirname(__file__)}/data/drive_data/drive_rotate/drive_rotate', 
    'drive_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_square/drive_square', 
    'nav_omni': f'{os.path.dirname(__file__)}data/ros2bag_json/omni_nav_warehouse_with_forklifts.json',
    'nav_diff': f'{os.path.dirname(__file__)}data/ros2bag_json/diff_nav_warehouse_with_forklifts.json',
    'slam_fw': f'{os.path.dirname(__file__)}data/ros2bag_json/slam_full_warehouse.json',
    'slam_wwf': f'{os.path.dirname(__file__)}data/ros2bag_json/slam_warehouse_with_forklifts.json',
}

def get_filepath(base_path, id, reversed):
    if reversed:
        return f'{base_path}_reversed_{id}.json'
    return f'{base_path}_{id}.json'

class Preprocessor:
    def __init__(self, filepath):
        self.filepath = filepath
        self.read_json()
        self.process_data()
        self.extract_plot_data()
        self.rot_matrix_to_z_rotation()


    def read_json(self):
        with open(self.filepath, 'r') as f:
            self.raw_data = json.load(f)
            self.raw_data = self.raw_data['Isaac Sim Data']
    
    def process_data(self):
        for data_point in self.raw_data:
            data_point['data']['base_link_transform_matrix'] = np.array(data_point['data']['base_link_transform_matrix'])

    def rot_matrix_to_z_rotation(self):
        sines = self.rot_matrix[:,1,0]
        cosines = self.rot_matrix[:,0,0]
        self.theta_rad = np.arctan2(sines, cosines)
        self.theta_deg = self.theta_rad * np.pi/180
            
    def extract_plot_data(self):
        self.time = [d['current_time'] for d in self.raw_data]
        self.x_pos = np.array([d['data']['base_link_transform_matrix'][-1][0] for d in self.raw_data])
        self.y_pos = np.array([d['data']['base_link_transform_matrix'][-1][1] for d in self.raw_data])
        self.z_pos = np.array([d['data']['base_link_transform_matrix'][-1][2] for d in self.raw_data])
        self.rot_matrix = np.array([d['data']['base_link_transform_matrix'][:3][:3] for d in self.raw_data])

        self.fl_vel = np.array([d['data']['wheel_velocity_fl'] for d in self.raw_data])
        self.fr_vel = np.array([d['data']['wheel_velocity_fr'] for d in self.raw_data])
        self.rl_vel = np.array([d['data']['wheel_velocity_rl'] for d in self.raw_data])
        self.rr_vel = np.array([d['data']['wheel_velocity_rr'] for d in self.raw_data])
        self.all_wheel_vel = np.vstack([self.fl_vel, self.fr_vel, self.rl_vel, self.rr_vel]).T
        
        if 'kmr_joint_1_pos' in self.raw_data[0]['data']:
            self.joint_1_angles = np.array([d['data']['kmr_joint_1_pos'] for d in self.raw_data])
            self.joint_2_angles = np.array([d['data']['kmr_joint_2_pos'] for d in self.raw_data])
            self.joint_3_angles = np.array([d['data']['kmr_joint_3_pos'] for d in self.raw_data])
            self.joint_4_angles = np.array([d['data']['kmr_joint_4_pos'] for d in self.raw_data])
            self.joint_5_angles = np.array([d['data']['kmr_joint_5_pos'] for d in self.raw_data])
            self.joint_6_angles = np.array([d['data']['kmr_joint_6_pos'] for d in self.raw_data])
            self.joint_7_angles = np.array([d['data']['kmr_joint_7_pos'] for d in self.raw_data])
            self.all_joint_angles = np.vstack([self.joint_1_angles, self.joint_2_angles, self.joint_3_angles, self.joint_4_angles, self.joint_5_angles, self.joint_6_angles, self.joint_7_angles]).T


class BagPreprocessor:
    def __init__(self, filepath):
        self.filepath = filepath
        self.extract_data()
    
    def extract_plot_data(self):
        with open(self.filepath, 'r') as f:
            self.raw_data = json.load(f)

        self.x_pos = np.array([d['x_pos'] for d in self.raw_data])
        self.y_pos = np.array([d['y_pos'] for d in self.raw_data])
        self.theta_rad = np.array([d['theta'] for d in self.raw_data])
        self.theta_deg = self.theta_rad * np.pi/180
