import matplotlib.pyplot as plt
import numpy as np
import json
import os

plt.rcParams["font.family"] = "serif"

class Preprocessor:
    def __init__(self, filepath):
        self.filepath = filepath
        self.raw_data = None

    def read_json(self):
        with open(self.filepath) as f:
            self.raw_data = json.load(f)
            self.raw_data = self.raw_data['Isaac Sim Data']
    
    def process_data(self):
        for data_point in self.raw_data:
            data_point['data']['base_link_transform_matrix'] = np.array(data_point['data']['base_link_transform_matrix'])


    def extract_plot_data(self):
        self.time_data = [d['current_time'] for d in self.raw_data]
        self.x_pos = np.array([d['data']['base_link_transform_matrix'][-1][0] for d in self.raw_data])
        self.y_pos = np.array([d['data']['base_link_transform_matrix'][-1][1] for d in self.raw_data])
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
        
    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.time_data, self.all_wheel_vel)
        ax.set_title("Title")

        fig1, ax1 = plt.subplots()
        ax1.plot(self.x_pos, self.y_pos)

        # fig, ax = plt.subplots()
        # ax.plot(self.time_data, self.all_joint_angles)
        # ax.legend()
        # ax.set_title('All Joints')
        
        plt.show()

def main():
    filepath = f'{os.path.dirname(__file__)}/data/nav_warehouse_with_forklifts.json'
    # filepath = f'{os.path.dirname(__file__)}/important_data/slam_warehouse_with_forklifts.json'
    p = Preprocessor(filepath)
    p.read_json()
    p.process_data()
    p.extract_plot_data()
    p.plot()
    # print(p.x_pos)
    return 



if __name__ == '__main__':
    main()

