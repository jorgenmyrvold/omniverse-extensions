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
        self.fl_vel =np.array([d['data']['wheel_velocity_fl'] for d in self.raw_data])
        self.fr_vel =np.array([d['data']['wheel_velocity_fr'] for d in self.raw_data])
        self.rl_vel =np.array([d['data']['wheel_velocity_rl'] for d in self.raw_data])
        self.rr_vel =np.array([d['data']['wheel_velocity_rr'] for d in self.raw_data])
        self.all_wheel_vel = np.vstack([self.fl_vel, self.fr_vel, self.rl_vel, self.rr_vel]).T
        
    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.time_data, self.all_wheel_vel)
        ax.set_title("Title")

        fig, ax = plt.subplots()
        ax.plot(self.x_pos, self.y_pos)
        plt.show()


def main():
    filepath = f'{os.path.dirname(__file__)}/data/output_data11.json'
    p = Preprocessor(filepath)
    p.read_json()
    p.process_data()
    p.extract_plot_data()
    p.plot()
    return 


if __name__ == '__main__':
    main()

