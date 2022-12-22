import matplotlib.pyplot as plt
import numpy as np
import json
import os

from preprocessor import Preprocessor

plt.rcParams["font.family"] = "serif"
# plt.rcParams['font.family'] = 'CMU Serif'
# plt.rcParams['font.family'] = 'CMU Serif'
# plt.rcParams['font.sans-serif'] = ['CMU Serif']

FILEPATHS = {
    'drive_circle': f'{os.path.dirname(__file__)}/data/drive_data/drive_circle/drive_circle', 
    'drive_diagonal_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_diagonal_square/drive_diagonal_square', 
    'drive_rotate': f'{os.path.dirname(__file__)}/data/drive_data/drive_rotate/drive_rotate', 
    'drive_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_square/drive_square', 
}

def get_filepath(base_path, id, reversed):
    if reversed:
        return f'{base_path}_reversed_{id}.json'
    return f'{base_path}_{id}.json'


class DrivePlotter():
    def __init__(self, d1, d2, d3, dr1, dr2, dr3) -> None:
        # d1, d2, d3 are the preprocessed datasets of the class preprocessor
        self.d1 = d1
        self.d2 = d2
        self.d3 = d3
        self.dr1 = dr1
        self.dr2 = dr2
        self.dr3 = dr3
    

    def ax_plot_drive_data(self, dataset, ax):
        fig, ax = plt.subplots()
        if reversed:
            ax.plot(self.dr1.y_pos, self.dr1.x_pos, label='1. Example')
            ax.plot(self.dr2.y_pos, self.dr2.x_pos, label='2. Example')
            ax.plot(self.dr3.y_pos, self.dr3.x_pos, label='3. Example')
        else:
            ax.plot(self.d1.y_pos, self.d1.x_pos, label='1. Example')
            ax.plot(self.d2.y_pos, self.d2.x_pos, label='2. Example')
            ax.plot(self.d3.y_pos, self.d3.x_pos, label='3. Example')

        ax.set_title('Drive square')
        ax.set_xlabel('Y position')
        ax.set_ylabel('X position')
        ax.legend()
        ax.grid(True, linestyle='--', color='gray')
        ax.set_aspect('equal')


    def ax_plot_pose(self, 
                     dataset: Preprocessor, 
                     ax: plt.Axes,
                     line_label='Position', 
                     quiver_label='Heading',
                     interval=30) -> None:
        # Data is logged approximatly 60 times per second. Interval of 30 equals approximatly every 0.5 second
        dx, dy = np.cos(dataset.theta_rad), np.sin(dataset.theta_rad)
        ax.plot(dataset.y_pos, dataset.x_pos, label=line_label, color='C0')
        ax.quiver(dataset.y_pos[::interval], 
                  dataset.x_pos[::interval], 
                  dx[::interval], 
                  dy[::interval],
                  scale=12, 
                  width=0.025, 
                  scale_units='xy',
                  color='C1', 
                  label=quiver_label)

    def plot_multiple_poses(self):
        fig, (ax1, ax2) = plt.subplots(1, 2)
        self.ax_plot_pose(self.d1, ax1)
        self.ax_plot_pose(self.d2, ax2)

        ax1.set_title('Drive Square 1')
        ax1.set_xlabel('Y position')
        ax1.set_ylabel('X position')
        ax1.legend()
        ax1.grid(True, linestyle='--', color='gray')
        ax1.set_aspect('equal', adjustable='box')

        ax2.set_title('Drive Square 2')
        ax2.set_xlabel('Y position')
        ax2.set_ylabel('X position')
        ax2.legend()
        ax2.grid(True, linestyle='--', color='gray')
        ax2.set_aspect('equal', adjustable='box')


    def plot_drive_data_and_reversed(self):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 7))

        ax1.plot(self.dr1.y_pos, self.dr1.x_pos, label='1. Example')
        ax1.plot(self.dr2.y_pos, self.dr2.x_pos, label='2. Example')
        ax1.plot(self.dr3.y_pos, self.dr3.x_pos, label='3. Example')
        ax1.set_title('Drive square Reversed')
        ax1.set_xlabel('Y position')
        ax1.set_ylabel('X position')
        ax1.legend()
        ax1.grid(True, linestyle='--', color='gray')
        ax1.set_aspect('equal')
        
        ax2.plot(self.d1.y_pos, self.d1.x_pos, label='1. Example')
        ax2.plot(self.d2.y_pos, self.d2.x_pos, label='2. Example')
        ax2.plot(self.d3.y_pos, self.d3.x_pos, label='3. Example')
        ax2.set_title('Drive square')
        ax2.set_xlabel('Y position')
        ax2.set_ylabel('X position')
        ax2.legend()
        ax2.grid(True, linestyle='--', color='gray')
        ax2.set_aspect('equal')

class PlotBagData:
    def __init__(self, filename):
        return


def main():

    square_1 = Preprocessor(get_filepath(FILEPATHS['drive_square'], id=0, reversed=False))
    square_2 = Preprocessor(get_filepath(FILEPATHS['drive_square'], id=1, reversed=False))
    square_3 = Preprocessor(get_filepath(FILEPATHS['drive_square'], id=2, reversed=False))
    square_r_1 = Preprocessor(get_filepath(FILEPATHS['drive_square'], id=0, reversed=True))
    square_r_2 = Preprocessor(get_filepath(FILEPATHS['drive_square'], id=1, reversed=True))
    square_r_3 = Preprocessor(get_filepath(FILEPATHS['drive_square'], id=2, reversed=True))
    
    dp = DrivePlotter(square_1, square_2, square_3, square_r_1, square_r_2, square_r_3)
    # dp.plot_drive_data(reversed=False)
    # dp.plot_drive_data_and_reversed()

    dp.plot_multiple_poses()

    plt.show()


    return 


if __name__ == '__main__':
    main()

