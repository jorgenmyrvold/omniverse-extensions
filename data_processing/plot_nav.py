import matplotlib.pyplot as plt
import numpy as np
import json
import os

from preprocessor import Preprocessor, get_filepath

plt.rcParams["font.family"] = "serif"


FILEPATHS = {
    'diff_nav': f'{os.path.dirname(__file__)}/data/important_data/nav_warehouse_with_forklifts_diff.json', 
    'omni_nav': f'{os.path.dirname(__file__)}/data/important_data/nav_warehouse_with_forklifts_omni.json', 
}


class NavPlotter():
    def __init__(self) -> None:
        self.diff_dataset = Preprocessor(FILEPATHS['diff_nav'])
        self.omni_dataset = Preprocessor(FILEPATHS['omni_nav'])
        self.offset = 4.5  # Seconds. Used to align time differences between datasets


    def plot_path(self):
        fig, ax = plt.subplots()
        ax.plot(self.diff_dataset.x_pos, self.diff_dataset.y_pos, label='Differential')
        ax.plot(self.omni_dataset.x_pos, self.omni_dataset.y_pos, label='Omnidirectional')
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', color='gray')
        ax.set_title('Navigated path')
        ax.set_xlabel('X position [m]')
        ax.set_ylabel('Y position [m]')
        ax.legend()


    def plot_xyz(self):
        # fig, [ax1, ax2, ax3] = plt.subplots(3, 1, sharex=True)
        fig, ax3 = plt.subplots()
        # fig.suptitle(f'X and Y position and rotation of the robot', y=0.94)
        ax3.set_title(f'X and Y position and rotation of the robot')
        
        # ax1.plot(self.diff_dataset.time, self.diff_dataset.x_pos, label='Differential')
        # ax1.plot(self.omni_dataset.time - np.full(self.omni_dataset.time.shape, self.offset), self.omni_dataset.x_pos, label='Omnidirectional')
        # ax1.set_ylabel('X position [m]')
        # ax1.legend()
        # ax2.plot(self.diff_dataset.time, self.diff_dataset.y_pos, label='Differential')
        # ax2.plot(self.omni_dataset.time - np.full(self.omni_dataset.time.shape, self.offset), self.omni_dataset.y_pos, label='Omnidirectional')
        # ax2.set_ylabel('Y position [m]')
        # ax2.legend()
        ax3.plot(self.diff_dataset.time, self.diff_dataset.theta_deg, label='Differential')
        ax3.plot(self.omni_dataset.time - np.full(self.omni_dataset.time.shape, self.offset), self.omni_dataset.theta_deg, label='Omnidirectional')
        ax3.axhline(y=180, color='grey', linestyle='--')
        ax3.axhline(y=-180, color='grey', linestyle='--')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Rotation [deg]')
        ax3.legend()

def main():
    plotter = NavPlotter()
    plotter.plot_path()
    plotter.plot_xyz()

    plt.tight_layout()
    plt.show()
    return 


if __name__ == '__main__':
    main()

