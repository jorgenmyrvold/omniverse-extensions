import matplotlib.pyplot as plt
import numpy as np
import json
import os

from preprocessor import Preprocessor, get_filepath

plt.rcParams["font.family"] = "serif"


FILEPATHS = {
    'drive_circle': f'{os.path.dirname(__file__)}/data/drive_data/drive_circle/drive_circle', 
    'drive_diagonal_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_diagonal_square/drive_diagonal_square', 
    'drive_rotate': f'{os.path.dirname(__file__)}/data/drive_data/drive_rotate/drive_rotate', 
    'drive_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_square/drive_square', 
}

class DrivePlotter():
    def __init__(self, dataset: str, title: str) -> None:
        self.dataset = dataset
        self.title = title
        self.datasets = {}
        self.fig, (self.ax1, self.ax2) = plt.subplots(1,2, figsize=(10,6))

        for i in range(6):
            if i<3:
                self.datasets[f'{dataset}_{i}'] = Preprocessor(get_filepath(FILEPATHS[dataset], id=i%3, reversed=False))
            else:
                self.datasets[f'{dataset}_reversed_{i%3}'] = Preprocessor(get_filepath(FILEPATHS[dataset], id=i%3, reversed=True))

    def plot_pose(self, 
                     dataset: Preprocessor, 
                     ax: plt.Axes,
                     line_label='Position', 
                     quiver_label='Heading',
                     interval=30,
                     line_color='C0',
                     quiver_color='C1') -> None:
        # Data is logged approximatly 60 times per second. Interval of 30 equals approximatly every 0.5 second
        dx, dy = np.cos(dataset.theta_rad + np.pi/2), np.sin(dataset.theta_rad + np.pi/2)
        ax.plot(dataset.y_pos, dataset.x_pos, label=line_label, color=line_color)
        ax.quiver(dataset.y_pos[::interval], 
                  dataset.x_pos[::interval], 
                  dx[::interval], 
                  dy[::interval],
                  scale=12, 
                  width=0.025, 
                  scale_units='xy',
                  color=quiver_color, 
                  label=quiver_label)

    def plot_ax(self, ax, dataset: str, reversed, plot_pose=False, interval=180):
        dataset_key = f'{dataset}{"_reversed_" if reversed else "_"}'
        if plot_pose:
            self.plot_pose(dataset=self.datasets[f'{dataset_key}0'],
                           ax=ax,
                           interval=interval,
                           line_label='1. Run',
                           quiver_label='',
                           line_color='C0',
                           quiver_color='C0')
            self.plot_pose(dataset=self.datasets[f'{dataset_key}1'],
                           ax=ax,
                           interval=interval,
                           line_label='2. Run',
                           quiver_label='',
                           line_color='C1',
                           quiver_color='C1')
            self.plot_pose(dataset=self.datasets[f'{dataset_key}2'],
                           ax=ax,
                           interval=interval,
                           line_label='3. Run',
                           quiver_label='',
                           line_color='C2',
                           quiver_color='C2')
        else: 
            ax.plot(self.datasets[f'{dataset_key}0'].y_pos, 
                    self.datasets[f'{dataset_key}0'].x_pos, 
                    label='1. Run')
            ax.plot(self.datasets[f'{dataset_key}1'].y_pos, 
                    self.datasets[f'{dataset_key}1'].x_pos, 
                    label='2. Run')
            ax.plot(self.datasets[f'{dataset_key}2'].y_pos, 
                    self.datasets[f'{dataset_key}2'].x_pos, 
                    label='3. Run')

        ax.scatter(self.datasets[f'{dataset_key}0'].y_pos[0], 
                self.datasets[f'{dataset_key}0'].x_pos[0], 
                label='Start',
                marker='D',
                color='C3',
                zorder=9)

        ax.set_title(f'{self.title} reversed' if reversed else self.title)
        ax.set_xlabel('Y position')
        if not reversed: ax.set_ylabel('X position')
        ax.set_aspect('equal')
        ax.legend()
        ax.grid(True, linestyle='--', color='gray')
    

    def plot_all(self):
        self.plot_ax(self.ax1, self.dataset, reversed=False, plot_pose=True)
        self.plot_ax(self.ax2, self.dataset, reversed=True, plot_pose=True)


def main():
    dp1 = DrivePlotter('drive_diagonal_square', 'Drive Diagonal Square')
    dp2 = DrivePlotter('drive_circle', 'Drive Circle')
    dp3 = DrivePlotter('drive_square', 'Drive Square')
    dp1.plot_all()
    dp2.plot_all()
    dp3.plot_all()
    plt.tight_layout()
    plt.show()
    return 


if __name__ == '__main__':
    main()

