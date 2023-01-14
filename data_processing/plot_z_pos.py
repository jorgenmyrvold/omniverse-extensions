import matplotlib.pyplot as plt
import numpy as np
import os

from preprocessor import Preprocessor, get_filepath

plt.rcParams["font.family"] = "serif"


FILEPATHS = {
    'drive_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_square/drive_square', 
    'drive_diagonal_square': f'{os.path.dirname(__file__)}/data/drive_data/drive_diagonal_square/drive_diagonal_square', 
    'drive_circle': f'{os.path.dirname(__file__)}/data/drive_data/drive_circle/drive_circle', 
    'drive_rotate': f'{os.path.dirname(__file__)}/data/drive_data/drive_rotate/drive_rotate', 
}

class ZPosPlotter():
    def __init__(self, dataset) -> None:
        self.datasets = {}

        for key in FILEPATHS:
            for i in range(6):
                if i<3:
                    self.datasets[f'{key}_{i}'] = Preprocessor(get_filepath(FILEPATHS[key], id=i%3, reversed=False))
                else:
                    self.datasets[f'{key}_reversed_{i%3}'] = Preprocessor(get_filepath(FILEPATHS[key], id=i%3, reversed=True))
        
    def plot_ax(self, ax, dataset_key):
        for id in range(3):
            ax.plot(np.arange(len(self.datasets[f'{dataset_key}_{id}'].z_pos)),
                    self.datasets[f'{dataset_key}_{id}'].z_pos,
                    label=f'{id+1}. Run',
            )
            ax.set_xticks([])
            ax.text(1.0, 1.0, dataset_key, transform=ax.transAxes, ha='right', va='top')


    def plot(self):
        fig, axs = plt.subplots(8, 1, sharex=False)
        fig.suptitle('Z position of robot', y=0.935)
        self.plot_ax(axs[0], 'drive_square')
        self.plot_ax(axs[1], 'drive_square_reversed')
        self.plot_ax(axs[2], 'drive_diagonal_square')
        self.plot_ax(axs[3], 'drive_diagonal_square_reversed')
        self.plot_ax(axs[4], 'drive_circle')
        self.plot_ax(axs[5], 'drive_circle_reversed')
        self.plot_ax(axs[6], 'drive_rotate')
        self.plot_ax(axs[7], 'drive_rotate_reversed')


def main():
    d1 = ZPosPlotter('dataset_drive_square')
    d1.plot()
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1, hspace=0.1)
    plt.show()
    return 


if __name__ == '__main__':
    main()

