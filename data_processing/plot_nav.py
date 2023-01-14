import matplotlib.pyplot as plt
import os

from preprocessor import Preprocessor

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


def main():
    plotter = NavPlotter()
    plotter.plot_path()

    plt.tight_layout()
    plt.show()
    return 


if __name__ == '__main__':
    main()

