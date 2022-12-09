import matplotlib.pyplot as plt
import numpy as np
import json
import os


class Preprocessor:
    def __init__(self, filepath):
        self.filepath = filepath
        self.raw_data = None

    def read_json(self):
        with open(self.filepath) as f:
            self.raw_data = json.load(f)
            self.raw_data = self.raw_data['Isaac Sim Data']


def main():
    filepath = f'{os.path.dirname(__file__)}/data/output_data.json'
    p = Preprocessor(filepath)
    p.read_json()
    for data in p.raw_data:
        print(data)
    return 


if __name__ == '__main__':
    main()