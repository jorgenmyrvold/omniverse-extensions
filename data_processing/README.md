# Data processing
This directory contains data from the experiments and sripts to process and plot the data.

The most important file is [`preprocessor.py`](./preprocessor.py) which generates a preprocessor class that makes all data arrays from the json data files located in the [data directory](./data/) available as numpy arrays. All other files just plots the data from the preprocessor.

## Data directory
The [data directory](./data/) contains data from each experiment
- [`drive_data`](./data/drive_data/) contains files from each of the experiments for evaluating the robots drive capabilities.
- [`important_data`](./data/important_data/) contains data from navigation and slam experiments.
- [ros2bag_json](./data/ros2bag_json/) is not used and contains bad data converted from bag files to json. Not very useful.