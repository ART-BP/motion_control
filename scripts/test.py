#!/usr/bin/env python3
import os

motion_control_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
path = os.path.join(motion_control_dir, "temp", "control_data.txt")

if __name__ == "__main__":
    current_position = []
    lookahead_point = []
    control_cmd = []
    line_num = 1
    with open(path, "r") as f:
        for line in f:
            data = line.strip().split(" ")
            print(data)
            break
    