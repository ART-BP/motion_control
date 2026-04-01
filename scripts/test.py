#!/usr/bin/env python3
import os
from matplotlib import pyplot as plt

motion_control_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
path = os.path.join(motion_control_dir, "temp", "control_data.txt")


def remove_comma(s):
    return s.replace(",", "")

if __name__ == "__main__":
    current_position = []
    lookahead_point = []
    control_cmd = []

    with open(path, "r") as f:
        for line in f:
            datas = line.strip().split(" ")
            data = [0.0] * 3
            for i,word in enumerate(datas):
                if word == "X:":
                    data[0] = float(remove_comma(datas[i+1]))
                elif word == "Y:":
                    data[1] = float(remove_comma(datas[i+1]))
                elif word == "Z:" or word == "Theta:":
                    data[2] = float(remove_comma(datas[i+1]))
                elif word == "Current":
                    current_position.append(data)
                elif word == "Lookahead":
                    lookahead_point.append(data)
                elif word == "Control":
                    control_cmd.append(data)

    
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP']
    plt.rcParams['axes.unicode_minus'] = False
    plt.plot([cmd[0] for cmd in current_position[10:20]], [cmd[1] for cmd in current_position[10:20]],
              marker='o', linestyle='-', linewidth=2)
    plt.plot([cmd[0] for cmd in lookahead_point[10:20]], [cmd[1] for cmd in lookahead_point[10:20]], 
             marker='x', linestyle='-', linewidth=2)

    plt.title("位置轨迹")
    plt.xlabel("X轴")
    plt.ylabel("Y轴")
    plt.grid(True)
    plt.show()
    