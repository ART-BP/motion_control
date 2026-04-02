#!/usr/bin/env python3
import os
import signal
import sys
from matplotlib import pyplot as plt

motion_control_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
path = os.path.join(motion_control_dir, "temp", "control_data.txt")

exit_requested = False


def remove_comma(s):
    return s.replace(",", "")

def request_exit(*_):
    global exit_requested
    exit_requested = True
    plt.close("all")

def install_signal_handlers():
    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)

def bind_fast_exit_keys(fig):
    def on_key(event):
        if event.key in ("q", "escape"):
            request_exit()

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", request_exit)

if __name__ == "__main__":
    current_position = []
    lookahead_point = []
    control_cmd = []
    install_signal_handlers()

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
    plt.plot([cmd[0] for cmd in current_position[0:40]], [cmd[1] for cmd in current_position[0:40]],
              marker='o', linestyle='-', linewidth=2)

    plt.plot([cmd[0] for cmd in lookahead_point[0:40]], [cmd[1] for cmd in lookahead_point[0:40]], 
             marker='x', linestyle='-', linewidth=2)
    plt.legend(["当前位置", "前瞻点"])
    plt.title("位置轨迹")
    plt.xlabel("X轴")
    plt.ylabel("Y轴")
    plt.grid(True)
    fig = plt.gcf()
    bind_fast_exit_keys(fig)
    print("按 q / Esc，或关闭图窗，或 Ctrl+C 退出。")
    plt.show(block=False)
    try:
        while plt.fignum_exists(fig.number) and not exit_requested:
            plt.pause(0.1)
    except KeyboardInterrupt:
        request_exit()
    finally:
        plt.close("all")
        sys.exit(0)
    
