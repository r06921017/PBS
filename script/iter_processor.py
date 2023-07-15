#! /home/rdaneel/anaconda3/lib/python3.8
# -*- coding: UTF-8 -*-
"""Iteration processor"""

import matplotlib.pyplot as plt
import numpy as np

# Read files
TEXT_SIZE=28
LINE_WIDTH=2.5
DATA_PATH="/home/rdaneel/PBS/local/maze-32-32-2/"
file_names = ["RR_iteration_data.csv", "SR_iteration_data.csv"]
iter_data = {"sum_cost": dict(), "sum_conflicts": dict(), "ll_calls":dict(), "acc_ll_calls":dict()}
for fn in file_names:
    with open(DATA_PATH+fn, "r") as fin:
        line = fin.readline().rstrip("\n").split(",")
        line.pop(0)
        line.pop(-1)
        iter_data["sum_cost"][fn] = [int(_ele_) for _ele_ in line]

        line = fin.readline().rstrip("\n").split(",")
        line.pop(0)
        line.pop(-1)
        iter_data["sum_conflicts"][fn] = [int(_ele_) for _ele_ in line]

        line = fin.readline().rstrip("\n").split(",")
        line.pop(0)
        line.pop(-1)
        iter_data["ll_calls"][fn] = [int(_ele_) for _ele_ in line]

# Transfer LNS data format
LNS_FILE = "iter_stats-initLNS.csv"
iter_data["sum_cost"][LNS_FILE] = list()
iter_data["sum_conflicts"][LNS_FILE] = list()
iter_data["ll_calls"][LNS_FILE] = list()
iter_data["acc_ll_calls"][LNS_FILE] = list()
with open(DATA_PATH+LNS_FILE, encoding="UTF-8", mode="r") as fin:
    fin.readline()
    for line in fin.readlines():
        line = line.rstrip("\n").split(",")
        iter_data["sum_cost"][LNS_FILE].append(int(line[0]))
        iter_data["sum_conflicts"][LNS_FILE].append(int(line[1]))
        iter_data["ll_calls"][LNS_FILE].append(int(line[3]))
file_names.append(LNS_FILE)

for fn in file_names:
    curr_sum = 0
    iter_data["acc_ll_calls"][fn] = list()
    for _ele_ in iter_data["ll_calls"][fn]:
        curr_sum += _ele_
        iter_data["acc_ll_calls"][fn].append(curr_sum)

# TARGET_OBJECTIVE = "sum_cost"
TARGET_OBJECTIVE = "sum_conflicts"
max_size = max(len(iter_data[TARGET_OBJECTIVE][file_names[0]]),
               len(iter_data[TARGET_OBJECTIVE][file_names[1]]),
               len(iter_data[TARGET_OBJECTIVE][file_names[2]]))
for fn in file_names:
    if len(iter_data["sum_cost"][fn]) < max_size:
        for _ in range(0, max_size-len(iter_data["sum_cost"][fn])):
            iter_data["sum_cost"][fn].append(np.inf)
        assert len(iter_data["sum_cost"][fn]) == max_size
    if len(iter_data["sum_conflicts"][fn]) < max_size:
        for _ in range(0, max_size-len(iter_data["sum_conflicts"][fn])):
            iter_data["sum_conflicts"][fn].append(np.inf)
        assert len(iter_data["sum_conflicts"][fn]) == max_size
    if len(iter_data["ll_calls"][fn]) < max_size:
        for _ in range(0, max_size-len(iter_data["ll_calls"][fn])):
            iter_data["ll_calls"][fn].append(np.inf)
        assert len(iter_data["ll_calls"][fn]) == max_size
    if len(iter_data["acc_ll_calls"][fn]) < max_size:
        for _ in range(0, max_size-len(iter_data["acc_ll_calls"][fn])):
            iter_data["acc_ll_calls"][fn].append(np.inf)
        assert len(iter_data["acc_ll_calls"][fn]) == max_size

# Plot all the subplots on the figure
plt.close('all')
plt.rcParams.update({'font.size': TEXT_SIZE})
fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(12,9), dpi=80, facecolor='w', edgecolor='k')

SHOW_SIZE = max_size
axs[0].plot(range(1,SHOW_SIZE+1), iter_data[TARGET_OBJECTIVE][file_names[2]][:SHOW_SIZE],
            label="MAPF-LNS2", color="green", linewidth=LINE_WIDTH)
# axs[0].plot(range(1,SHOW_SIZE+1), iter_data[TARGET_OBJECTIVE][file_names[0]][:SHOW_SIZE],
#             label="GPBS(PE,TR,IC,RR)", color="blue", linewidth=LINE_WIDTH+1)
axs[0].plot(range(1,SHOW_SIZE+1), iter_data[TARGET_OBJECTIVE][file_names[1]][:SHOW_SIZE],
            label="GPBS(PE,TR,IC,SR)", color="red", linewidth=LINE_WIDTH)
axs[0].axhline(y = 0, color = 'grey', linewidth=0.5)

# num_conf = iter_data[TARGET_OBJECTIVE][file_names[2]][SHOW_SIZE-1]
# tmp_text = "MAPF-LNS2 timeout,\n" + str(num_conf) + " pairs left"
# axs[0].annotate(tmp_text, color="black", xy=(SHOW_SIZE, num_conf+1), xytext=(1200, 150),
#                 horizontalalignment='right',
#                 fontsize=TEXT_SIZE-2, arrowprops=dict(color="black", shrink=0.05))

axs[1].plot(range(1,SHOW_SIZE+1), iter_data["acc_ll_calls"][file_names[2]][:SHOW_SIZE],
            label="MAPF-LNS2", color="green", linewidth=LINE_WIDTH)
# axs[1].plot(range(1,SHOW_SIZE+1), iter_data["acc_ll_calls"][file_names[0]][:SHOW_SIZE],
#             label="GPBS(PE,TR,IC,RR)", color="blue", linewidth=LINE_WIDTH+1)
axs[1].plot(range(1,SHOW_SIZE+1), iter_data["acc_ll_calls"][file_names[1]][:SHOW_SIZE],
            label="GPBS(PE,TR,IC,SR)", color="red", linewidth=LINE_WIDTH)

axs[0].grid(axis="y")
axs[1].grid(axis="y")

axs[1].set_xlabel("Iteration")
Y_LABEL = ""
if TARGET_OBJECTIVE == "sum_conflicts":
    Y_LABEL = "Number of \nconflicting pairs"
if TARGET_OBJECTIVE == "sum_cost":
    Y_LABEL = "SOC"
axs[0].set_ylabel(Y_LABEL)
axs[1].set_ylabel("Cumulative\nnumber of calls")

leg = axs[0].legend(
    loc="upper center",
    bbox_to_anchor= (0.48, 1.25),
    borderpad=0.1,
    handletextpad=0.1,
    labelspacing=0.1,
    columnspacing=0.5,
    ncol=3,
    fontsize=TEXT_SIZE,
    handlelength=0.5,
)
axs[0].set_yscale('log')
axs[1].set_yscale('log')
plt.tight_layout()
plt.show()
