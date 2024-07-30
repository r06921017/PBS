#! /home/rdaneel/anaconda3/lib/python3.8
# -*- coding: UTF-8 -*-
""" Highway Heuristic generator
"""

from typing import List, Dict, Tuple
from enum import Enum
from copy import deepcopy

INT_MAX = 2**31 - 1
DBL_MAX = 1.79769e+308

class Direction(Enum):
    RIGHT = 0
    UP = 1
    LEFT = 2
    DOWN = 3
    WAIT = 4


class HWYHeuristicGenerator:
    """Class for loading paths to compute highway heuristic
    """
    def __init__(self, map_file:str, paths_file:str) -> None:
        self.subopt = 1.05

        self.height = -1
        self.width = -1
        self.env_map:List[List[bool]] = []

        self.paths:Dict[int, List[int]] = {}
        self.goals:List[int] = []  # agent index to its goal location
        self.edge_flow:Dict[Tuple[int], int] = {}
        self.hwy:List[Dict[Direction, float]] = []

        # Load map
        with open(map_file, mode="r", encoding="utf-8") as fin:
            fin.readline()  # ignore type
            self.height = int(fin.readline().strip().split(' ')[1])
            self.width  = int(fin.readline().strip().split(' ')[1])
            fin.readline()  # ignore 'map' line
            for line in fin.readlines():
                out_line: List[bool] = []
                for word in list(line.strip()):
                    if word == '.':
                        out_line.append(True)
                    else:
                        out_line.append(False)
                assert len(out_line) == self.width
                self.env_map.append(out_line)
        assert len(self.env_map) == self.height

        # Load paths
        with open(paths_file, mode="r", encoding="utf-8") as fin:
            for line in fin.readlines():
                line = line.strip().split(",")
                ag_id = int(line[0])
                self.paths[ag_id] = [int(line[t]) for t in range(1, len(line))]
                self.goals.append(int(line[-1]))
                assert len(self.goals) == (ag_id + 1)

        # Compute edge flow
        for _, path in self.paths:
            for t in range(len(path)-1):
                cur_edge:Tuple[int] = (path[t], path[t+1])
                if cur_edge not in self.edge_flow:
                    self.edge_flow[cur_edge] = 0
                self.edge_flow[cur_edge] += 1

    def compute_hwy(self) -> None:
        """ Compute highways based on the edge flow and current path of the agent
        """
        # Traverse all the five actions from each location
        for rid, row in self.env_map:
            for cid, is_cur_free in row:
                cur_idx = rid * self.width + cid
                cur_weights:Dict[Direction, int] = {  # initialize to infinity
                    Direction.RIGHT: DBL_MAX,
                    Direction.UP: DBL_MAX,
                    Direction.LEFT: DBL_MAX,
                    Direction.DOWN: DBL_MAX,
                    Direction.WAIT: DBL_MAX
                }
                if not is_cur_free:  # skip if the current location is an obstacle
                    self.hwy.append(cur_weights)
                    continue

                nxt_locations:Dict[Direction, Tuple[int]] = {
                    Direction.RIGHT: (rid, cid+1),
                    Direction.UP: (rid-1, cid),
                    Direction.LEFT: (rid, cid-1),
                    Direction.DOWN: (rid+1, cid)
                }
                for nxt_dir, nxt_loc in nxt_locations.items():  # skip the wait action
                    if not (
                        0 <= nxt_loc[0] < self.height and
                        0 <= nxt_loc[1] < self.width and
                        self.env_map[nxt_loc[0]][nxt_loc[1]]
                    ):
                        continue

                    cur_weights[nxt_dir] = 1.0
                    nxt_idx = nxt_loc[0] * self.width + nxt_loc[1]
                    if (cur_idx, nxt_idx) in self.edge_flow:
                        cur_weights[nxt_dir] = self.edge_flow[(cur_idx, nxt_idx)]

        # For each agent, benefit the edges on its path and output its own HWY heuristic
        output_hwy:Dict[int,List[Dict[Direction,float]]] = {}
        for ag, path in enumerate(self.paths):
            cur_hwy = deepcopy(self.hwy)
            for t in range(len(path) - 1):
                cur_idx = path[t]
                nxt_idx = path[t+1]
                cur_direction = nxt_idx - cur_idx
                if cur_direction == 1:  # Direction.RIGHT
                    cur_hwy[cur_idx][Direction.RIGHT] = 1
                elif cur_direction == -self.width:
                    cur_hwy[cur_idx][Direction.UP] = 1
                elif cur_direction == -1:
                    cur_hwy[cur_idx][Direction.LEFT] = 1
                elif cur_direction == self.width:
                    cur_hwy[cur_idx][Direction.DOWN] = 1
            for other_ag, goal in enumerate(self.goals):
                if ag == other_ag:
                    continue
                for dr in Direction:
                    if dr.value == Direction.WAIT:
                        continue
                    cur_hwy[goal][dr.value] *= self.subopt
            output_hwy[ag] = cur_hwy

        # Save the highway
        


if __name__ == "__main__":
    CUR_MAP = "/home/rdaneel/mapf_benchmark/mapf-map/random-32-32-20.map"
    CUR_PATH = "/home/rdaneel/GPBS/local/random-32-32-20_random-12_ag-100_GPBS.csv"
    CUR_HWY = "./out.heu"

    generator = HWYHeuristicGenerator(CUR_MAP, CUR_PATH)
    generator.compute_hwy()
    generator.save_hwy()
