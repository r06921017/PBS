#! /home/rdaneel/anaconda3/lib/python3.8
# -*- coding: UTF-8 -*-
"""Data processor"""

import logging
import os
import sys
import argparse
from typing import Dict, List, Tuple
import yaml
import matplotlib.pyplot as plt
import util
import numpy as np


class DataProcessor:
    def __init__(self, in_config) -> None:
        self.config: Dict = {}
        config_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), in_config)
        with open(config_dir, encoding='utf-8', mode='r') as fin:
            self.config = yaml.load(fin, Loader=yaml.FullLoader)

        # Plot parameters
        self.max_x_num = 5  # on the x-axis
        self.fig_size:Tuple[int,int] = (self.config['figure_width'], self.config['figure_height'])
        self.marker_size:int = self.config['marker_size'] # 25
        self.line_width:float = self.config['line_width']  # 4.0
        self.mark_width:float = self.config['marker_width']  # 4.0
        self.text_size:int = self.config['text_size']
        self.fig_axs:Dict[int, Tuple[int,int]] = {1: (1,1),
                                                  2: (1,2),
                                                  3: (1,3),
                                                  4: (2,2),
                                                  5: (1,5),
                                                  6: (2,3),
                                                  8: (2,4),
                                                  9: (3,3)}
        self.y_labels:Dict[str, str] = {'succ': 'Success rate',
                                        'runtime': 'Runtime (sec)',
                                        'runtime of detecting conflicts':\
                                            'Runtiem of conflict detection (sec)',
                                        'runtime of path finding': 'Runtime of path finding (sec)',
                                        'solution cost': 'SOC',
                                        '#low-level generated': 'Number of generated LL Nodes',
                                        '#low-level expanded': 'Number of expansions',
                                        '#high-level generated': 'Number of generated HL Nodes',
                                        '#high-level expanded': 'Number expanded HL nodes',
                                        '#pathfinding': 'Number of replaned Agents', # (K)
                                        '#low-level search calls': 'Number of calls',
                                        '#backtrack': 'Number of backtrackings', # (K)
                                        '#restarts': 'Number of restarts', # (K)
                                        'num_total_conf': 'Number of total Conflicts',
                                        'add': 'Sum',
                                        'sub': 'Subtraction',
                                        'mul': 'Multiplication',
                                        'div': 'Average number\nof expansions',
                                        'mod': 'Mod'}
        self.x_labels:Dict[str,str] = {'num': 'Number of agents',
                                       'ins': 'MAPF Instance'}

    def get_subfig_pos(self, f_idx: int):
        """Transfer subplot index to 2-D position
        Args:
            f_idx (int): subplot index

        Returns:
            int, int: 2D position
        """
        f_row = self.fig_axs[len(self.config['maps'])][1]
        return f_idx // f_row, f_idx % f_row


    def get_val(self, x_index:str='num', y_index:str='succ', is_avg:bool=True):
        """Get the value on the y axid

        Args:
            x_index (str, optional): value of the x-axid. Defaults to 'num'.
            y_index (str, optional): value of the y-axid. Defaults to 'succ'.
            is_avg (bool, optional): whether to averaging the y value. Defaults to True.

        Returns:
            Dict: the y value on the y axid
        """
        if x_index == 'ins':
            return self.get_ins_val(y_index)
        elif x_index == 'num':
            return self.get_num_val(y_index, is_avg)

    def get_ins_val(self, in_index:str='runtime'):
        """Compute the success rate versus the numbers of agents

        Args:
            in_index (str, optional): which data we want to analyze. Defaults to 'runtime'.

        Returns:
            Dict: the success rate (versus the numbers of agents) of each solver
        """

        result: Dict = {}
        for solver in self.config['solvers']:
            result[solver['name']] = {}
            for _map_ in self.config['maps']:
                result[solver['name']][_map_['name']] = {'x': [], 'val': [], 'ci': []}
                global_idx = 1

                for ag_num in _map_['num_of_agents']:
                    for scen in _map_['scens']:
                        data_frame = util.get_csv_instance(self.config['exp_path'], _map_['name'],
                                                           scen, ag_num, solver['name'])
                        for _, row in data_frame.iterrows():
                            _val_ = util.process_val(row[in_index], in_index, row['solution cost'],
                                                     row['runtime'], self.config['time_limit'],
                                                     solver['name'], self.config['succ_only'])

                            result[solver['name']][_map_['name']]['val'].append(_val_)
                            result[solver['name']][_map_['name']]['x'].append(global_idx)
                            global_idx += 1
        return result


    def get_num_val(self, in_index:str='succ', is_avg:bool=True):
        """Compute the success rate versus the numbers of agents

        Args:
            in_index (str, optional): which data we want to analyze. Defaults to 'succ'.

        Returns:
            Dict: the success rate (versus the numbers of agents) of each solver
        """

        result: Dict = {}
        for solver in self.config['solvers']:
            result[solver['name']] = {}
            for _map_ in self.config['maps']:
                result[solver['name']][_map_['name']] = {'x': [], 'val': [], 'ci': []}

                for ag_num in _map_['num_of_agents']:
                    total_val = 0.0
                    total_num = 0.0
                    _data_:List = []
                    for scen in _map_['scens']:
                        tmp_ins_num = 0
                        data_frame = util.get_csv_instance(self.config['exp_path'], _map_['name'],
                                                           scen, ag_num, solver['name'])
                        for _, row in data_frame.iterrows():
                            tmp_ins_num += 1
                            raw_data = None
                            if in_index != 'succ':
                                raw_data = row[in_index]
                            _val_ = util.process_val(raw_data, in_index, row['solution cost'],
                                                    row['runtime'], self.config['time_limit'],
                                                    solver['name'], False)
                            total_val += _val_
                            _data_.append(_val_)

                        total_num += self.config['ins_num']
                        if tmp_ins_num != self.config['ins_num']:
                            logging.warning('Ins number does no match at map:%s, scen:%s, ag:%d',
                                            _map_['name'], scen, ag_num)

                    if is_avg:
                        if total_num == 0:
                            _rate_ = 0
                        else:
                            _rate_ = total_val / total_num  # average value
                    else:
                        _rate_ = total_val

                    result[solver['name']][_map_['name']]['x'].append(ag_num)
                    result[solver['name']][_map_['name']]['val'].append(_rate_)

                    if self.config['plot_ci'] and len(_data_) > 0:  # non empty list
                        _ci_ = 1.96*np.std(_data_) / np.sqrt(total_num)  # confident interval
                        result[solver['name']][_map_['name']]['ci'].append(_ci_)
                    elif self.config['plot_std'] and len(_data_) > 0:
                        _ci_ = np.std(_data_)  # standard deviation
                        result[solver['name']][_map_['name']]['ci'].append(_ci_)

        return result


    def get_w_val(self, in_index:str, is_avg:bool=True):
        """Compute the success rate versus the numbers of agents

        Args:
            in_index (str, optional): which data we want to analyze.

        Returns:
            Dict: the success rate (versus the numbers of agents) of each solver
        """

        result: Dict = {}
        for solver in self.config['solvers']:
            result[solver['name']] = {}
            for _map_ in self.config['maps']:
                result[solver['name']][_map_['name']] = {'x': [], 'val': [], 'ci': []}
                default_w = solver['w']

                for tmp_fw in self.config['f_weights']:
                    solver['w'] = tmp_fw
                    total_val = 0.0
                    total_num = 0.0
                    _data_:List = []

                    for ag_num in _map_['num_of_agents']:
                        for scen in _map_['scens']:
                            data_frame = util.get_csv_instance(self.config['exp_path'],
                                                    _map_['name'], scen, ag_num, solver['name'])
                            for _, row in data_frame.iterrows():
                                if in_index == 'succ':
                                    if row['solution cost'] >= 0 and \
                                        row['runtime'] <= self.config['time_limit']:
                                        total_val += 1
                                elif in_index == 'runtime':
                                    _data_.append(min(row[in_index], self.config['time_limit']))
                                    total_val += min(row[in_index], self.config['time_limit'])
                                else:
                                    assert row[in_index] >= 0
                                    _data_.append(row[in_index])
                                    total_val += row[in_index]

                            total_num += self.config['ins_num']

                    if is_avg:
                        _rate_ = total_val / total_num  # average value
                    else:
                        _rate_ = total_val

                    result[solver['name']][_map_['name']]['x'].append(tmp_fw)
                    result[solver['name']][_map_['name']]['val'].append(_rate_)

                    if self.config['plot_ci'] and len(_data_) > 0:  # non empty list
                        # _ci_ = 1.96*np.std(_data_) / np.sqrt(total_num)  # confident interval
                        _ci_ = np.std(_data_)  # standard deviation
                        result[solver['name']][_map_['name']]['ci'].append(_ci_)

                solver['x'] = default_w

        return result


    def subplot_fig(self, x_index, y_index, in_axs, in_map_idx, in_map, in_result):
        _x_ = in_result[self.config['solvers'][0]['name']][in_map['name']]['x']
        left_bd = -1 * self.config['set_shift']
        right_bd = self.config['set_shift']
        plt_rng = (right_bd - left_bd) / len(self.config['solvers'])
        _num_ = range(1, len(_x_)+1)

        # Plot the lower bound






        for s_idx, solver in enumerate(self.config['solvers']):
            mf_color = 'white'
            if 'markerfacecolor' in solver.keys():
                mf_color = solver['markerfacecolor']

            _val_ = in_result[solver['name']][in_map['name']]['val']
            _ci_  = in_result[solver['name']][in_map['name']]['ci']
            if abs(self.config['set_shift']) > 0:
                _num_ = [_n_ + plt_rng*s_idx for _n_ in _num_]

            if in_map_idx == 0:
                if (self.config['plot_std'] or self.config['plot_ci']) and len(_ci_) > 0:
                    in_axs.errorbar(_num_, _val_, yerr=_ci_,
                                    label=solver['label'],
                                    linewidth=self.line_width,
                                    markerfacecolor=mf_color,
                                    markeredgewidth=self.mark_width,
                                    ms=self.marker_size,
                                    color=solver['color'],
                                    marker=solver['marker'],
                                    alpha=self.config['alpha'],
                                    zorder=solver['zorder'])
                else:
                    in_axs.plot(_num_, _val_,
                                label=solver['label'],
                                linewidth=self.line_width,
                                markerfacecolor=mf_color,
                                markeredgewidth=self.mark_width,
                                ms=self.marker_size,
                                color=solver['color'],
                                marker=solver['marker'],
                                alpha=self.config['alpha'],
                                zorder=solver['zorder'])
            else:
                if (self.config['plot_std'] or self.config['plot_ci']) and len(_ci_) > 0:
                    in_axs.errorbar(_num_, _val_, yerr=_ci_,
                                    linewidth=self.line_width,
                                    markerfacecolor=mf_color,
                                    markeredgewidth=self.mark_width,
                                    ms=self.marker_size,
                                    color=solver['color'],
                                    marker=solver['marker'],
                                    alpha=self.config['alpha'],
                                    zorder=solver['zorder'])
                else:
                    in_axs.plot(_num_, _val_,
                                linewidth=self.line_width,
                                markerfacecolor=mf_color,
                                markeredgewidth=self.mark_width,
                                ms=self.marker_size,
                                color=solver['color'],
                                marker=solver['marker'],
                                alpha=self.config['alpha'],
                                zorder=solver['zorder'])

            # # Plot confident interval with fill_between
            # if self.config['plot_ci'] and len(_ci_) > 0:
            #     _lb_ = [_val_[i] - _ci_[i] for i in range(len(_val_))]
            #     _ub_ = [_val_[i] + _ci_[i] for i in range(len(_val_))]
            #     in_axs.fill_between(_num_, _lb_, _ub_, color=solver['color'], alpha=0.2)
        if self.config['set_title']:
            in_axs.set_title(in_map['label'], fontsize=self.text_size)

        if len(_num_) > self.max_x_num and x_index == "ins":  # This is for instance analysis
            _num_ = list(range(len(_x_)//self.max_x_num, len(_x_)+1, len(_x_)//self.max_x_num))
            _num_.insert(0, 1)
            _x_ = _num_

        in_axs.axes.set_xticks(_num_)
        in_axs.axes.set_xticklabels(_x_, fontsize=self.text_size)
        in_axs.set_xlabel(self.x_labels[x_index], fontsize=self.text_size)

        y_list = in_axs.axes.get_yticks()
        if y_index == 'succ':
            y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
            in_axs.axes.set_yticks(y_list)

        elif y_index == 'runtime':
            # y_list = range(0, 61, 10)
            y_list = range(0, self.config['time_limit']+1, self.config['time_gap'])
            in_axs.axes.set_yticks(y_list)

        elif y_index == 'max_ma_size':
            y_list = range(0, max(in_map['num_of_agents'])+5, 20)
            in_axs.axes.set_yticks(y_list)

        elif y_index == '#low-level expanded' or y_index == '#low-level generated':
            # in_axs.set_yscale('log')
            label_scale = 1000000
            tmp_range = 5
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)
            in_axs.axes.set_yticks(y_list)

            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        elif y_index == '#high-level generated':
            label_scale = 10
            tmp_range = 2
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)

            in_axs.axes.set_yticks(y_list)
            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        elif y_index == '#pathfinding' or y_index == '#low-level search calls' or \
            y_index =='#restarts' or y_index == 'solution cost':
            label_scale = 1000

            if in_map['name'] in util.LARGE_MAPS:
                tmp_range = 80
            else:
                tmp_range = 20
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)

            in_axs.axes.set_yticks(y_list)
            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        elif y_index == 'div':
            # y_list = [0, 0.5, 1.0, 1.5]
            y_list = [0, 1, 2, 3, 4]
            in_axs.axes.set_yticks(y_list)

        elif y_index == 'num_total_conf':
            label_scale = 1000
            tmp_range = 10
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)

            in_axs.axes.set_yticks(y_list)
            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        elif y_index == 'num_0child':
            label_scale = 1000
            tmp_range = 1
            scale = label_scale * tmp_range
            y_list = np.arange(0, max(y_list)+5, scale)

            in_axs.axes.set_yticks(y_list)
            if isinstance(tmp_range, float):
                y_list = [str(y/label_scale) for y in y_list]
            elif isinstance(tmp_range, int):
                y_list = [str(int(y//label_scale)) for y in y_list]

        else:
            in_axs.axes.set_yticks(y_list)

        if self.config['y_grid']:
            in_axs.yaxis.grid()
        if self.config['x_grid']:
            in_axs.xaxis.grid()
        in_axs.axes.set_yticklabels(y_list, fontsize=self.text_size)
        in_axs.set_ylabel(self.y_labels[y_index], fontsize=self.text_size)

    def subplot_fig2(self, x_index, y_index, in_axs, in_result):
        _x_ = in_result[self.config['solvers'][0]['name']]['x']
        _num_ = range(1, len(_x_)+1)

        for solver in self.config['solvers']:
            mf_color = 'white'
            if 'markerfacecolor' in solver.keys():
                mf_color = solver['markerfacecolor']

            _val_ = in_result[solver['name']]['val']

            in_axs.plot(_num_, _val_,
                        label=solver['label'],
                        linewidth=self.line_width,
                        markerfacecolor=mf_color,
                        markeredgewidth=self.mark_width,
                        ms=self.marker_size,
                        color=solver['color'],
                        marker=solver['marker'])

        if len(_num_) > self.max_x_num:
            _num_ = list(range(len(_x_)//self.max_x_num, len(_x_)+1, len(_x_)//self.max_x_num))
            _num_.insert(0, 1)
            _x_ = _num_

        in_axs.axes.set_xticks(_num_)
        in_axs.axes.set_xticklabels(_x_, fontsize=self.text_size)
        in_axs.set_xlabel(self.x_labels[x_index], fontsize=self.text_size)

        y_list = in_axs.axes.get_yticks()
        if y_index == 'succ':
            y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
            in_axs.axes.set_yticks(y_list)
        elif y_index == 'runtime':
            y_list = range(0, 61, 10)
            in_axs.axes.set_yticks(y_list)
        in_axs.axes.set_yticklabels(y_list, fontsize=self.text_size)
        in_axs.set_ylabel(self.y_labels[y_index], fontsize=self.text_size)


    def get_avg_vals(self, y_index='succ'):
        results = self.get_ins_val(y_index)
        output = {}
        for solver in self.config['solvers']:
            output[solver['name']] = {}
            for _map_ in self.config['maps']:
                total_val = 0
                total_ins = 0
                for _v_ in results[solver['name']][_map_['name']]['val']:
                    total_val += _v_
                    total_ins += 1
                tmp_avg = total_val / total_ins
                output[solver['name']][_map_['name']] = tmp_avg
        print (yaml.dump(output, allow_unicode=True, default_flow_style=False))

    def get_avg_vals_all(self, y_index='succ'):
        results = self.get_ins_val(y_index)
        output = {}
        for solver in self.config['solvers']:
            output[solver['name']] = {}
            total_val = 0
            total_ins = 0
            for _map_ in self.config['maps']:
                for _v_ in results[solver['name']][_map_['name']]['val']:
                    total_val += _v_
                    total_ins += 1
            tmp_avg = total_val / total_ins
            output[solver['name']] = tmp_avg
        print (yaml.dump(output, allow_unicode=True, default_flow_style=False))


    # def subplot_hist_fig(self, x_index, y_indices, in_axs, in_map_idx, in_map, in_results):
    #     _x_ = in_results[0][self.config['solvers'][0]['name']][in_map['name']]['x']
    #     _num_ = range(1, len(_x_)+1)

    #     for solver in self.config['solvers']:
    #         _val_ = in_result[solver['name']][in_map['name']]['val']
    #         _ci_  = in_result[solver['name']][in_map['name']]['ci']

    #         if in_map_idx == 0:
    #             in_axs.plot(_num_, _val_,
    #                         label=solver['label'],
    #                         linewidth=self.line_width,
    #                         markerfacecolor='white',
    #                         markeredgewidth=self.mark_width,
    #                         ms=self.marker_size,
    #                         color=solver['color'],
    #                         marker=solver['marker'])
    #         else:
    #             in_axs.plot(_num_, _val_,
    #                         linewidth=self.line_width,
    #                         markerfacecolor='white',
    #                         markeredgewidth=self.mark_width,
    #                         ms=self.marker_size,
    #                         color=solver['color'],
    #                         marker=solver['marker'])

    #         # Plot confident interval
    #         if self.config['plot_ci'] and len(_ci_) > 0:
    #             _lb_ = [_val_[i] - _ci_[i] for i in range(len(_val_))]
    #             _ub_ = [_val_[i] + _ci_[i] for i in range(len(_val_))]
    #             in_axs.fill_between(_num_, _lb_, _ub_, color=solver['color'], alpha=0.2)

    #     in_axs.set_title(in_map['label'], fontsize=self.text_size)

    #     if len(_num_) > self.max_x_num and x_index == "ins":  # This is for instance analysis
    #         _num_ = list(range(len(_x_)//self.max_x_num, len(_x_)+1, len(_x_)//self.max_x_num))
    #         _num_.insert(0, 1)
    #         _x_ = _num_

    #     in_axs.axes.set_xticks(_num_)
    #     in_axs.axes.set_xticklabels(_x_, fontsize=self.text_size)
    #     in_axs.set_xlabel(self.x_labels[x_index], fontsize=self.text_size)

    #     y_list = in_axs.axes.get_yticks()
    #     if y_index == 'succ':
    #         y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0]
    #         in_axs.axes.set_yticks(y_list)
    #     elif y_index == 'runtime':
    #         y_list = range(0, 61, 10)
    #         # y_list = range(0, 32, 5)
    #         # y_list = range(0, 26, 5)
    #         # y_list = range(0, 11, 2)
    #         # y_list = range(0, 2, 1)
    #         # y_list = [0, 0.5, 1.0, 1.5, 2.0]
    #         in_axs.axes.set_yticks(y_list)
    #     elif y_index == '#findPathForSingleAgent':
    #         in_axs.axes.set_yticks(y_list)
    #     elif y_index == '#low-level generated':
    #         label_scale = 1000000
    #         scale = label_scale * 0.1
    #         y_list = np.arange(0, max(y_list)+5, scale)
    #         # y_list = np.arange(0, max(y_list)+5, scale)
    #         # y_list = np.delete(y_list, 0)
    #         # y_list = np.delete(y_list, 0)
    #         # y_list = np.delete(y_list, -1)
    #         # y_list = np.delete(y_list, -1)
    #         in_axs.axes.set_yticks(y_list)
    #         y_list = [str(y/label_scale) for y in y_list]
    #         # y_list = [str(int(y//label_scale)) for y in y_list]
    #     elif y_index == '#high-level generated':
    #         label_scale = 1000
    #         scale = label_scale * 0.2
    #         y_list = np.arange(0, max(y_list)+5, scale)

    #         in_axs.axes.set_yticks(y_list)
    #         # y_list = [str(int(y)) for y in y_list]
    #         y_list = [str(y/label_scale) for y in y_list]
    #         # y_list = [str(int(y//label_scale)) for y in y_list]
    #     else:
    #         if y_index == 'div':
    #             y_list = [0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2]
    #             in_axs.axes.set_yticks(y_list)
    #         else:
    #             label_scale = 1000
    #             scale = label_scale * 1
    #             y_list = np.arange(0, max(y_list)+5, scale)
    #             in_axs.axes.set_yticks(y_list)
    #             y_list = [str(int(y//label_scale)) for y in y_list]        

    #     in_axs.yaxis.grid()
    #     in_axs.axes.set_yticklabels(y_list, fontsize=self.text_size)
    #     in_axs.set_ylabel(self.y_labels[y_index], fontsize=self.text_size)

    def plot_fig(self, x_index:str='num', y_index:str='succ'):
        tmp_lw = self.line_width
        if x_index == 'ins':
            self.line_width = 0.0
        # Get the result from the experiments
        result = self.get_val(x_index, y_index)

        # Plot all the subplots on the figure
        plt.close('all')

        fig, axs = plt.subplots(nrows=self.fig_axs[len(self.config['maps'])][0],
                                ncols=self.fig_axs[len(self.config['maps'])][1],
                                figsize=self.fig_size,
                                dpi=80, facecolor='w', edgecolor='k')

        for idx, _map_ in enumerate(self.config['maps']):
            frow, fcol = self.get_subfig_pos(idx)
            if len(self.config['maps']) == 1:
                self.subplot_fig(x_index, y_index, axs, idx, _map_, result)
            elif self.fig_axs[len(self.config['maps'])][0] == 1:
                self.subplot_fig(x_index, y_index, axs[fcol], idx, _map_, result)
            else:
                self.subplot_fig(x_index, y_index, axs[frow,fcol], idx, _map_, result)

        # manager = plt.get_current_fig_manager()
        # manager.full_screen_toggle()
        # plt.tight_layout(pad=0.05)

        if len(self.config['solvers']) > 7:
            # val_ncol = len(self.config['solvers'])
            val_ncol = int(np.ceil(len(self.config['solvers']) / 2))

        else:
            val_ncol = len(self.config['solvers'])
        # plt.subplots_adjust(left=val_left, right=val_right, top=val_top, bottom=val_bottom)

        if self.config['set_legend']:
            if len(self.config['maps']) > 1:
                fig.legend(loc="upper center",
                    bbox_to_anchor= (0.5, 1.01),
                    borderpad=0.1,
                    handletextpad=0.1,
                    labelspacing=0.1,
                    columnspacing=1.0,
                    ncol=val_ncol,
                    fontsize=self.text_size)
            else:
                plt.legend(loc="lower left", fontsize=self.text_size)

        fig_name = ''  # Set the figure name
        for _map_ in self.config['maps']:
            fig_name += _map_['label'] + '_'
        fig_name += x_index + '_' + y_index + '_plot.png'
        plt.savefig(fig_name)
        if x_index == 'ins':
            self.line_width = tmp_lw  # set the line width back

        plt.show()

    def plot_op(self, x_index:str='num', y_index1:str='#pathfinding',
                y_index2:str='#high-level generated',use_op:str='add'):
        """Plot the ratio between the sum of y_index1 / the sum of the y_index2

        Args:
            y_index1 (str, optional): list of the 1st numbers. Defaults to '#pathfinding'.
            y_index2 (str, optional): list of the 2nd numbers. Defaults to '#high-level generated'.
            use_op (str, optional): which operator to use. Defaults to 'add'.
        """
        op_list = ['add', 'sub', 'mul', 'div', 'mod']
        if use_op not in op_list:
            logging.error('use_op is undefine!, Should be one of the %s', op_list)
            sys.exit()

        # Get the result (sum) from the experiments
        val1 = self.get_val(x_index, y_index1, False)
        val2 = self.get_val(x_index, y_index2, False)
        x_list = val1[self.config['solvers'][0]['name']][self.config['maps'][0]['name']]['x']

        result = {}
        for _solver_ in self.config['solvers']:
            result[_solver_['name']] = {}
            for _map_ in self.config['maps']:
                result[_solver_['name']][_map_['name']] = {'x': [], 'val': [], 'ci': []}
                for idx, _x_ in enumerate(x_list):
                    tmp_val1 = val1[_solver_['name']][_map_['name']]['val'][idx]
                    tmp_val2 = val2[_solver_['name']][_map_['name']]['val'][idx]

                    if use_op == 'add':
                        tmp_val = tmp_val1 + tmp_val2
                    elif use_op == 'sub':
                        tmp_val = tmp_val1 - tmp_val2
                    elif use_op == 'mil':
                        tmp_val = tmp_val1 * tmp_val2
                    elif use_op == 'div':
                        if tmp_val2 == 0:
                            tmp_val = np.inf
                        else:
                            tmp_val = float(tmp_val1) / float(tmp_val2)
                    elif use_op == 'mod':
                        if tmp_val2 == 0:
                            tmp_val = np.inf
                        else:
                            tmp_val = float(tmp_val1) % float(tmp_val2)

                    result[_solver_['name']][_map_['name']]['x'].append(_x_)
                    result[_solver_['name']][_map_['name']]['val'].append(tmp_val)

        # Plot all the subplots on the figure
        fig, axs = plt.subplots(nrows=self.fig_axs[len(self.config['maps'])][0],
                                ncols=self.fig_axs[len(self.config['maps'])][1],
                                figsize=self.fig_size,
                                dpi=80, facecolor='w', edgecolor='k')

        for idx, _map_ in enumerate(self.config['maps']):
            frow, fcol = self.get_subfig_pos(idx)
            if len(self.config['maps']) == 1:
                self.subplot_fig(x_index, use_op, axs, idx, _map_, result)
            elif self.fig_axs[len(self.config['maps'])][0] == 1:
                self.subplot_fig(x_index, use_op, axs[fcol], idx, _map_, result)
            else:
                self.subplot_fig(x_index, use_op, axs[frow,fcol], idx, _map_, result)

        fig.tight_layout()

        if self.config['set_legend']:
            if use_op == 'div':
                plt.legend(loc="lower right", fontsize=self.text_size)
            else:
                plt.legend(loc="best", fontsize=self.text_size)

        fig_name = x_index + '_' + use_op + '_plot.png'
        plt.savefig(fig_name)
        plt.show()

    # def plot_hist_fig(self, x_index:str='num', y_index:List[str]=['num_ex_conf', 'num_in_conf']):
    #     # Get the result from the experiments
    #     results_list = []
    #     for y_idx in y_index:
    #         result = self.get_val(x_index, y_idx)
    #         results_list.append(result)

    #     # Plot all the subplots on the figure
    #     fig, axs = plt.subplots(nrows=self.fig_axs[len(self.config['maps'])][0],
    #                             ncols=self.fig_axs[len(self.config['maps'])][1],
    #                             figsize=self.fig_size,
    #                             dpi=80, facecolor='w', edgecolor='k')

    #     for idx, _map_ in enumerate(self.config['maps']):
    #         frow, fcol = self.get_subfig_pos(idx)
    #         if len(self.config['maps']) == 1:
    #             self.subplot_fig(x_index, y_index, axs, idx, _map_, results_list)
    #         elif self.fig_axs[len(self.config['maps'])][0] == 1:
    #             self.subplot_fig(x_index, y_index, axs[fcol], idx, _map_, results_list)
    #         else:
    #             self.subplot_fig(x_index, y_index, axs[frow,fcol], idx, _map_, results_list)

    #     fig.tight_layout()
    #     if y_index == 'succ':
    #         plt.legend(loc="lower left", fontsize=self.text_size)
    #     elif y_index == 'runtime' or y_index == '#low-level generated' or \
    #           y_index == '#high-level generated':
    #         plt.legend(loc="upper left", fontsize=self.text_size)
    #     else:
    #         plt.legend(loc="best", fontsize=self.text_size)
    #     fig_name = x_index + '_' + y_index + '_plot.png'
    #     plt.savefig(fig_name)
    #     plt.show()

    def get_ins_from_samples(self, sol_dir:str, sol_names:List[str],
                             mode:str='min', objective:str='runtime'):
        for _map_ in self.config['maps']:
            for _ag_num_ in _map_['num_of_agents']:
                for _scen_ in _map_['scens']:
                    util.create_csv_file(exp_path=self.config['exp_path'],
                                         map_name=_map_['name'],
                                         scen=_scen_,
                                         ag_num=_ag_num_,
                                         ins_num=self.config['ins_num'],
                                         sol_dir=sol_dir,
                                         sol_names=sol_names,
                                         mode=mode,
                                         objective=objective)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Take config.yaml as input!')
    parser.add_argument('--config', type=str, default='config.yaml')

    args = parser.parse_args()

    # Create data processor
    data_processor = DataProcessor(args.config)

    # data_processor.get_avg_vals(y_index='#low-level expanded')  # LL expanded nodes
    # data_processor.get_avg_vals(y_index='#low-level search calls')  #  LL runs
    # data_processor.get_avg_vals(y_index='#high-level expanded')
    # data_processor.get_avg_vals(y_index='succ')
    # data_processor.get_avg_vals_all(y_index='succ')

    # data_processor.plot_fig(x_index='num', y_index='succ')
    # data_processor.plot_fig(x_index='num', y_index='runtime')
    data_processor.plot_fig(x_index='num', y_index='#low-level search calls')
    # data_processor.plot_fig(x_index='num', y_index='#low-level expanded')
    # data_processor.plot_fig(x_index='num', y_index='#high-level expanded')
    # data_processor.plot_fig(x_index='num', y_index='#restarts')
    # data_processor.plot_fig(x_index='num', y_index='#backtrack')
    # data_processor.plot_fig(x_index='num', y_index='#pathfinding')

    # data_processor.plot_fig(x_index='ins', y_index='solution cost')
    # data_processor.plot_fig(x_index='ins', y_index='#high-level generated')
    # data_processor.plot_fig(x_index='ins', y_index='#low-level expanded')
    # data_processor.plot_fig(x_index='ins', y_index='#backtrack')
    # data_processor.plot_fig(x_index='ins', y_index='#low-level search calls')
    # data_processor.plot_fig(x_index='ins', y_index='num_total_conf')
    # data_processor.plot_fig(x_index='ins', y_index='num_in_conf')
    # data_processor.plot_fig(x_index='ins', y_index='num_ex_conf')

    # data_processor.plot_op(x_index='ins',y_index1='#low-level expanded',
    #                        y_index2='#high-level generated',use_op='div')
    # data_processor.plot_op(x_index='ins',y_index1='#low-level expanded',
    #                        y_index2='#high-level expanded',use_op='div')
