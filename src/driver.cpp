﻿/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "PBS.h"
#include "PBS2.h"
#include "PVCS.h"
#include "PP.h"

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("output,o", po::value<string>(), "output file for statistics")
		("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<clock_t>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")

		("sipp", po::value<bool>()->default_value(1), "using SIPP as the low-level solver")
		("solver", po::value<string>()->default_value("PBS"), "Which high-level solver to use")
		("tr", po::value<bool>()->default_value(1), "using target reasoning")
		("ic", po::value<bool>()->default_value(1), "using implicit constraint")
		("rr", po::value<bool>()->default_value(0), "using random restart for PBS")
		("lh", po::value<bool>()->default_value(0), "using LH heuristic for PP")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);

	// srand((int)time(0));
	///////////////////////////////////////////////////////////////////////////
	// load the instance
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>());

	// srand(0);

	if (vm["solver"].as<string>() == "PBS")
	{
		PBS pbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
		// run
		pbs.solve(vm["cutoffTime"].as<clock_t>());
		if (vm.count("output"))
			pbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
		if (pbs.solution_found && vm.count("outputPaths"))
			pbs.savePaths(vm["outputPaths"].as<string>());
		pbs.clearSearchEngines();
	}
	else if (vm["solver"].as<string>() == "PBS2")
	{
		PBS2 pbs2(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>(),
			vm["tr"].as<bool>(), vm["ic"].as<bool>(), vm["rr"].as<bool>());
		// run
		pbs2.solve(vm["cutoffTime"].as<clock_t>());
		if (vm.count("output"))
			pbs2.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
		if (pbs2.solution_found && vm.count("outputPaths"))
			pbs2.savePaths(vm["outputPaths"].as<string>());
		pbs2.clearSearchEngines();
	}
	else if (vm["solver"].as<string>() == "PVCS")
	{
		PVCS pvcs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>(), vm["tr"].as<bool>());
		// run
		pvcs.solve(vm["cutoffTime"].as<clock_t>());
		if (vm.count("output"))
			pvcs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
		if (pvcs.solution_found && vm.count("outputPaths"))
			pvcs.savePaths(vm["outputPaths"].as<string>());
		pvcs.clearSearchEngines();
	}
	else if (vm["solver"].as<string>() == "PP")
	{
		PP pp(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>(), vm["lh"].as<bool>());
		// run
		pp.solve(vm["cutoffTime"].as<clock_t>());
		if (vm.count("output"))
			pp.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
		if (pp.solution_found && vm.count("outputPaths"))
			pp.savePaths(vm["outputPaths"].as<string>());
		pp.clearSearchEngines();
	}

	return 0;

}