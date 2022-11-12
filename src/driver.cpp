/* Copyright (C) Jiaoyang Li
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
		("outputPT", po::value<string>(), "output file for prioritized tree")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<clock_t>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")
		("seed", po::value<int>()->default_value(-1), "Set random seed")

		("sipp", po::value<bool>()->default_value(1), "using SIPPS as the low-level solver")
		("opt", po::value<bool>()->default_value(0), "using optimal low-level SIPPS")
		("solver", po::value<string>()->default_value("PBS"), "Which high-level solver to use")
		("greedy", po::value<bool>()->default_value(0), "True if PBS chooses the child PT node with the lower number of conflicts")
		("tr", po::value<bool>()->default_value(0), "using target reasoning")
		("ic", po::value<bool>()->default_value(0), "using implicit constraint")
		("alpha", po::value<double>()->default_value(1.0), "the ratio of using implicit constraint")
		("rr", po::value<bool>()->default_value(0), "using random restart for PBS")
		("rth", po::value<uint64_t>()->default_value(0), "threshold to random restart for PBS")
		("lh", po::value<bool>()->default_value(0), "using LH heuristic for PP")
		("sh", po::value<bool>()->default_value(0), "using SH heuristic for PP")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);

	if (vm["seed"].as<int>() == -1)
		srand((int)time(0));
	else
		srand(vm["seed"].as<int>());

	///////////////////////////////////////////////////////////////////////////
	// load the instance
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>());

	if (vm["solver"].as<string>() == "PBS")
	{
		PBS pbs(instance, vm["screen"].as<int>(), vm["sipp"].as<bool>(), 
			vm["opt"].as<bool>(), vm["greedy"].as<bool>());
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
		PBS2 pbs2(instance, vm["screen"].as<int>(), vm["sipp"].as<bool>(), vm["opt"].as<bool>(),
			vm["tr"].as<bool>(), vm["ic"].as<bool>(), vm["rr"].as<bool>(), vm["rth"].as<uint64_t>(),
			vm["alpha"].as<double>(), vm["lh"].as<bool>(), vm["sh"].as<bool>());
		// run
		pbs2.solve(vm["cutoffTime"].as<clock_t>());
		if (vm.count("output"))
			pbs2.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
		if (pbs2.solution_found && vm.count("outputPaths"))
			pbs2.savePaths(vm["outputPaths"].as<string>());
		if (pbs2.solution_found && vm.count("outputPT"))
			pbs2.saveCT(vm["outputPT"].as<string>());
		pbs2.clearSearchEngines();
	}
	else if (vm["solver"].as<string>() == "PVCS")
	{
		PVCS pvcs(instance, vm["screen"].as<int>(), vm["sipp"].as<bool>(), vm["tr"].as<bool>());
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
		if (vm["lh"].as<bool>() and vm["sh"].as<bool>())
		{
			cerr << "ERROR: --lh and --sh should not be true at the same time!" << endl;
			exit(1);
		}

		PP pp(instance,  vm["screen"].as<int>(), vm["sipp"].as<bool>(),
			vm["opt"].as<bool>(), vm["lh"].as<bool>(), vm["sh"].as<bool>());
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