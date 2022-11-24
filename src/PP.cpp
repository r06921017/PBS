#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "PP.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"


PP::PP(const Instance& instance, int screen, bool sipp, bool is_ll_opt, bool use_LH, bool use_SH) :
    screen(screen), num_of_agents(instance.getDefaultNumberOfAgents()),
    num_of_cols(instance.num_of_cols), map_size(instance.map_size), 
    is_ll_opt(is_ll_opt), use_LH(use_LH), use_SH(use_SH)
{
    steady_clock::time_point t = steady_clock::now();

    search_engines.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        if (sipp)
            search_engines[i] = new SIPP(instance, i);
        else
            search_engines[i] = new SpaceTimeAStar(instance, i);
    }

    paths = vector<Path*>(num_of_agents, nullptr);
    ordered_agents = vector<int>(num_of_agents);
    iota(ordered_agents.begin(), ordered_agents.end(), 0);
    if (use_LH || use_SH)
    {
        ConstraintTable constraint_table(num_of_cols, map_size);
        vector<pair<int, size_t>> init_path_size;
        for (const int& _ag_ : ordered_agents)
        {
            // Use the individual shortest path to sort priorities
            int path_size = search_engines[_ag_]->my_heuristic[search_engines[_ag_]->goal_location];
            init_path_size.emplace_back(_ag_, path_size);
        }

        if (use_LH)
            sort(init_path_size.begin(), init_path_size.end(), sortByLongerPaths);
        else if (use_SH)
            sort(init_path_size.begin(), init_path_size.end(), sortByShorterPaths);

        ordered_agents.clear();
        for (const auto& _p_ : init_path_size)
        {
            ordered_agents.push_back(_p_.first);
        }
    }
    else
    {
        std::random_shuffle(ordered_agents.begin(), ordered_agents.end());
    }
    runtime_preprocessing = getDuration(t, steady_clock::now());

    #ifndef NDEBUG
    if (screen > 1)  // print start and goals
        instance.printAgents();
    #endif

    if (screen > 0)
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
}

bool PP::solve(double _time_limit)
{
    this->time_limit = _time_limit;
    start = steady_clock::now();  // set timer
    while (runtime < time_limit * CLOCKS_PER_SEC)
    {
        ConstraintTable constraint_table(num_of_cols, map_size);
        auto p = ordered_agents.begin();
        while (p != ordered_agents.end())
        {
            int id = *p;
            Path new_path;
            if (is_ll_opt)
                new_path = search_engines[id]->findOptimalPath(constraint_table);
            else
                new_path = search_engines[id]->findPath(constraint_table);

            runtime_path_finding += search_engines[id]->runtime;
            runtime_build_CT += search_engines[id]->runtime_build_CT;
            runtime_build_CAT += search_engines[id]->runtime_build_CAT;
            runtime = getDuration(start, steady_clock::now());
            if (runtime >= time_limit * CLOCKS_PER_SEC)
            {
                solution_cost = -1;
                break;
            }
            else if (new_path.empty())
            {
                if (screen > 1)
                    cout << "No path exists for agent " << id << endl;
                std::random_shuffle(ordered_agents.begin(), ordered_agents.end());
                num_restart ++;
                break;
            }
            paths[id] = new Path(new_path);
            constraint_table.insert2CT(new_path);

            if (screen > 1)
                printAgentPath(id);

            ++p;
        }
        if (p == ordered_agents.end())
        {
            runtime = getDuration(start, steady_clock::now());
            solution_found = true;
            solution_cost = 0;
            for (const auto& p : paths)
                solution_cost += (int)p->size() - 1;
            if (!validateSolution())
            {
                cout << "Solution invalid!!!" << endl;
                printPaths();
                exit(1);
            }
            if (screen > 0) // 1 or 2
			    printResults();

            #ifndef NDEBUG
            if (screen > 1)
            {
                cout << "init agents,";
                for (const int& ag : ordered_agents)
                    cout << ag << ",";
                cout << endl;
                cout << "ll exp,";
                for (const int& jj: ordered_agents)
                    cout << search_engines[jj]->getNumExpanded() << ",";
                cout << endl;
            }
            #endif

            return solution_found;
        }
        else if (solution_cost == -1)
        {
            runtime = getDuration(start, steady_clock::now());
            cout << "Timeout," << (double)runtime / CLOCKS_PER_SEC << ",0,0," << endl;
            return solution_found;
        }
        std::random_shuffle(ordered_agents.begin(), ordered_agents.end());
        runtime = getDuration(start, steady_clock::now());
    }
    return solution_found;
}

void PP::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();
}

void PP::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated," <<
            "#low-level expanded,#low-level generated,#backtrack," << 
            "#low-level search calls,#restart," <<
			"solution cost,root g value," <<
			"runtime of detecting conflicts,runtime of building constraint tables," << 
            "runtime of building CATs,runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,runtime of implicit constraints,runtime of MVC," << 
            "solver name,instance name" << endl;
		addHeads.close();
	}

    double out_runtime = (double)runtime / CLOCKS_PER_SEC;
    double out_runtime_detect_conflicts = (double)runtime_detect_conflicts / CLOCKS_PER_SEC;
    double out_runtime_build_CT = (double)runtime_build_CT / CLOCKS_PER_SEC;
    double out_runtime_build_CAT = (double)runtime_build_CAT / CLOCKS_PER_SEC;
    double out_runtime_path_finding = (double)runtime_path_finding / CLOCKS_PER_SEC;
    double out_runtime_preprocessing = (double)runtime_preprocessing / CLOCKS_PER_SEC;

    uint64_t num_LL_runs = 0, num_LL_expanded = 0, num_LL_generated = 0;
    for (int i=0; i < num_of_agents; i++)
    {
        num_LL_runs += search_engines[i]->num_runs;
        num_LL_expanded += search_engines[i]->accumulated_num_expanded;
        num_LL_generated += search_engines[i]->accumulated_num_generated;
    }

	ofstream stats(fileName, std::ios::app);
	stats << out_runtime << "," << num_HL_expanded << "," << num_HL_generated << "," <<
        num_LL_expanded << "," << num_LL_generated << "," << 0 << "," <<
        num_LL_runs << "," << num_restart << "," <<
        solution_cost << "," << 0 << "," <<
		out_runtime_detect_conflicts << "," << out_runtime_build_CT << "," << 
        out_runtime_build_CAT << "," << out_runtime_path_finding << "," << 
        0 << "," << out_runtime_preprocessing << "," << 
        0 << "," << 0 << "," << 
        getSolverName() << "," << instanceName << endl;
	stats.close();
}

void PP::savePaths(const string &fileName) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        output << "Agent " << i << ": ";
        for (const auto & t : *paths[i])
            output << "(" << search_engines[0]->instance.getRowCoordinate(t.location)
                   << "," << search_engines[0]->instance.getColCoordinate(t.location) << ")->";
        output << endl;
    }
    output.close();
}

// used for rapid random  restart
void PP::clear()
{
	paths.clear();
	solution_found = false;
	solution_cost = -2;
}

string PP::getSolverName() const
{
    string sol_name = "PP";
    if (use_LH)
        sol_name += "-LH";
    if (use_SH)
        sol_name += "-SH";
    
    sol_name += " with ";
    if (is_ll_opt)
        sol_name += "opt";
    sol_name += search_engines[0]->getName();

    return sol_name;
}

int PP::getSumOfCosts() const
{
   int cost = 0;
   for (const auto & path : paths)
       cost += (int)path->size() - 1;
   return cost;
}

void PP::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Succeed, ";
	else if (solution_cost == -1) // time_out
		cout << "Timeout, ";
	else if (solution_cost == -2) // no solution
		cout << "No solutions, ";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout, ";

	cout << "Runtime:" << (double)runtime / CLOCKS_PER_SEC << ", cost:" << solution_cost << endl;
    /*if (solution_cost >= 0) // solved
    {
        cout << "fhat = [";
        auto curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "hhat = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "d = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->distance_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "soc = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() - curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
    }*/
}

bool PP::validateSolution() const
{
	// check whether the paths are feasible
	size_t soc = 0;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		soc += paths[a1]->size() - 1;
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
					return false;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
						loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
					return false;
				}
			}
			if (paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
						return false; // It's at least a semi conflict			
					}
				}
			}
		}
	}
	if ((int)soc != solution_cost)
	{
		cout << "The solution cost is wrong!" << endl;
		return false;
	}
	return true;
}

inline int PP::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t)0);
	return paths[agent_id]->at(t).location;
}

void PP::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << search_engines[i]->my_heuristic[search_engines[i]->start_location] << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (const auto & t : *paths[i])
			cout << t.location << "->";
		cout << endl;
	}
}

void PP::printAgentPath(int i) const
{
    cout << "Agent " << i << " (" << search_engines[i]->my_heuristic[search_engines[i]->start_location] << " -->" <<
        paths[i]->size() - 1 << "): ";
    for (const auto & t : *paths[i])
        cout << t.location << "->";
    cout << endl;
}

bool PP::hasConflicts(int a1, int a2) const
{
	int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size());
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2 or (timestep < min_path_length - 1 and loc1 == paths[a2]->at(timestep + 1).location
                             and loc2 == paths[a1]->at(timestep + 1).location)) // vertex or edge conflict
		{
            return true;
		}
	}
	if (paths[a1]->size() != paths[a2]->size())
	{
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (int timestep = min_path_length; timestep < (int)paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{
				return true; // target conflict
			}
		}
	}
    return false; // conflict-free
}