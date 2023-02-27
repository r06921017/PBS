//#pragma warning(disable: 4996)
#include <algorithm>
#include <random>
#include <chrono>
#include "PBS2.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"

PBS2::PBS2(const Instance& instance, int screen, bool sipp, bool is_ll_opt, bool use_tr,
    bool use_ic, bool use_rr, uint64_t rr_th, double ic_ratio, bool use_LH, bool use_SH): 
    PBS(instance, screen, sipp, is_ll_opt, use_LH, use_SH, use_rr, rr_th), 
    use_tr(use_tr), use_ic(use_ic), ic_ratio(ic_ratio) {}

bool PBS2::solve(clock_t time_limit)
{
    this->time_limit = time_limit;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
    }
    // set timer
    start = steady_clock::now();

    while (solution_cost == -2)  // Not yet find a solution
    {
        generateRoot();

        while (!open_list.empty())
        {
            auto curr = selectNode();

            if (terminate(curr)) break;

            steady_clock::time_point t1;
            if (!curr->is_expanded)  // This is not a back-tracking
            {
                curr->conflict = chooseConflict(*curr);
                curr->is_expanded = true;
                if (screen > 1)
                    cout << "Expand " << *curr << " on " << *(curr->conflict) << endl;

                // assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and 
                //     !hasHigherPriority(curr->conflict->a2, curr->conflict->a1) );

                t1 = steady_clock::now();
                generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
                runtime_generate_child += getDuration(t1, steady_clock::now());

                if (curr->children[0] != nullptr)
                    pushNode(curr->children[0]);
            }
            else  // We only generate another child node if back-tracking happens
            {
                if (screen > 1)
                    cout << "Expand " << *curr << " on " << *(curr->conflict) << endl;

                open_list.pop();
                t1 = steady_clock::now();    
                generateChild(1, curr, curr->conflict->a2, curr->conflict->a1);
                runtime_generate_child += getDuration(t1, steady_clock::now());

                if (curr->children[1] != nullptr)
                {
                    pushNode(curr->children[1]);
                }
                else
                {
                    num_backtrack ++;
                    local_num_backtrack ++;
                    if (use_rr && local_num_backtrack > rr_th)
                    {
                        num_restart ++;
                        local_num_backtrack = 0;

                        // Only restart the Priority Graph
                        update(curr);
                        PBSNode* new_root = generateRoot(curr);
                        clear();
                        stack<PBSNode*>().swap(open_list);  // clear the open_list
                        pushNode(new_root);
                        dummy_start = new_root;
                        // std::random_shuffle(init_agents.begin(), init_agents.end());
                        // break;  // leave the while loop of open_list.empty
                        #ifndef NDEBUG
                        assert(open_list.size() == 1);
                        assert(allNodes_table.size() == 1);
                        if (screen > 1)
                        {
                            printConflicts(*new_root);
                            cout << endl;
                        }
                        #endif
                        continue;
                    }
                }
                curr->clear();
            }
        }  // end of while loop
    }
    return solution_found;
}

PBSNode* PBS2::selectNode(void)
{
	PBSNode* curr = open_list.top();
    update(curr);
    if (!curr->is_expanded)
    {
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;
    }
	return curr;
}

string PBS2::getSolverName(void) const
{
    string sol_name = "PBS2";
    if (use_tr)
        sol_name += "+TR";
    if (use_ic)
        sol_name += "+IC";
    if (use_rr)
        sol_name += "+RR";
    sol_name += " with " + search_engines[0]->getName();
    return sol_name;
}

bool PBS2::generateRoot(void)
{
    paths = vector<Path*>(num_of_agents, nullptr);
    PBSNode* root = new PBSNode();
	root->cost = 0;
	root->depth = 0;
    set<int> higher_agents;
    for (const int& _ag_ : init_agents)  // Find a path for individual agents
    {
        Path new_path;
        if (is_ll_opt)
            new_path = search_engines[_ag_]->findOptimalPath(higher_agents, paths, _ag_);
        else
            new_path = search_engines[_ag_]->findPath(higher_agents, paths, _ag_);
        runtime_path_finding += search_engines[_ag_]->runtime;
        runtime_build_CT += search_engines[_ag_]->runtime_build_CT;
        runtime_build_CAT += search_engines[_ag_]->runtime_build_CAT;        
        if (new_path.empty())
        {
            cout << "No path exists for agent " << _ag_ << endl;
            return false;
        }

        root->paths.emplace_back(_ag_, new_path);
        paths[_ag_] = &root->paths.back().second;
        root->makespan = max(root->makespan, new_path.size() - 1);
        root->cost += (int)new_path.size() - 1;
    }

    // Find all conflicts among paths
    steady_clock::time_point t = steady_clock::now();
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            if (use_tr)
            {
                conflict_priority priority = hasConflicts(a1, a2);
                if (priority > conflict_priority::NONE)
                {
                    // bool is_passing_start = false;
                    // for (const auto& _p_ : *paths[a1])
                    // {
                    //     if (_p_.location == paths[a2]->front().location)
                    //     {
                    //         is_passing_start = true;
                    //         break;
                    //     }
                    // }
                    // if (!is_passing_start)
                    // {
                    //     for (const auto& _p_ : *paths[a2])
                    //     if (_p_.location == paths[a1]->front().location)
                    //     {
                    //         is_passing_start = true;
                    //         break;
                    //     }
                    // }

                    // if (is_passing_start)
                    // {
                    //     if (priority == conflict_priority::TARGET)
                    //     {
                    //         priority = conflict_priority::TARGET_START;
                    //     }
                    //     else
                    //     {
                    //         assert(priority == conflict_priority::NORMAL);
                    //         priority = conflict_priority::START;
                    //     }
                    // }

                    // Prioritize to resolve target conflicts earlier others
                    // We set agents with longer paths higher priorities
                    shared_ptr<Conflict> new_conflict;
                    if (paths[a1]->size() < paths[a2]->size())
                        new_conflict = make_shared<Conflict>(a1, a2, priority);
                    else
                        new_conflict = make_shared<Conflict>(a2, a1, priority);
                    root->conflicts.push(new_conflict);
                }
            }
            else if (PBS::hasConflicts(a1, a2))  // not using target reasoning
            {
                // We set agents with longer paths higher priorities
                shared_ptr<Conflict> new_conflict;
                if (paths[a1]->size() < paths[a2]->size())
                    new_conflict = make_shared<Conflict>(a1, a2);
                else
                    new_conflict = make_shared<Conflict>(a2, a1);
                root->conflicts.push(new_conflict);
            }
        }
    }
    runtime_detect_conflicts += getDuration(t, steady_clock::now());

    // Initialize the matrix for the number of implicit constraints
    // Initially, there is no implicit constraints
    if (use_ic)
    {
        root->num_IC = make_shared<vector<vector<uint>>>(
            vector<vector<uint>>(num_of_agents, vector<uint>(num_of_agents, 0)));
    }

    num_HL_generated++;
    root->time_generated = num_HL_generated;
    pushNode(root);
	dummy_start = root;
    #ifndef NDEBUG
    if (screen > 1)
        cout << "Generate " << *root << endl;
	if (screen >= 2) // print start and goals
		printPaths();
    #endif

    return true;
}

PBSNode* PBS2::generateRoot(const PBSNode* node)
{
    PBSNode* root = new PBSNode();
    root->cost = 0;
    root->depth = 0;
    root->is_expanded = false;
    for (const int& _ag_ : init_agents)
    {
        Path new_path(*paths[_ag_]);
        root->paths.emplace_back(_ag_, new_path);
        paths[_ag_] = &root->paths.back().second;
        root->makespan = max(root->makespan, new_path.size() - 1);
        root->cost += (int)new_path.size() - 1;
    }

    for (shared_ptr<Conflict> conf : node->conflicts)
    {
        conf->num_ic = 0;
        // conf->ll_calls = 1;
        conf->ll_calls = paths[conf->a1]->size();
        root->conflicts.push(conf);
    }

    root->time_generated = node->time_generated;
    root->time_expanded = node->time_expanded;
    if (use_ic)
    {
        root->num_IC = make_shared<vector<vector<uint>>>(
            vector<vector<uint>>(num_of_agents, vector<uint>(num_of_agents, 0)));
    }
    return root;
}

bool PBS2::generateChild(int child_id, PBSNode* parent, int low, int high)
{
    assert(child_id == 0 or child_id == 1);
    parent->children[child_id] = new PBSNode(*parent);
    PBSNode* node = parent->children[child_id];
    node->constraint.set(low, high);
    priority_graph[high][low] = false;
    priority_graph[low][high] = true;
    assert(node->conflicts.empty());

    list<shared_ptr<Conflict>> cur_conflicts;  // Copy parent conflicts to a list
    for (shared_ptr<Conflict> conf : parent->conflicts)
        cur_conflicts.push_back(conf);

    topologicalSort(ordered_agents);
    // if (screen > 2)
    // {
    //     cout << "Ordered agents: ";
    //     for (int i : ordered_agents)
    //         cout << i << ",";
    //     cout << endl;
    // }
    vector<int> topological_orders(num_of_agents); // map agent i to its position in ordered_agents
    int i = num_of_agents - 1;
    for (const int & a : ordered_agents)
    {
        topological_orders[a] = i;
        i--;
    }

    std::priority_queue<pair<int, int>> to_replan; // <position in ordered_agents, agent id>
    vector<bool> lookup_table(num_of_agents, false);  // True if the agent is in to_replan
    to_replan.emplace(topological_orders[low], low);
    lookup_table[low] = true;

    // find conflicts where one agent is higher than high and the other agent is lower than low
    set<int> higher_agents;
    auto p = ordered_agents.rbegin();
    std::advance(p, topological_orders[high]);
    assert(*p == high);
    getHigherPriorityAgents(p, higher_agents);
    higher_agents.insert(high);

    set<int> lower_agents;
    auto p2 = ordered_agents.begin();
    std::advance(p2, num_of_agents - 1 - topological_orders[low]);
    assert(*p2 == low);
    getLowerPriorityAgents(p2, lower_agents);

    for (const auto & conflict : cur_conflicts)
    {
        int a1 = conflict->a1;
        int a2 = conflict->a2;
        if (a1 == low or a2 == low)
            continue;
        if (topological_orders[a1] > topological_orders[a2])
        {
            std::swap(a1, a2);  // a1 always has a smaller priority than a2
        }
        if (lower_agents.find(a1) != lower_agents.end() and 
            higher_agents.find(a2) != higher_agents.end() and
            !lookup_table[a1])
        {
            to_replan.emplace(topological_orders[a1], a1);
            lookup_table[a1] = true;
        }
    }

    while(!to_replan.empty())  // Only replan agents with lower priorities AND has known conflicts
    {
        int a, rank;
        tie(rank, a) = to_replan.top();
        to_replan.pop();
        lookup_table[a] = false;

        // Get agents with higher priorities
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, rank);
        getHigherPriorityAgents(p, higher_agents);
        #ifndef NDEBUG
        assert(*p == a);
        assert(!higher_agents.empty());
        if (screen > 1)
            cout << "\tReplan agent " << a << endl;
        if (screen > 2)
        {
            cout << "\tHigher agents: ";
            for (auto i : higher_agents)
                cout << i << ",";
            cout << endl;
        }
        #endif

        // Re-plan path
        Path new_path;
        if(!findPathForSingleAgent(*node, higher_agents, a, new_path))
        {
            delete node;
            parent->children[child_id] = nullptr;
            return false;
        }

        // Delete old conflicts
        list<shared_ptr<Conflict>>::iterator _cit_ = cur_conflicts.begin();
        while(_cit_ != cur_conflicts.end())
        {
            if ((*_cit_)->a1 == a or (*_cit_)->a2 == a)
                _cit_ = cur_conflicts.erase(_cit_);
            else
                ++_cit_;
        }

        // Update conflicts and to_replan
        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - rank);
        getLowerPriorityAgents(p2, lower_agents);
        #ifndef NDEBUG
        assert(*p2 == a);
        if (screen > 2 and !lower_agents.empty())
        {
            cout << "\tLower agents: ";
            for (auto i : lower_agents)
                cout << i << ",";
            cout << endl;
        }
        #endif

        // Find new conflicts
        for (auto a2 = 0; a2 < num_of_agents; a2++)
        {
            if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0)
                continue;  // already in to_replan or has higher priority
            steady_clock::time_point t = steady_clock::now();

            if (use_tr)
            {
                conflict_priority priority = hasConflicts(a, a2);
                if (priority > conflict_priority::NONE)  // there is a conflict
                {
                    if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                    {
                        #ifndef NDEBUG
                        if (screen > 1)
                            cout << "\tshould replan " << a2 << " for colliding with " << a << endl;
                        #endif
                        to_replan.emplace(topological_orders[a2], a2);
                        lookup_table[a2] = true;
                    }
                    else
                    {
                        // bool is_passing_start = false;
                        // for (const auto& _p_ : *paths[a])
                        // {
                        //     if (_p_.location == paths[a2]->front().location)
                        //     {
                        //         is_passing_start = true;
                        //         break;
                        //     }
                        // }
                        // if (!is_passing_start)
                        // {
                        //     for (const auto& _p_ : *paths[a2])
                        //     if (_p_.location == paths[a]->front().location)
                        //     {
                        //         is_passing_start = true;
                        //         break;
                        //     }
                        // }
                        // if (is_passing_start)
                        // {
                        //     if (priority == conflict_priority::TARGET)
                        //     {
                        //         priority = conflict_priority::TARGET_START;
                        //     }
                        //     else
                        //     {
                        //         assert(priority == conflict_priority::NORMAL);
                        //         priority = conflict_priority::START;
                        //     }
                        // }

                        // We set agents with longer paths higher priorities
                        shared_ptr<Conflict> new_conflict;
                        if (paths[a]->size() < paths[a2]->size())
                            new_conflict = make_shared<Conflict>(a, a2, priority);
                        else
                            new_conflict = make_shared<Conflict>(a2, a, priority);
                        cur_conflicts.push_back(new_conflict);
                    }
                }
            }
            else if (PBS::hasConflicts(a, a2))
            {
                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                {
                    #ifndef NDEBUG
                    if (screen > 1)
                        cout << "\tshould replan " << a2 << " for colliding with " << a << endl;
                    #endif
                    to_replan.emplace(topological_orders[a2], a2);
                    lookup_table[a2] = true;
                }
                else
                {
                    shared_ptr<Conflict> new_conflict;
                    if (paths[a]->size() < paths[a2]->size())
                        new_conflict = make_shared<Conflict>(a, a2);
                    else
                        new_conflict = make_shared<Conflict>(a2, a);
                    cur_conflicts.push_back(new_conflict);
                }
            }
            runtime_detect_conflicts += getDuration(t, steady_clock::now());
        }
    }

    if (use_ic)  // Compute the implicit constraints
        computeImplicitConstraints(node, cur_conflicts, topological_orders);

    assert(node->conflicts.empty());
    for (shared_ptr<Conflict> c: cur_conflicts)
        node->conflicts.push(c);

    num_HL_generated++;
    node->time_generated = num_HL_generated;
    if (screen > 1)
        cout << "\tGenerate " << *node << endl;
    return true;
}

conflict_priority PBS2::hasConflicts(int a1, int a2) const
{
	int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size());
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
				return conflict_priority::TARGET; // target conflict
			}
		}
	}

    for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2 or (timestep < min_path_length - 1 and
                             loc1 == paths[a2]->at(timestep + 1).location and
                             loc2 == paths[a1]->at(timestep + 1).location))
		{
            return conflict_priority::NORMAL;  // vertex or edge conflict
		}
	}
    return conflict_priority::NONE; // conflict-free
}

shared_ptr<Conflict> PBS2::chooseConflict(const PBSNode &node) const
{
    if (node.conflicts.empty())
        return nullptr;
    shared_ptr<Conflict> out = node.conflicts.top();
    return out;
}

void PBS2::computeImplicitConstraints(PBSNode* node, list<shared_ptr<Conflict>> conflicts, 
    const vector<int>& topological_orders)
{
    vector<set<int>> higher_pri_agents(num_of_agents);
    vector<set<int>> lower_pri_agents(num_of_agents);

    list<int>::reverse_iterator ag_rit;
    list<int>::iterator ag_it;

    steady_clock::time_point t = steady_clock::now();
    for (shared_ptr<Conflict> conf : conflicts)
    {
        if (conf->priority == conflict_priority::TARGET)
            continue;

        // Compute the additional implicit constraints for priority a2 <- a1
        if (higher_pri_agents[conf->a1].empty())
        {
            ag_rit = ordered_agents.rbegin();
            std::advance(ag_rit, topological_orders[conf->a1]);
            assert(*ag_rit == conf->a1);
            getHigherPriorityAgents(ag_rit, higher_pri_agents[conf->a1]);
        }

        if (lower_pri_agents[conf->a2].empty())
        {
            ag_it = ordered_agents.begin();
            std::advance(ag_it, num_of_agents - 1 - topological_orders[conf->a2]);
            assert(*ag_it == conf->a2);
            getLowerPriorityAgents(ag_it, lower_pri_agents[conf->a2]);
        }

        uint num_ic_a2_a1 = (uint) ((higher_pri_agents[conf->a1].size()+1) * 
            (lower_pri_agents[conf->a2].size()+1));
        for (const int& h_ag : higher_pri_agents[conf->a1])
        {
            for (const int& l_ag : lower_pri_agents[conf->a2])
            {
                if (priority_graph[l_ag][h_ag])  // Reduce the IC that already exists
                {
                    num_ic_a2_a1 -= node->num_IC->at(l_ag).at(h_ag);
                }
            }
        }
        node->num_IC->at(conf->a2).at(conf->a1) = num_ic_a2_a1;
        // double val_a1_a2 = ic_ratio * (double)num_ic_a2_a1 + 
        //     (1.0-ic_ratio) / (double)(lower_pri_agents[conf->a2].size()+1);

        // Compute the additional implicit constraints for priority a1 <- a2
        if (higher_pri_agents[conf->a2].empty())
        {
            ag_rit = ordered_agents.rbegin();
            std::advance(ag_rit, topological_orders[conf->a2]);
            assert(*ag_rit == conf->a2);
            getHigherPriorityAgents(ag_rit, higher_pri_agents[conf->a2]);
        }

        if (lower_pri_agents[conf->a1].empty())
        {
            ag_it = ordered_agents.begin();
            std::advance(ag_it, num_of_agents - 1 - topological_orders[conf->a1]);
            assert(*ag_it == conf->a1);
            getLowerPriorityAgents(ag_it, lower_pri_agents[conf->a1]);
        }

        uint num_ic_a1_a2 = (uint) (higher_pri_agents[conf->a2].size()+1) * 
            (lower_pri_agents[conf->a1].size()+1);
        for (const int& h_ag : higher_pri_agents[conf->a2])
        {
            for (const int& l_ag : lower_pri_agents[conf->a1])
            {
                if (priority_graph[l_ag][h_ag])  // Reduce the IC that already exists
                {
                    num_ic_a1_a2 -= node->num_IC->at(l_ag).at(h_ag);
                }
            }
        }
        node->num_IC->at(conf->a1).at(conf->a2) = num_ic_a1_a2;
        // double val_a2_a1 = ic_ratio * (double)num_ic_a2_a1 +
        //     (1.0-ic_ratio) / (double)(lower_pri_agents[conf->a1].size()+1);

        // Assign the maximum to the current conflict
        if (num_ic_a2_a1 > num_ic_a1_a2)
        {
            conf->num_ic = num_ic_a2_a1;
            // conf->ll_calls = lower_pri_agents[conf->a2].size()+1;
            conf->ll_calls = paths[conf->a2]->size();
            for (const int& _la_ : lower_pri_agents[conf->a2])
                conf->ll_calls += paths[_la_]->size();
            std::swap(conf->a1, conf->a2);
        }
        else
        {
            conf->num_ic = num_ic_a1_a2;
            // conf->ll_calls = lower_pri_agents[conf->a1].size()+1;
            conf->ll_calls = paths[conf->a1]->size();
            for (const int& _la_ : lower_pri_agents[conf->a1])
                conf->ll_calls += paths[_la_]->size();
        }
    }
    runtime_implicit_constraints += getDuration(t, steady_clock::now());
}