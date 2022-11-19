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
                    cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

                assert(!hasHigherPriority(curr->conflict->a1, curr->conflict->a2) and 
                    !hasHigherPriority(curr->conflict->a2, curr->conflict->a1) );

                t1 = steady_clock::now();
                generateChild(0, curr, curr->conflict->a1, curr->conflict->a2);
                runtime_generate_child += getDuration(t1, steady_clock::now());

                if (curr->children[0] != nullptr)
                    pushNode(curr->children[0]);
            }
            else  // We only generate another child node if back-tracking happens
            {
                if (screen > 1)
                    cout << "	Expand " << *curr << "	on " << *(curr->conflict) << endl;

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
                        clear();
                        stack<PBSNode*>().swap(open_list);  // clear the open_list
                        num_restart ++;
                        local_num_backtrack = 0;
                        break;  // leave the while loop of open_list.empty
                    }
                }
                curr->clear();
            }
        }  // end of while loop
    }
    return solution_found;
}

PBSNode* PBS2::selectNode()
{
	PBSNode* curr = open_list.top();
    update(curr);
    if (!curr->is_expanded)
    {
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;
    }
	if (screen > 1)
		cout << endl << "Select " << *curr << endl;
	return curr;
}

string PBS2::getSolverName() const
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

bool PBS2::generateRoot()
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
                int priority = hasConflicts(a1, a2);
                if (priority > 0)
                {
                    // We set agents with longer paths higher priorities
                    shared_ptr<Conflict> new_conflict;
                    if (paths[a1]->size() < paths[a2]->size())
                        new_conflict = make_shared<Conflict>(a1, a2, priority);
                    else
                        new_conflict = make_shared<Conflict>(a2, a1, priority);

                    // Prioritize to resolve target conflicts earlier others
                    if (priority == 1)
                        root->conflicts.push_front(new_conflict);
                    else if(priority == 2)
                        root->conflicts.push_back(new_conflict);
                }
            }
            else if (PBS::hasConflicts(a1, a2))  // not using target reasoning
            {
                if (paths[a1]->size() < paths[a2]->size())
                    root->conflicts.emplace_back(new Conflict(a1, a2));
                else
                    root->conflicts.emplace_back(new Conflict(a2, a1));
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

bool PBS2::generateChild(int child_id, PBSNode* parent, int low, int high)
{
    assert(child_id == 0 or child_id == 1);
    parent->children[child_id] = new PBSNode(*parent);
    auto node = parent->children[child_id];
    node->constraint.set(low, high);
    priority_graph[high][low] = false;
    priority_graph[low][high] = true;
    #ifndef NDEBUG
    if (screen > 2)
        printPriorityGraph();
    #endif

    topologicalSort(ordered_agents);
    if (screen > 2)
    {
        cout << "Ordered agents: ";
        for (int i : ordered_agents)
            cout << i << ",";
        cout << endl;
    }
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

    for (const auto & conflict : node->conflicts)
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
        if (screen > 2) cout << "Replan agent " << a << endl;
        // Re-plan path
        set<int> higher_agents;
        auto p = ordered_agents.rbegin();
        std::advance(p, rank);
        assert(*p == a);
        getHigherPriorityAgents(p, higher_agents);
        assert(!higher_agents.empty());
        if (screen > 2)
        {
            cout << "Higher agents: ";
            for (auto i : higher_agents)
                cout << i << ",";
            cout << endl;
        }
        Path new_path;
        if(!findPathForSingleAgent(*node, higher_agents, a, new_path))
        {
            delete node;
            parent->children[child_id] = nullptr;
            return false;
        }

        // Delete old conflicts
        for (auto c = node->conflicts.begin(); c != node->conflicts.end();)
        {
            if ((*c)->a1 == a or (*c)->a2 == a)
                c = node->conflicts.erase(c);
            else
                ++c;
        }

        // Update conflicts and to_replan
        set<int> lower_agents;
        auto p2 = ordered_agents.begin();
        std::advance(p2, num_of_agents - 1 - rank);
        assert(*p2 == a);
        getLowerPriorityAgents(p2, lower_agents);
        if (screen > 2 and !lower_agents.empty())
        {
            cout << "Lower agents: ";
            for (auto i : lower_agents)
                cout << i << ",";
            cout << endl;
        }

        // Find new conflicts
        for (auto a2 = 0; a2 < num_of_agents; a2++)
        {
            if (a2 == a or lookup_table[a2] or higher_agents.count(a2) > 0)
                continue;  // already in to_replan or has higher priority
            steady_clock::time_point t = steady_clock::now();

            if (use_tr)
            {
                int priority = hasConflicts(a, a2);
                if (priority > 0)  // there is a conflict
                {
                    if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                    {
                        if (screen > 1)
                            cout << "\tshould replan " << a2 << " for colliding with " << a << endl;
                        to_replan.emplace(topological_orders[a2], a2);
                        lookup_table[a2] = true;
                    }
                    else
                    {
                        shared_ptr<Conflict> new_conflict;
                        if (paths[a]->size() < paths[a2]->size())
                            new_conflict = make_shared<Conflict>(a, a2, priority);
                        else
                            new_conflict = make_shared<Conflict>(a2, a, priority);

                        // Prioritize to resolve target conflicts earlier others
                        if (priority == 1)
                            node->conflicts.push_front(new_conflict);
                        else if (priority == 2)  // Target conflict
                            node->conflicts.push_back(new_conflict);
                    }
                }
            }
            else if (PBS::hasConflicts(a, a2))
            {
                if (lower_agents.count(a2) > 0) // has a collision with a lower priority agent
                {
                    if (screen > 1)
                        cout << "\tshould replan " << a2 << " for colliding with " << a << endl;
                    to_replan.emplace(topological_orders[a2], a2);
                    lookup_table[a2] = true;
                }
                else
                {
                    if (paths[a]->size() < paths[a2]->size())
                        node->conflicts.emplace_back(new Conflict(a, a2));
                    else
                        node->conflicts.emplace_back(new Conflict(a2, a));
                }
            }
            runtime_detect_conflicts += getDuration(t, steady_clock::now());
        }
    }

    if (use_ic)  // Compute the implicit constraints
    {
        computeImplicitConstraints(node, topological_orders);
    }

    num_HL_generated++;
    node->time_generated = num_HL_generated;
    if (screen > 1)
        cout << "Generate " << *node << endl;
    return true;
}

int PBS2::hasConflicts(int a1, int a2) const
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
				return 2; // target conflict
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
            return 1;  // vertex or edge conflict
		}
	}
    return 0; // conflict-free
}

shared_ptr<Conflict> PBS2::chooseConflict(const PBSNode &node) const
{
    if (node.conflicts.empty())
        return nullptr;

    shared_ptr<Conflict> out = node.conflicts.back();

    if (use_tr and out->priority == 2)
        return out;

    if (use_ic)
    {
        for (const auto& conf : node.conflicts)
            if (conf->max_num_ic > out->max_num_ic)
                out = conf;
    }

    return out;
}

void PBS2::computeImplicitConstraints(PBSNode* node, const vector<int>& topological_orders)
{
    vector<set<int>> higher_pri_agents(num_of_agents);
    vector<set<int>> lower_pri_agents(num_of_agents);

    list<int>::reverse_iterator ag_rit;
    list<int>::iterator ag_it;

    steady_clock::time_point t = steady_clock::now();
    for (auto& conf : node->conflicts)
    {
        if (conf->priority == 2) continue;

        // Compute the additional implicit constraints for priority a1 -> a2
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

        uint num_ic_a1_a2 = (uint) ((higher_pri_agents[conf->a1].size()+1) * 
            (lower_pri_agents[conf->a2].size()+1));
        for (const int& h_ag : higher_pri_agents[conf->a1])
        {
            for (const int& l_ag : lower_pri_agents[conf->a2])
            {
                if (priority_graph[l_ag][h_ag])  // Reduce the IC that already exists
                {
                    num_ic_a1_a2 -= node->num_IC->at(l_ag).at(h_ag);
                }
            }
        }
        node->num_IC->at(conf->a2).at(conf->a1) = num_ic_a1_a2;
        double val_a1_a2 = ic_ratio * (double)num_ic_a1_a2 + 
            (1.0-ic_ratio) / (double)(lower_pri_agents[conf->a2].size()+1);

        // Compute the additional implicit constraints for priority a2 -> a1
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

        uint num_ic_a2_a1 = (uint) (higher_pri_agents[conf->a2].size()+1) * 
            (lower_pri_agents[conf->a1].size()+1);
        for (const int& h_ag : higher_pri_agents[conf->a2])
        {
            for (const int& l_ag : lower_pri_agents[conf->a1])
            {
                if (priority_graph[l_ag][h_ag])  // Reduce the IC that already exists
                {
                    num_ic_a2_a1 -= node->num_IC->at(l_ag).at(h_ag);
                }
            }
        }
        node->num_IC->at(conf->a1).at(conf->a2) = num_ic_a2_a1;
        double val_a2_a1 = ic_ratio * (double)num_ic_a2_a1 +
            (1.0-ic_ratio) / (double)(lower_pri_agents[conf->a1].size()+1);

        // Assign the maximum to the current conflict
        conf->max_num_ic = max(val_a1_a2, val_a2_a1);
        if (val_a1_a2 > val_a2_a1)
            std::swap(conf->a1, conf->a2);
    }
    runtime_implicit_constraints += getDuration(t, steady_clock::now());
}