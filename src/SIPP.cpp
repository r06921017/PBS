#include "SIPP.h"

void SIPP::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    //num_collisions = goal->num_of_conflicts;
    path.resize(goal->timestep + 1);
    // num_of_conflicts = goal->num_of_conflicts;

    const auto* curr = goal;
    while (curr->parent != nullptr) // non-root node
    {
        const auto* prev = curr->parent;
        int t = prev->timestep + 1;
        while (t < curr->timestep)
        {
            path[t].location = prev->location; // wait at prev location
            t++;
        }
        path[curr->timestep].location = curr->location; // move to curr location
        curr = prev;
    }
    assert(curr->timestep == 0);
    path[0].location = curr->location;
}

Path SIPP::findOptimalPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent)
{
    reset();

    // build constraint table
    steady_clock::time_point t = steady_clock::now();
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
    for (int a : higher_agents)
    {
        constraint_table.insert2CT(*paths[a]);
    }
    runtime_build_CT = getDuration(t, steady_clock::now());

    int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);

    t = steady_clock::now();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = getDuration(t, steady_clock::now());

    // build reservation table
    ReservationTable reservation_table(constraint_table, goal_location);

    Path path;
    num_expanded = 0;
    num_generated = 0;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;

    // generate start and add it to the OPEN list
    int h = max(my_heuristic[start_location], holding_time);
    SIPPNode* start = new SIPPNode(start_location, 0, h, nullptr, 0,
        get<1>(interval), get<1>(interval), get<2>(interval), get<2>(interval));
    min_f_val = max(holding_time, (int)start->getFVal());
    pushNodeToOpen(start);

    t = steady_clock::now();
    while (!open_list.empty())
    {
        SIPPNode* curr = open_list.top(); open_list.pop();
        curr->in_openlist = false;
        num_expanded++;

        // check if the popped node is a goal node
        if (curr->location == goal_location && // arrive at the goal location
            !curr->wait_at_goal && // not wait at the goal location
            curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            updatePath(curr, path);
            break;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (tuple<int, int, int, bool, bool> & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                // compute cost to next_id via curr node
                int next_g_val = next_timestep;
                int next_h_val = max(my_heuristic[next_location], curr->getFVal() - next_g_val);  // path max
                if (next_g_val + next_h_val > reservation_table.constraint_table.length_max)
                    continue;
                int next_conflicts = curr->num_of_conflicts +
                                     (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) +
                                     + (int)next_v_collision + (int)next_e_collision;
                SIPPNode* next = new SIPPNode(next_location, next_g_val, next_h_val, curr, next_timestep,
                                        next_high_generation, next_high_expansion, next_v_collision, next_conflicts);
                if (dominanceCheck(next))
                    pushNodeToOpen(next);
                else
                    delete next;
            }
        }  // end for loop that generates successors

        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
            get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            int next_timestep = get<0>(interval);
            int next_h_val = max(curr->h_val, curr->getFVal() - next_timestep);  // path max
            int next_collisions = curr->num_of_conflicts +
                                (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) // wait time
                                + (int)get<2>(interval);
            SIPPNode* next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, next_timestep,
                                     get<1>(interval), get<1>(interval), get<2>(interval), next_collisions);
            if (curr->location == goal_location)
                next->wait_at_goal = true;
            if (dominanceCheck(next))
                pushNodeToOpen(next);
            else
                delete next;
        }
    }  // end while loop
    runtime = getDuration(t, steady_clock::now());

    // no path found
    releaseNodes();
    return path;
}

Path SIPP::findOptimalPath(const ConstraintTable& constraint_table)
{
    reset();

    // holding_time is the earliest time that the agent can hold its goal location 
    int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);

    // build reservation table
    ReservationTable reservation_table(constraint_table, goal_location);

    Path path;
    num_expanded = 0;
    num_generated = 0;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;

    // generate start and add it to the OPEN list
    int h = max(my_heuristic[start_location], holding_time);
    SIPPNode* start = new SIPPNode(start_location, 0, h, nullptr, 0,
                            get<1>(interval), get<1>(interval), get<2>(interval), get<2>(interval));
    min_f_val = max(holding_time, (int)start->getFVal());
    pushNodeToOpen(start);

    steady_clock::time_point t = steady_clock::now();
    while (!open_list.empty())
    {
        auto curr = open_list.top(); open_list.pop();
        curr->in_openlist = false;
        num_expanded++;

        // check if the popped node is a goal node
        if (curr->location == goal_location && // arrive at the goal location
            !curr->wait_at_goal && // not wait at the goal location
            curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            updatePath(curr, path);
            break;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (auto & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                // compute cost to next_id via curr node
                int next_g_val = next_timestep;
                int next_h_val = max(my_heuristic[next_location], curr->getFVal() - next_g_val);  // path max
                if (next_g_val + next_h_val > reservation_table.constraint_table.length_max)
                    continue;
                int next_conflicts = curr->num_of_conflicts +
                                     (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) +
                                     + (int)next_v_collision + (int)next_e_collision;
                SIPPNode* next = new SIPPNode(next_location, next_g_val, next_h_val, curr, next_timestep,
                                        next_high_generation, next_high_expansion, next_v_collision, next_conflicts);
                if (dominanceCheck(next))
                    pushNodeToOpen(next);
                else
                    delete next;
            }
        }  // end for loop that generates successors

        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
            get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            int next_timestep = get<0>(interval);
            int next_h_val = max(curr->h_val, curr->getFVal() - next_timestep);  // path max
            int next_collisions = curr->num_of_conflicts +
                                   (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) // wait time
                                   + (int)get<2>(interval);
            SIPPNode* next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, next_timestep,
                                    get<1>(interval), get<1>(interval), get<2>(interval), next_collisions);
            if (curr->location == goal_location)
                next->wait_at_goal = true;
            if (dominanceCheck(next))
                pushNodeToOpen(next);
            else
                delete next;
        }
    }  // end while loop
    runtime = getDuration(t, steady_clock::now());

    // no path found
    releaseNodes();
    return path;
}

Path SIPP::findPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent)
{
    reset();

    // build constraint table
    steady_clock::time_point t = steady_clock::now();
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
    for (int a : higher_agents)
    {
        constraint_table.insert2CT(*paths[a]);
    }
    runtime_build_CT = getDuration(t, steady_clock::now());

    t = steady_clock::now();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = getDuration(t, steady_clock::now());
    // #ifndef NDEBUG
    // constraint_table.printCT();
    // constraint_table.printCAT();
    // #endif

    // build reservation table
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;

    int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    int last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
    // generate start and add it to the OPEN list
    int h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time+1);
    SIPPNode* start = new SIPPNode(start_location, 0, h, nullptr, 0,
        get<1>(interval), get<1>(interval), get<2>(interval), get<2>(interval));
    pushNodeToFocal(start);

    t = steady_clock::now();
    while (!focal_list.empty())
    {
        SIPPNode* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // #ifndef NDEBUG
        // cout << "Expand: " << *curr << endl;
        // #endif

        // check if the popped node is a goal
        if (curr->is_goal)
        {
            updatePath(curr, path);
            break;
        }
        else if (curr->location == goal_location && // arrive at the goal location
                 !curr->wait_at_goal && // not wait at the goal location
                 curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            if (future_collisions == 0)
            {
                updatePath(curr, path);
                // #ifndef NDEBUG
                // printPath(path);
                // #endif
                break;
            }
            // generate a goal node
            SIPPNode* goal = new SIPPNode(*curr);
            goal->is_goal = true;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            if (dominanceCheck(goal))
                pushNodeToFocal(goal);
            else
                delete goal;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (tuple<int, int, int, bool, bool> & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                if (next_timestep + my_heuristic[next_location] > constraint_table.length_max)
                    break;
                int next_collisions = curr->num_of_conflicts +
                                    (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) // wait time
                                      + (int)next_v_collision + (int)next_e_collision;
                int next_h_val = max(my_heuristic[next_location], (next_collisions > 0?
                    holding_time : curr->getFVal()) - next_timestep); // path max
                // generate (maybe temporary) node
                SIPPNode* next = new SIPPNode(next_location, next_timestep, next_h_val, curr, 
                        next_timestep, next_high_generation, next_high_expansion, 
                        next_v_collision, next_collisions);
                // #ifndef NDEBUG
                // cout << "Move -> generate: " << *next;
                // #endif

                // try to retrieve it from the hash table
                if (dominanceCheck(next))
                {
                    // #ifndef NDEBUG
                    // cout << " add to focal" << endl;
                    // #endif
                    pushNodeToFocal(next);
                }
                else
                {
                    // #ifndef NDEBUG
                    // cout << " remove" << endl;
                    // #endif
                    delete next;
                }
            }
        }  // end for loop that generates successors
        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
                get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            int next_timestep = get<0>(interval);
            int next_h_val = max(curr->h_val, (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
            int next_collisions = curr->num_of_conflicts +
                    (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) + (int)get<2>(interval);
            SIPPNode* next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, 
                next_timestep, get<1>(interval), get<1>(interval), get<2>(interval), next_collisions);
            next->wait_at_goal = (curr->location == goal_location);
            // #ifndef NDEBUG
            // cout << "Wait -> generate " << *next;
            // #endif

            if (dominanceCheck(next))
            {
                // #ifndef NDEBUG
                // cout << " add to focal" << endl;
                // #endif
                pushNodeToFocal(next);
            }
            else
            {
                // #ifndef NDEBUG
                // cout << " remove" << endl;
                // #endif
                delete next;
            }
        }
    }  // end while loop
    runtime = getDuration(t, steady_clock::now());

    // #ifndef NDEBUG
    // printSearchTree();
    // #endif
    //if (path.empty())
    //{
    //    printSearchTree();
    //}
    releaseNodes();
    return path;
}

// find path by SIPP
// Returns a path that minimizes the collisions with the paths in the path table, breaking ties by the length
Path SIPP::findPath(const ConstraintTable& constraint_table)
{
    reset();
    //Path path = findNoCollisionPath(constraint_table);
    //if (!path.empty())
    //    return path;
    // #ifndef NDEBUG
    // constraint_table.printCT();
    // constraint_table.printCAT();
    // #endif

    // build reservation table
    ReservationTable reservation_table(constraint_table, goal_location);
    Path path;
    Interval interval = reservation_table.get_first_safe_interval(start_location);
    if (get<0>(interval) > 0)
        return path;
    int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    int last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
    // generate start and add it to the OPEN & FOCAL list
    int h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time+1);
    SIPPNode* start = new SIPPNode(start_location, 0, h, nullptr, 0, 
        get<1>(interval), get<1>(interval), get<2>(interval), get<2>(interval));
    pushNodeToFocal(start);

    steady_clock::time_point t = steady_clock::now();
    while (!focal_list.empty())
    {
        SIPPNode* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // #ifndef NDEBUG
        // cout << "Expand: " << *curr << endl;
        // #endif

        // check if the popped node is a goal
        if (curr->is_goal)
        {
            updatePath(curr, path);
            break;
        }
        else if (curr->location == goal_location && // arrive at the goal location
                 !curr->wait_at_goal && // not wait at the goal location
                 curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            if (future_collisions == 0)
            {
                updatePath(curr, path);
                // #ifndef NDEBUG
                // printPath(path);
                // #endif
                break;
            }
            // generate a goal node
            SIPPNode* goal = new SIPPNode(*curr);
            goal->is_goal = true;
            goal->h_val = 0;
            goal->num_of_conflicts += future_collisions;
            // try to retrieve it from the hash table
            if (dominanceCheck(goal))
                pushNodeToFocal(goal);
            else
                delete goal;
        }

        for (int next_location : instance.getNeighbors(curr->location)) // move to neighboring locations
        {
            for (tuple<int, int, int, bool, bool> & i : reservation_table.get_safe_intervals(
                    curr->location, next_location, curr->timestep + 1, curr->high_expansion + 1))
            {
                int next_high_generation, next_timestep, next_high_expansion;
                bool next_v_collision, next_e_collision;
                tie(next_high_generation, next_timestep, next_high_expansion, next_v_collision, next_e_collision) = i;
                if (next_timestep + my_heuristic[next_location] > constraint_table.length_max)
                    break;
                int next_collisions = curr->num_of_conflicts +
                                    (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) // wait time
                                      + (int)next_v_collision + (int)next_e_collision;
                int next_h_val = max(my_heuristic[next_location], (next_collisions > 0?
                    holding_time : curr->getFVal()) - next_timestep); // path max
                // generate (maybe temporary) node
                SIPPNode* next = new SIPPNode(next_location, next_timestep, next_h_val, curr, 
                        next_timestep, next_high_generation, next_high_expansion, 
                        next_v_collision, next_collisions);
                // #ifndef NDEBUG
                // cout << "Move -> generate: " << *next;
                // #endif

                // try to retrieve it from the hash table
                if (dominanceCheck(next))
                {
                    // #ifndef NDEBUG
                    // cout << " add to focal" << endl;
                    // #endif
                    pushNodeToFocal(next);
                }
                else
                {
                    // #ifndef NDEBUG
                    // cout << " remove" << endl;
                    // #endif
                    delete next;
                }
            }
        }  // end for loop that generates successors
        // wait at the current location
        if (curr->high_expansion == curr->high_generation and
            reservation_table.find_safe_interval(interval, curr->location, curr->high_expansion) and
                get<0>(interval) + curr->h_val <= reservation_table.constraint_table.length_max)
        {
            int next_timestep = get<0>(interval);
            int next_h_val = max(curr->h_val, (get<2>(interval) ? holding_time : curr->getFVal()) - next_timestep); // path max
            int next_collisions = curr->num_of_conflicts +
                    (int)curr->collision_v * max(next_timestep - curr->timestep - 1, 0) + (int)get<2>(interval);
            SIPPNode* next = new SIPPNode(curr->location, next_timestep, next_h_val, curr, 
                next_timestep, get<1>(interval), get<1>(interval), get<2>(interval), next_collisions);
            next->wait_at_goal = (curr->location == goal_location);
            // #ifndef NDEBUG
            // cout << "Wait -> generate: " << *next;
            // #endif

            if (dominanceCheck(next))
            {
                // #ifndef NDEBUG
                // cout << " add to focal" << endl;
                // #endif
                pushNodeToFocal(next);
            }
            else
            {
                // #ifndef NDEBUG
                // cout << " remove" << endl;
                // #endif
                delete next;
            }
        }
    }  // end while loop
    runtime = getDuration(t, steady_clock::now());

    // #ifndef NDEBUG
    // printSearchTree();
    // #endif

    //if (path.empty())
    //{
    //    printSearchTree();
    //}
    releaseNodes();
    return path;
}

void SIPP::updateFocalList()
{
    SIPPNode* open_head = open_list.top();
    if (open_head->getFVal() > min_f_val)
    {
        int new_min_f_val = (int) open_head->getFVal();
        for (SIPPNode* n : open_list)
        {
            if (n->getFVal() > w * min_f_val && n->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}
inline void SIPP::pushNodeToOpen(SIPPNode* node)
{
    num_generated++;
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    allNodes_table[node].push_back(node);
}
inline void SIPP::pushNodeToOpenAndFocal(SIPPNode* node)
{
    num_generated++;
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    if (node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
    allNodes_table[node].push_back(node);
}
inline void SIPP::pushNodeToFocal(SIPPNode* node)
{
    num_generated++;
    allNodes_table[node].push_back(node);
    node->in_openlist = true;
    node->focal_handle = focal_list.push(node); // we only use focal list; no open list is used
}
inline void SIPP::eraseNodeFromLists(SIPPNode* node)
{
    if (open_list.empty())
    { // we only have focal list
        focal_list.erase(node->focal_handle);
    }
    else if (focal_list.empty())
    {  // we only have open list
        open_list.erase(node->open_handle);
    }
    else
    { // we have both open and focal
        open_list.erase(node->open_handle);
        if (node->getFVal() <= w * min_f_val)
            focal_list.erase(node->focal_handle);
    }
}
void SIPP::releaseNodes()
{
    if (num_generated > 0)
    {
        accumulated_num_expanded += num_expanded;
        accumulated_num_generated += num_generated;
        accumulated_num_reopened += num_reopened;
        num_runs++;
    }

    open_list.clear();
    focal_list.clear();
    for (auto & node_list : allNodes_table)
        for (SIPPNode* n : node_list.second)
            delete n;
    allNodes_table.clear();
    for (auto n : useless_nodes)
        delete n;
    useless_nodes.clear();
}

// return true iff we the new node is not dominated by any old node
bool SIPP::dominanceCheck(SIPPNode* new_node)
{
    auto ptr = allNodes_table.find(new_node);
    if (ptr == allNodes_table.end())
        return true;
    for (auto & old_node : ptr->second)
    {
        if (old_node->timestep <= new_node->timestep and
            old_node->num_of_conflicts <= new_node->num_of_conflicts)
        { // the new node is dominated by the old node
            return false;
        }
        else if (old_node->timestep >= new_node->timestep and
                 old_node->num_of_conflicts >= new_node->num_of_conflicts) // the old node is dominated by the new node
        { // delete the old node
            if (old_node->in_openlist) // the old node has not been expanded yet
                eraseNodeFromLists(old_node); // delete it from open and/or focal lists
            useless_nodes.push_back(old_node);
            ptr->second.remove(old_node);
            num_generated--; // this is because we later will increase num_generated when we insert the new node into lists.
            return true;
        }
        else if(old_node->timestep < new_node->high_expansion and new_node->timestep < old_node->high_expansion)
        { // intervals overlap --> we need to split the node to make them disjoint
            if (old_node->timestep <= new_node->timestep)
            {
                assert(old_node->num_of_conflicts > new_node->num_of_conflicts);
                old_node->high_expansion = new_node->timestep;
            }
            else // i.e., old_node->timestep > new_node->timestep
            {
                assert(old_node->num_of_conflicts <= new_node->num_of_conflicts);
                new_node->high_expansion = old_node->timestep;
            }
        }
    }
    return true;
}

std::ostream& operator<<(std::ostream& os, const SIPPNode& node)
{
    os << node.location << "@" << node.timestep << "(f=" << node.g_val << "+" << node.h_val << ", c=" 
        << node.num_of_conflicts << ")";
    return os;
}

void SIPP::printSearchTree() const
{
    vector<list<SIPPNode*>> nodes;
    for (const auto & node_list : allNodes_table)
    {
        for (const auto & n : node_list.second)
        {
            if (nodes.size() <= n->timestep)
                nodes.resize(n->timestep + 1);
            nodes[n->timestep].emplace_back(n);
        }
    }
    cout << "Search Tree" << endl;
    for(int t = 0; t < nodes.size(); t++)
    {
        cout << "t=" << t << ":\t";
        for (const auto & n : nodes[t])
            cout << *n << "[" << n->timestep << "," << n->high_expansion << "),c=" << n->num_of_conflicts << "\t";
        cout << endl;
    }
}