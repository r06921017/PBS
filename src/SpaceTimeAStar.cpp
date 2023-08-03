#include "SpaceTimeAStar.h"


void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    const LLNode* curr = goal;
    if (curr->is_goal)
        curr = curr->parent;
    path.reserve(curr->g_val + 1);
    while (curr != nullptr)
    {
        path.emplace_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
}


Path SpaceTimeAStar::findOptimalPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent)
{
    reset();
    optimal = true;
    Path path;

    // build constraint table
    steady_clock::time_point t = steady_clock::now();
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
    for (int a : higher_agents)
    {
        constraint_table.insert2CT(*paths[a]);
    }
    runtime_build_CT = getDuration(t, steady_clock::now());

    if (constraint_table.constrained(start_location, 0))
    {
        return path;
    }

    t = steady_clock::now();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = getDuration(t, steady_clock::now());

    // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    auto static_timestep = constraint_table.getMaxTimestep()+1; // everything is static after this timestep

    // generate start and add it to the OPEN & FOCAL list
    auto start = new AStarNode(start_location, 0, max(holding_time, my_heuristic[start_location]), nullptr, 0, 0);

    num_generated++;
    start->open_handle = open_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);

    t = steady_clock::now();
    while (!open_list.empty())
    {
        auto* curr = popNode();
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->location == goal_location && // arrive at the goal location
            !curr->wait_at_goal && // not wait at the goal location
            curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            updatePath(curr, path);
            break;
        }

        if (curr->timestep >= constraint_table.length_max)
            continue;

        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            { // now everything is static, so switch to space A* where we always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            if (constraint_table.constrained(next_location, next_timestep) ||
                constraint_table.constrained(curr->location, next_location, next_timestep))
                continue;

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = max(holding_time - next_g_val, my_heuristic[next_location]);
            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;
            int next_internal_conflicts = curr->num_of_conflicts +
                                          constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep, next_internal_conflicts);
            //if (next_location == goal_location && curr->location == goal_location)
            //    next->wait_at_goal = true;

            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;
            if (existing_next->getFVal() > next->getFVal() or  // it has smaller f value
                    (existing_next->getFVal() == next->getFVal() and existing_next->num_of_conflicts > next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
            {
                existing_next->copy(*next);	// update existing node
                if (!existing_next->in_openlist) // if its in the closed list (reopen)
                {
                    pushNode(existing_next);
                }
                else
                {
                   open_list.increase(existing_next->open_handle);  // increase because #conflicts improved
                }
            }
            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    runtime = getDuration(t, steady_clock::now());
    releaseNodes();
    return path;
}

// This function is for PP, so we don't need to worry about CAT
Path SpaceTimeAStar::findOptimalPath(const ConstraintTable& constraint_table)
{
    reset();
    optimal = true;
    Path path;

    // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    auto static_timestep = constraint_table.getMaxTimestep()+1; // everything is static after this timestep

    // generate start and add it to the OPEN & FOCAL list
    auto start = new AStarNode(start_location, 0, max(holding_time, my_heuristic[start_location]), nullptr, 0, 0);

    num_generated++;
    start->open_handle = open_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);

    steady_clock::time_point t = steady_clock::now();
    while (!open_list.empty())
    {
        auto* curr = popNode();
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->location == goal_location && // arrive at the goal location
            !curr->wait_at_goal && // not wait at the goal location
            curr->timestep >= holding_time) // the agent can hold the goal location afterward
        {
            updatePath(curr, path);
            break;
        }

        if (curr->timestep >= constraint_table.length_max)
            continue;

        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            { // now everything is static, so switch to space A* where we always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            if (constraint_table.constrained(next_location, next_timestep) ||
                constraint_table.constrained(curr->location, next_location, next_timestep))
                continue;

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = max(holding_time - next_g_val, my_heuristic[next_location]);
            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;
            int next_internal_conflicts = curr->num_of_conflicts +
                                          constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep, next_internal_conflicts);
            //if (next_location == goal_location && curr->location == goal_location)
            //    next->wait_at_goal = true;

            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;
            if (existing_next->getFVal() > next->getFVal() or  // it has smaller f value
                    (existing_next->getFVal() == next->getFVal() and existing_next->num_of_conflicts > next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
            {
                existing_next->copy(*next);	// update existing node
                if (!existing_next->in_openlist) // if its in the closed list (reopen)
                {
                    pushNode(existing_next);
                }
                else
                {
                   open_list.increase(existing_next->open_handle);  // increase because #conflicts improved
                }
            }
            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    runtime = getDuration(t, steady_clock::now());
    releaseNodes();
    return path;
}


Path SpaceTimeAStar::findPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent)
{
    reset();
    Path path;

    // build constraint table
    steady_clock::time_point t = steady_clock::now();
    ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);
    for (int a : higher_agents)
    {
        constraint_table.insert2CT(*paths[a]);
    }
    runtime_build_CT = getDuration(t, steady_clock::now());

    if (constraint_table.constrained(start_location, 0))
    {
        return path;
    }

    t = steady_clock::now();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = getDuration(t, steady_clock::now());

    // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
    int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    int static_timestep = constraint_table.getMaxTimestep()+1; // everything is static after this timestep
    int last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);

    // generate start and add it to the OPEN & FOCAL list
    int h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time + 1);
    AStarNode* start = new AStarNode(start_location, 0, h, nullptr, 0, 0);
    num_generated++;
    start->in_openlist = true;
    start->focal_handle = focal_list.push(start); // we only use focal list; no open list is used
    allNodes_table.insert(start);

    t = steady_clock::now();
    while (!focal_list.empty())
    {
        auto* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal)
        {
            updatePath(curr, path);
            break;
        }
        else if (curr->location == goal_location and // arrive at the goal location
                 !curr->wait_at_goal and // not wait at the goal location
                 curr->timestep >= holding_time) // the agent can hold the goal location afterward
         {
            int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            if (future_collisions == 0)
            {
                updatePath(curr, path);
                break;
            }
            // generate a goal node
            AStarNode* goal = new AStarNode(*curr);
            goal->is_goal = true;
            goal->parent = curr;
            goal->num_of_conflicts += future_collisions;
            goal->h_val = 0;
            // try to retrieve it from the hash table
            hashtable_t::const_iterator it = allNodes_table.find(goal);
            if (it == allNodes_table.end())
            {
                goal->focal_handle = focal_list.push(goal);
                goal->in_openlist = true;
                num_generated++;
                allNodes_table.insert(goal);
            }
            else // update existing node's if needed (only in the open_list)
            {
                AStarNode* existing_next = *it;
                if (existing_next->num_of_conflicts > goal->num_of_conflicts ||
                   (existing_next->num_of_conflicts == goal->num_of_conflicts &&
                    existing_next->getFVal() > goal->getFVal()))
                {
                    assert(existing_next->in_openlist);
                    existing_next->copy(*goal);	// update existing node
                    focal_list.update(existing_next->focal_handle);
                    num_generated++; // reopen is considered as a new node
                }
                delete (goal);
            }
        }
        if (curr->timestep >= constraint_table.length_max)
            continue;

        list<int> next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            { // now everything is static, so switch to space A* where we always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            if (constraint_table.constrained(next_location, next_timestep) ||
                constraint_table.constrained(curr->location, next_location, next_timestep))
                continue;

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = my_heuristic[next_location];
            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;
            int num_conflicts = curr->num_of_conflicts +
                    constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);
            if (num_conflicts == 0)
                next_h_val = max(next_h_val, curr->getFVal() - next_g_val);  // path max
            else
                next_h_val = max(next_h_val, holding_time - next_g_val); // path max
            // generate (maybe temporary) node
            AStarNode* next = new AStarNode(next_location, next_g_val, next_h_val,
                                            curr, next_timestep, num_conflicts);

            if (next_location == goal_location && curr->location == goal_location)
                next->wait_at_goal = true;

            // try to retrieve it from the hash table
            hashtable_t::const_iterator it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                next->focal_handle = focal_list.push(next);
                next->in_openlist = true;
                num_generated++;
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            AStarNode* existing_next = *it;
            if (existing_next->num_of_conflicts > next->num_of_conflicts  ||
                (existing_next->num_of_conflicts == next->num_of_conflicts &&
                existing_next->getFVal() > next->getFVal()))
            {
                if (!existing_next->in_openlist) // if its in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    existing_next->focal_handle = focal_list.push(existing_next);
                    existing_next->in_openlist = true;
                    num_generated++; // reopen is considered as a new node
                }
                else
                {
                    existing_next->copy(*next);	// update existing node
                    focal_list.update(existing_next->focal_handle);
                }
            }

            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    runtime = getDuration(t, steady_clock::now());
    releaseNodes();
    return path;
}


// find path by time-space A* search
// Returns a path that minimizes the collisions with the paths in the path table, breaking ties by the length
Path SpaceTimeAStar::findPath(const ConstraintTable& constraint_table)
{
    reset();
    Path path;
    if (constraint_table.constrained(start_location, 0))
    {
        return path;
    }

    // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
    int holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
    int static_timestep = constraint_table.getMaxTimestep()+1; // everything is static after this timestep
    int last_target_collision_time = constraint_table.getLastCollisionTimestep(goal_location);
    // generate start and add it to the OPEN & FOCAL list
    int h = max(max(my_heuristic[start_location], holding_time), last_target_collision_time + 1);
    AStarNode* start = new AStarNode(start_location, 0, h, nullptr, 0, 0);
    num_generated++;
    start->in_openlist = true;
    start->focal_handle = focal_list.push(start); // we only use focal list; no open list is used
    allNodes_table.insert(start);

    steady_clock::time_point t = steady_clock::now();
    while (!focal_list.empty())
    {
        AStarNode* curr = focal_list.top();
        focal_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->is_goal)
        {
            updatePath(curr, path);
            break;
        }
        else if (curr->location == goal_location and // arrive at the goal location
                 !curr->wait_at_goal and // not wait at the goal location
                 curr->timestep >= holding_time) // the agent can hold the goal location afterward
         {
            int future_collisions = constraint_table.getFutureNumOfCollisions(curr->location, curr->timestep);
            if (future_collisions == 0)
            {
                updatePath(curr, path);
                break;
            }
            // generate a goal node
            AStarNode* goal = new AStarNode(*curr);
            goal->is_goal = true;
            goal->parent = curr;
            goal->num_of_conflicts += future_collisions;
            goal->h_val = 0;
            // try to retrieve it from the hash table
            hashtable_t::const_iterator it = allNodes_table.find(goal);
            if (it == allNodes_table.end())
            {
                goal->focal_handle = focal_list.push(goal);
                goal->in_openlist = true;
                num_generated++;
                allNodes_table.insert(goal);
            }
            else // update existing node's if needed (only in the open_list)
            {
                AStarNode* existing_next = *it;
                if (existing_next->num_of_conflicts > goal->num_of_conflicts ||
                   (existing_next->num_of_conflicts == goal->num_of_conflicts &&
                    existing_next->getFVal() > goal->getFVal()))
                {
                    assert(existing_next->in_openlist);
                    existing_next->copy(*goal);	// update existing node
                    focal_list.update(existing_next->focal_handle);
                    num_generated++; // reopen is considered as a new node
                }
                delete (goal);
            }
        }
        if (curr->timestep >= constraint_table.length_max)
            continue;

        list<int> next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            { // now everything is static, so switch to space A* where we always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }

            if (constraint_table.constrained(next_location, next_timestep) ||
                constraint_table.constrained(curr->location, next_location, next_timestep))
                continue;

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = my_heuristic[next_location];
            if (next_g_val + next_h_val > constraint_table.length_max)
                continue;
            int num_conflicts = curr->num_of_conflicts +
                    constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);
            if (num_conflicts == 0)
                next_h_val = max(next_h_val, curr->getFVal() - next_g_val);  // path max
            else
                next_h_val = max(next_h_val, holding_time - next_g_val); // path max
            // generate (maybe temporary) node
            AStarNode* next = new AStarNode(next_location, next_g_val, next_h_val,
                                            curr, next_timestep, num_conflicts);

            if (next_location == goal_location && curr->location == goal_location)
                next->wait_at_goal = true;

            // try to retrieve it from the hash table
            hashtable_t::const_iterator it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                next->focal_handle = focal_list.push(next);
                next->in_openlist = true;
                num_generated++;
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            AStarNode* existing_next = *it;
            if (existing_next->num_of_conflicts > next->num_of_conflicts  ||
                (existing_next->num_of_conflicts == next->num_of_conflicts &&
                existing_next->getFVal() > next->getFVal()))
            {
                if (!existing_next->in_openlist) // if its in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    existing_next->focal_handle = focal_list.push(existing_next);
                    existing_next->in_openlist = true;
                    num_generated++; // reopen is considered as a new node
                }
                else
                {
                    existing_next->copy(*next);	// update existing node
                    focal_list.update(existing_next->focal_handle);
                }
            }

            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    runtime = getDuration(t, steady_clock::now());
    releaseNodes();
    return path;
}


inline AStarNode* SpaceTimeAStar::popNode()
{
    AStarNode* node;
    if (optimal)
    {
        node = open_list.top(); open_list.pop();
    }
    else
    {
        node = focal_list.top(); focal_list.pop();
        assert(node->in_openlist);
        open_list.erase(node->open_handle);
    }
    node->in_openlist = false;
    num_expanded++;
    return node;
}


inline void SpaceTimeAStar::pushNode(AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
    if (!optimal and node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
}


void SpaceTimeAStar::updateFocalList()
{
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val)
    {
        int new_min_f_val = open_head->getFVal();
        for (auto n : open_list)
        {
            if (n->getFVal() > w * min_f_val && n->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}


void SpaceTimeAStar::releaseNodes()
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
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
}
