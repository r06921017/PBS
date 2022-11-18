#include "ConstraintTable.h"

int ConstraintTable::getMaxTimestep() const // everything is static after (\geq) the max timestep
{
    int rst = max(max(ct_max_timestep, cat_max_timestep), length_min+1);
    if (length_max < MAX_TIMESTEP)
        rst = max(rst, length_max+1);
    if (!landmarks.empty())
        rst = max(rst, landmarks.rbegin()->first+1);
    return rst;
}
int ConstraintTable::getLastCollisionTimestep(int location) const
{
    if (!cat.empty())
    {
        // #ifndef NDEBUG
        // cout << "cat[" << location << "].size(): " << cat[location].size() << endl;
        // #endif
        for (int t = (int) cat[location].size() - 1; t > -1; t--)
        {
            if (cat[location][t])
                return t;
        }
    }
    return -1;
}
void ConstraintTable::insert2CT(size_t from, size_t to, int t_min, int t_max)
{
    insert2CT(getEdgeIndex(from, to), t_min, t_max);
}
void ConstraintTable::insert2CT(size_t loc, int t_min, int t_max)
{
    assert(loc >= 0);
    ct[loc].emplace_back(t_min, t_max);
    if (t_max < MAX_TIMESTEP && t_max > ct_max_timestep)
    {
        ct_max_timestep = t_max;
    }
    else if (t_max == MAX_TIMESTEP && t_min > ct_max_timestep)
    {
        ct_max_timestep = t_min;
    }
}

void ConstraintTable::insert2CT(const Path& path)
{
    int prev_location = path.front().location;
    int prev_timestep = 0;
    for (int timestep = 0; timestep < (int) path.size(); timestep++)
    {
        auto curr_location = path[timestep].location;
        if (prev_location != curr_location)
        {
            insert2CT(prev_location, prev_timestep, timestep); // add vertex conflict
            insert2CT(curr_location, prev_location, timestep, timestep + 1); // add edge conflict
            prev_location = curr_location;
            prev_timestep = timestep;
        }
    }
    insert2CT(path.back().location, (int) path.size() - 1, MAX_TIMESTEP);
}

void ConstraintTable::insertLandmark(size_t loc, int t)
{
    auto it = landmarks.find(t);
    if (it == landmarks.end())
    {
        landmarks[t] = loc;
    }
    else
        assert(it->second == loc);
}

// build the conflict avoidance table
void ConstraintTable::insert2CAT(int agent, const vector<Path*>& paths)
{
    for (size_t ag = 0; ag < paths.size(); ag++)
    {
        if (ag == agent || paths[ag] == nullptr)
            continue;
        insert2CAT(*paths[ag], ag);
    }
}
void ConstraintTable::insert2CAT(const Path& path, int agent)
{
    if (cat.empty())
    {
        cat.resize(map_size);
        cat_goals.resize(map_size, MAX_TIMESTEP);
    }
    assert(cat_goals[path.back().location] == MAX_TIMESTEP);
    cat_goals[path.back().location] = path.size() - 1;
    for (auto timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep].location;
        if (cat[loc].size() <= timestep)
            cat[loc].resize(timestep + 1, false);
        cat[loc][timestep] = true;
    }
    cat_max_timestep = max(cat_max_timestep, (int)path.size());
}


// return the location-time pairs on the barrier in an increasing order of their timesteps
list<pair<int, int> > ConstraintTable::decodeBarrier(int x, int y, int t) const
{
    list<pair<int, int> > rst;
    int x1 = x / num_col, y1 = x % num_col;
    int x2 = y / num_col, y2 = y % num_col;
    if (x1 == x2)
    {
        if (y1 < y2)
            for (int i = min(y2 - y1, t); i>= 0; i--)
            {
                rst.emplace_back(x1 * num_col + y2 - i, t - i);
            }
        else
            for (int i = min(y1 - y2, t); i >= 0; i--)
            {
                rst.emplace_back(x1 * num_col + y2 + i, t - i);
            }
    }
    else // y1== y2
    {
        if (x1 < x2)
            for (int i = min(x2 - x1, t); i>= 0; i--)
            {
                rst.emplace_back((x2 - i) * num_col + y1, t - i);
            }
        else
            for (int i = min(x1 - x2, t); i>= 0; i--)
            {
                rst.emplace_back((x2 + i) * num_col + y1, t - i);
            }
    }
    return rst;
}

bool ConstraintTable::constrained(size_t loc, int t) const
{
    assert(loc >= 0);
    if (loc < map_size)
    {
        const auto& it = landmarks.find(t);
        if (it != landmarks.end() && it->second != loc)
            return true;  // violate the positive vertex constraint
    }

    const auto& it = ct.find(loc);
    if (it == ct.end())
    {
        return false;
    }
    for (const auto& constraint: it->second)
    {
        if (constraint.first <= t && t < constraint.second)
            return true;
    }
    return false;
}
bool ConstraintTable::constrained(size_t curr_loc, size_t next_loc, int next_t) const
{
    return constrained(getEdgeIndex(curr_loc, next_loc), next_t);
}

void ConstraintTable::copy(const ConstraintTable& other)
{
    length_min = other.length_min;
    length_max = other.length_max;
    num_col = other.num_col;
    map_size = other.map_size;
    ct = other.ct;
    ct_max_timestep = other.ct_max_timestep;
    cat = other.cat;
    cat_goals = other.cat_goals;
    cat_max_timestep = other.cat_max_timestep;
    landmarks = other.landmarks;
}


int ConstraintTable::getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const
{
    int rst = 0;
    if (!cat.empty())
    {
        if (cat[next_id].size() > next_timestep and cat[next_id][next_timestep])
            rst++;
        if (curr_id != next_id and cat[next_id].size() >= next_timestep and cat[curr_id].size() > next_timestep and
            cat[next_id][next_timestep - 1]and cat[curr_id][next_timestep])
            rst++;
        if (cat_goals[next_id] < next_timestep)
            rst++;  // target conflict
    }
    return rst;
}
bool ConstraintTable::hasConflictForStep(size_t curr_id, size_t next_id, int next_timestep) const
{
    if (!cat.empty())
    {
        if (cat[next_id].size() > next_timestep and cat[next_id][next_timestep])
            return true;
        if (curr_id != next_id and cat[next_id].size() >= next_timestep and cat[curr_id].size() > next_timestep and
            cat[next_id][next_timestep - 1]and cat[curr_id][next_timestep])
            return true;
        if (cat_goals[next_id] < next_timestep)
            return true; // target conflict
    }
    return false;
}
bool ConstraintTable::hasEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const
{
    assert(curr_id != next_id);
    return !cat.empty() and curr_id != next_id and cat[next_id].size() >= next_timestep and
           cat[curr_id].size() > next_timestep and
           cat[next_id][next_timestep - 1] and cat[curr_id][next_timestep];
}
int ConstraintTable::getFutureNumOfCollisions(int loc, int t) const
{
    int rst = 0;
    if (!cat.empty())
    {
        for (auto timestep = t + 1; timestep < cat[loc].size(); timestep++)
        {
            rst += (int)cat[loc][timestep];
        }
    }
    return rst;
}

// return the earliest timestep that the agent can hold the location
int ConstraintTable::getHoldingTime(int location, int earliest_timestep) const
{
    int rst = earliest_timestep;
    // CT
    auto it = ct.find(location);
    if (it != ct.end())
    {
        for (auto time_range : it->second)
            rst = max(rst, time_range.second);
    }
    // Landmark
    for (auto landmark : landmarks)
    {
        if (landmark.second != location)
            rst = max(rst, (int)landmark.first + 1);
    }

    return rst;
}

void ConstraintTable::printCT(void) const
{
    cout << "\n--- begin CT ---" << endl;
    cout << "\tmax_timestep: " << ct_max_timestep << endl;
    map<size_t, list< pair<int, int> > > tmp_ct(ct.begin(), ct.end());
    for (const auto& _ele_ : tmp_ct)
    {
        if (_ele_.first > map_size)
        {
            pair<size_t, size_t> cur_edge = getEdge(_ele_.first);
            cout << "\tloc:" << cur_edge.first << "->" << cur_edge.second;
        }
        else
        {
            cout << "\tloc:" << _ele_.first;
        }
        cout << ", time: ";
        for (const pair<int,int>& _t_ : _ele_.second)
        {
            cout << "[" << _t_.first << "," << _t_.second << ")";
            if (_t_ != _ele_.second.back())
                cout << ", ";
        }
        cout << endl;
    }
    cout << "---  end CT  ---" << endl;
}

void ConstraintTable::saveCT(const string &fileName) const
{
    ofstream stats(fileName, std::ios::app);
    stats << "--- begin CT ---" << endl;
    stats << "max_timestep: " << ct_max_timestep << endl;
    map<size_t, list< pair<int, int> > > tmp_ct(ct.begin(), ct.end());
    for (const auto& _ele_ : tmp_ct)
    {
        if (_ele_.first > map_size)
        {
            pair<size_t, size_t> cur_edge = getEdge(_ele_.first);
            stats << "\tloc:" << cur_edge.first << "->" << cur_edge.second;
        }
        else
        {
            stats << "\tloc:" << _ele_.first;
        }
        stats << ", time: ";
        for (const pair<int,int>& _t_ : _ele_.second)
        {
            stats << "[" << _t_.first << "," << _t_.second << ")";
            if (_t_ != _ele_.second.back())
                stats << ", ";
        }
        stats << endl;
    }
    stats << "---  end CT  ---" << endl;
	stats.close();
}

void ConstraintTable::printCAT(void) const
{
    cout << "\n--- begin CAT ---" << endl;
    cout << "\tmax_timestep: " << cat_max_timestep << endl;
    for (int loc=0; loc < cat.size(); loc++)
    {
        if (std::none_of(cat[loc].begin(), cat[loc].end(), [](bool v) { return v; }))
            continue;  // skip the location if there is no conflicting timestep
        
        if (loc > map_size)
        {
            pair<size_t, size_t> cur_edge = getEdge(loc);
            cout << "\tloc:" << cur_edge.first << "->" << cur_edge.second;
        }
        else
        {
            cout << "\tloc:" << loc;
        }
        cout << ", time: ";

        for (int _t_=0; _t_ < cat[loc].size(); _t_++)
        {
            if (cat[loc][_t_])
            {
                cout << _t_;
                if (_t_ < cat[loc].size()-1) cout << ", ";
            }
        }
        cout << endl;
    }
    cout << "---  end CAT  ---" << endl;
}

void ConstraintTable::saveCAT(const string& fileName) const
{
    ofstream stats(fileName, std::ios::app);
    stats << "--- begin CAT ---" << endl;
    stats << "max_timestep," << cat_max_timestep << endl;
    for (int loc=0; loc < cat.size(); loc++)
    {
        if (std::none_of(cat[loc].begin(), cat[loc].end(), [](bool v) { return v; }))
            continue;  // skip the location if there is no conflicting timestep

        if (loc > map_size)
        {
            pair<size_t, size_t> cur_edge = getEdge(loc);
            stats << "\tloc:" << cur_edge.first << "->" << cur_edge.second;
        }
        else
        {
            stats << "\tloc:" << loc;
        }
        stats << ", time: ";

        for (int _t_=0; _t_ < cat[loc].size(); _t_++)
        {
            if (cat[loc][_t_])
            {
                stats << _t_;
                if (_t_ < cat[loc].size()-1) stats << ", ";
            }
        }
        stats << endl;
    }
    stats << "---  end CAT  ---" << endl;
}
