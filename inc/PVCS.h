#pragma once
#include "PBS.h"

class PVCS : public PBS
{
public:
    PVCS(const Instance& instance, bool sipp, int screen, bool use_tr);
    bool solve(clock_t time_limit);

protected:
    bool use_tr;

    string getSolverName() const;

    // high-level search
    bool generateRoot();
    bool generateChild(int child_id, PBSNode* parent);
    PBSNode* selectNode();

    bool topologicalSort(list<int>& stack);  // Both functions returns true if there is a cycle
    bool topologicalSortUtil(int v, vector<bool>& visited, list<int>& stack, vector<bool>& onstack);

private:
    bool runWMVC(PBSNode* node);
    bool runTotalOrdering(PBSNode* node);
    int getLowerAgentNum(int agent, const vector<int>& topo_orders);
};