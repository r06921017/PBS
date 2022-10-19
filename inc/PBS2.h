#pragma once
#include "PBS.h"

class PBS2 : public PBS
{
public:
    PBS2(const Instance& instance, bool sipp, int scrren,
        bool use_tr, bool use_ic, bool use_rr, double ic_ratio=1.0);
    bool solve(clock_t time_limit);

protected:
    bool use_tr;
    bool use_ic;
    bool use_rr;
    double ic_ratio;

    string getSolverName() const;

    // high-level search
    bool generateRoot(void);
    bool generateChild(int child_id, PBSNode* parent, int low, int high);
    int hasConflicts(int a1, int a2) const;
	shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;
	PBSNode* selectNode();
    void computeImplicitConstraints(PBSNode* node, const vector<int>& topological_orders);
};