#pragma once
#include "PBS.h"

class PBS2 : public PBS
{
public:
    PBS2(const Instance& instance, int screen, bool sipp, bool is_ll_opt=false, bool use_tr=false,
        bool use_ic=false, bool use_rr=false,  uint64_t rr_th=0,
        double ic_ratio=1.0, bool use_LH=false, bool use_SH=false);
    bool solve(clock_t time_limit);

protected:
    bool use_tr;
    bool use_ic;
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