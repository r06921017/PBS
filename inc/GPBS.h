#pragma once
#include "PBS.h"

class GPBS : public PBS
{
public:
    GPBS(const Instance& instance, int screen, bool sipp, bool is_ll_opt=false, bool use_tr=false,
        bool use_ic=false, bool use_rr=false, uint64_t rr_th=0,
        bool use_LH=false, bool use_SH=false);
    bool solve(clock_t time_limit);

protected:
    bool use_tr;
    bool use_ic;

    string getSolverName(void) const;

	// high-level search
    bool generateRoot(void);
    PBSNode* generateRoot(const PBSNode* node);
    bool generateChild(int child_id, PBSNode* parent, int low, int high);
    conflict_priority hasConflicts(int a1, int a2) const;
	shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;
	PBSNode* selectNode(void);
    void computeImplicitConstraints(PBSNode* node, list<shared_ptr<Conflict>> conflicts,
        const vector<int>& topological_orders);
};