#pragma once
#include "common.h"

enum conflict_priority {NONE, NORMAL, TARGET};

struct Constraint
{
    int low = -1;
    int high = -1;
    void set(int _low, int _high)
    {
        low = _low;
        high = _high;
    }
};

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


struct Conflict
{
    // First constraint is a1<-a2 (a1 has lower priority than a2), and the second is a1->a2
    // Do this for partial expansion
	int a1;
	int a2;
    int priority;
    uint num_ic;  // Maximum number of implicit constraints between a1->a2 and a2->a2
    uint ll_calls;
    explicit Conflict(int a1, int a2, int priority=conflict_priority::NORMAL,
        uint num_ic=0, uint ll_calls=1):
        a1(a1), a2(a2), priority(priority), num_ic(num_ic), ll_calls(ll_calls) {}
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);

struct compare_conflicts
{
    bool operator()(const shared_ptr<Conflict> c1, const shared_ptr<Conflict> c2) const
    {
        if (c1->priority == c2->priority)
        {
            if (c1->num_ic == c2->num_ic)
            {
                if (c1->ll_calls == c2->ll_calls)
                {
                    return rand() % 2;
                }
                return c1->ll_calls > c2->ll_calls;
            }
            return c1->num_ic < c2->num_ic;
        }
        return c1->priority < c2->priority;
    }
};