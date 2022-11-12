#include "common.h"

std::ostream& operator<<(std::ostream& os, const Path& path)
{
	for (const auto& state : path)
	{
		os << state.location; // << "(" << state.is_single() << "),";
	}
	return os;
}


bool isSamePath(const Path& p1, const Path& p2)
{
	if (p1.size() != p2.size())
		return false;
	for (unsigned i = 0; i < p1.size(); i++)
	{
		if (p1[i].location != p2[i].location)
			return false;
	}
	return true;
}

void printPath(const Path& _path_)
{
    for (const PathEntry & t : _path_)
        cout << t.location << "->";
    cout << endl;
};

// utility comparator function to pass to the sort() module
bool sortByLongerPaths(const pair<int, size_t> &a, const pair<int, size_t> &b) 
{ 
    return (a.second > b.second); 
}
bool sortByShorterPaths(const pair<int, size_t> &a, const pair<int, size_t> &b) 
{ 
    return (a.second < b.second); 
} 