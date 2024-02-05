
#ifndef heuristics_hpp
#define heuristics_hpp

#include "Types.h"
#include "utils.hpp"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"

namespace TrafficMAPF{

void init_flow_heuristic(TrajLNS& lns, std::vector<int>& traffic, int i);
void init_flow_heuristic(TrajLNS& lns, std::vector<int>& traffic, FlowHeuristic& ht, int i, int goal_loc);

s_node* get_flow_heuristic(FlowHeuristic& ht, SharedEnvironment* env, std::vector<int>& traffic, std::vector<Int4>& flow, int source);

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);


int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, std::vector<int>& traffic, std::vector<Int4>& flow, int source);




void compute_dist_2_path(std::vector<int>& my_heuristic, SharedEnvironment* env, Traj& path);

void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path, std::vector<int>& traffic);

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, std::vector<int>& traffic, int source);




}
#endif