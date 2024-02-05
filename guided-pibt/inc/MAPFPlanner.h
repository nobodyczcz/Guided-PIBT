#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "Types.h"
#include "TrajLNS.h"

using namespace TrafficMAPF;


class MAPFPlanner
{
public:

    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){
        delete env;
    }

    std::vector<int> decision; 
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> task_change;

    // Start kit dummy implementation

    std::vector<int> traffic; // traffic rules
    bool traffic_control;

    TrajLNS trajLNS;
    // bool traffic_control = false;

    int fg_init_count = 0;




    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    // Start kit dummy implementation
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end,unordered_set<tuple<int,int,int>> reservation, int time);
    std::list<pair<int,int>>single_agent_plan(int start,int start_direct, int end);
    int getManhattanDistance(int loc1, int loc2);
    std::list<pair<int,int>> getNeighbors(int location, int direction);
    bool validateMove(int loc,int loc2);




};
