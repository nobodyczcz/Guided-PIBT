#include <MAPFPlanner.h>
#include <random>
#include "pibt.hpp"
#include "flow.hpp"
#include "heuristics.hpp"

using namespace TrafficMAPF;

struct AstarNode {
    int location;
    int direction;
    int f,g,h;
    AstarNode* parent;
    int t = 0;
    bool closed = false;
    AstarNode(int _location,int _direction, int _g, int _h, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),parent(_parent) {}
    AstarNode(int _location,int _direction, int _g, int _h, int _t, AstarNode* _parent):
        location(_location), direction(_direction),f(_g+_h),g(_g),h(_h),t(_t),parent(_parent) {}
};


struct cmp {
    bool operator()(AstarNode* a, AstarNode* b) {
        if(a->f == b->f) return a->g <= b->g;
        else return a->f > b->f;
    }
};



void MAPFPlanner::initialize(int preprocess_time_limit) {
    assert(env->num_of_agents != 0);
    p.resize(env->num_of_agents);
    decision.resize(env->map.size(), -1);
    prev_states.resize(env->num_of_agents);
    next_states.resize(env->num_of_agents);
    decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
    occupied.resize(env->map.size(),false);
    checked.resize(env->num_of_agents,false);
    ids.resize(env->num_of_agents);
    task_change.resize(env->num_of_agents,false);
    for (int i = 0; i < ids.size();i++){
        ids[i] = i;
    }

    trajLNS = TrajLNS(env);
    trajLNS.init_mem();

    env->init_neighbor();

    std::shuffle(ids.begin(), ids.end(), std::mt19937(std::random_device()()));
    for (int i = 0; i < ids.size();i++){
        p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
    }
    p_copy = p;


    traffic.resize(env->map.size(),-1);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

}


// return next states for all agents
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    cout<<"---timestep,"<< env->curr_timestep<<endl;
    prev_decision.clear();
    prev_decision.resize(env->map.size(), -1);
    occupied.clear();
    occupied.resize(env->map.size(),false);

    int count = 0;
    
    for(int i=0; i<env->num_of_agents; i++)
    {
        if ( (trajLNS.traj_inited < env->num_of_agents && count < RELAX) || (trajLNS.traj_inited >= env->num_of_agents)){
            for(int j=0; j<env->goal_locations[i].size(); j++)
            {
                int goal_loc = env->goal_locations[i][j].first;

                    if (trajLNS.heuristics.at(goal_loc).empty()){
                        init_heuristic(trajLNS.heuristics[goal_loc],env,goal_loc);
                        count++;
                        #ifndef GUIDANCE
                        trajLNS.traj_inited++;
                        #endif

                    }
                    if (OBJECTIVE >= OBJ::SUI_TC){
                        int dist = get_heuristic(trajLNS.heuristics[goal_loc],env,traffic,trajLNS.flow,env->curr_states[i].location);
                        if ( dist > env->max_h)
                            env->max_h = dist;
                    }
            }
        }

        assert(env->goal_locations[i].size()>0);
        task_change[i] =  env->goal_locations[i].front().first != trajLNS.tasks[i];
        trajLNS.tasks[i] = env->goal_locations[i].front().first;

        assert(env->curr_states[i].location >=0);
        prev_states[i] = env->curr_states[i];
        next_states[i] = State();
        prev_decision[env->curr_states[i].location] = i; 
        if (decided[i].loc == -1){
            decided[i].loc = env->curr_states[i].location;
            assert(decided[i].state == DONE::DONE);
        }
        if (prev_states[i].location == decided[i].loc){
            decided[i].state = DONE::DONE;
        }
        if (decided[i].state == DONE::NOT_DONE){
            occupied.at(decided[i].loc) = true;
            occupied.at(prev_states[i].location) = true;
        }

        if(task_change[i])
            p[i] = p_copy[i];
        else
            p[i] = p[i]+1;
        
    }

#ifdef GUIDANCE
    bool init_done = trajLNS.traj_inited == env->num_of_agents;
    TimePoint start_time = std::chrono::steady_clock::now();

    // cout<<"Check task updates"<<endl;
    // #ifndef FLOW_GUIDANCE
        for (int i = 0; i < env->num_of_agents;i++){
            if (task_change[i] && !trajLNS.trajs[i].empty()){

                remove_traj(trajLNS, i);
                update_traj(trajLNS, i, traffic);
            }
        }
    // #endif
    cout << "---t-update," << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_time).count() << endl;


    if (trajLNS.traj_inited < env->num_of_agents){
        // cout << "init traj"<<endl;
#ifdef INIT_PP
        init_traj(trajLNS, traffic, RELAX);
#else
        init_traj_st(trajLNS, traffic);
#endif
        // exit(1);
    }
    cout << "---t-init," << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() << endl;


    std::unordered_set<int> updated;

    #ifdef GUIDANCE_LNS
    if (init_done){
        cout << "---op-flow,"<<trajLNS.op_flow << endl;
        cout << "---vetex-flow,"<<trajLNS.vertex_flow << endl;
        // destory and improve traj
        destory_improve(trajLNS, traffic, updated, GUIDANCE_LNS, time_limit * 0.8);

        // cout << "---updated size:" << updated.size() << endl;
        cout << "---op-flow,"<<trajLNS.op_flow << endl;
        cout << "---vertex-flow,"<<trajLNS.vertex_flow << endl;

        
        #ifndef FLOW_GUIDANCE
            //use dist to path/trajectory
            // cout<<"update dist to path"<<endl;
            for (int i : updated){
                update_dist_2_path(trajLNS, i,traffic);
            }
            
        #endif
    }
    #endif

    cout << "---t-lns," << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() << endl;


    #ifndef FLOW_GUIDANCE
        if (trajLNS.dist2path_inited < env->num_of_agents){
            // cout<<"Init dist to path"<<endl;
            init_dist_table(trajLNS,traffic, RELAX);
        }

        for (int i = 0; i < trajLNS.dist2path_inited;i++){
            if (task_change[i]&& updated.find(i) == updated.end()){
                update_dist_2_path(trajLNS, i,traffic);
            }
        }
        
    #else

        // cout<<"init flow guidance heuristic for all agents."<<endl;
        // if (env->curr_timestep%FLOW_GUIDANCE ==0)
        //     for (int i = 0; i < env->num_of_agents;i++){
        //         if (!trajLNS.trajs[i].empty() && trajLNS.trajs[i].back() == trajLNS.tasks[i])
        //             continue;
        //         remove_traj(trajLNS, i);
        //         update_traj(trajLNS, i, traffic);
        //     }
        // if (env->curr_timestep%FLOW_GUIDANCE ==0)
        //     fg_init_count = 0;
        count = 0;

        for (int i = 0; i < env->num_of_agents; i++){
            if (task_change[i]){
                init_flow_heuristic(trajLNS, traffic,i);
                count++;
            }
        }

        if (fg_init_count >= env->num_of_agents)
            fg_init_count = 0;

        for (int i= fg_init_count; i < env->num_of_agents; i++){
            if (count >= RELAX)
                break;
            if (!task_change[i]){
                init_flow_heuristic(trajLNS, traffic,i);
                count++;
            }
            fg_init_count++;
        }

    #endif

    cout << "---t-done," << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() << endl;

#endif


    std::sort(ids.begin(), ids.end(), [&](int a, int b) {
            return p.at(a) > p.at(b);
        }
    );

    for (int i : ids){
        
        if (decided[i].state == DONE::NOT_DONE){
            continue;
        }
        if (next_states[i].location==-1){
            assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
            causalPIBT(i,-1,prev_states,next_states,
                prev_decision,decision,
                occupied, traffic, trajLNS);
        }
    }
    
    actions.resize(env->num_of_agents);
    for (int id : ids){

        if (next_states.at(id).location!= -1)
            decision.at(next_states.at(id).location) = -1;
        
        assert(
            (next_states.at(id).location >=0 && decided.at(id).state == DONE::DONE)||
            (next_states.at(id).location == -1 && decided.at(id).state == DONE::NOT_DONE)
        );

        if (next_states.at(id).location >=0){
            decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
        }

        

        actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
        checked.at(id) = false;
        #ifndef NDEBUG
            std::cout<<id <<":"<<actions.at(id)<<";"<<std::endl;
        #endif

    }

#ifdef MAPFT
    for (int id=0;id < env->num_of_agents ; id++){
        if (!checked.at(id) && actions.at(id) == Action::FW){
            moveCheck(id,checked,decided,actions,prev_decision);
        }
    }
#endif



    #ifndef NDEBUG
        for (auto d : decision){
            assert(d == -1);
        }
    #endif

    prev_states = next_states;
    return;
}





int MAPFPlanner::getManhattanDistance(int loc1, int loc2) {
    int loc1_x = loc1/env->cols;
    int loc1_y = loc1%env->cols;
    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
}

bool MAPFPlanner::validateMove(int loc, int loc2)
{
    int loc_x = loc/env->cols;
    int loc_y = loc%env->cols;

    if (loc_x >= env->rows || loc_y >= env->cols || env->map[loc] == 1)
        return false;

    int loc2_x = loc2/env->cols;
    int loc2_y = loc2%env->cols;
    if (abs(loc_x-loc2_x) + abs(loc_y-loc2_y) > 1)
        return false;
    return true;

}


list<pair<int,int>> MAPFPlanner::getNeighbors(int location,int direction) {
    list<pair<int,int>> neighbors;
    //forward
    int candidates[4] = { location + 1,location + env->cols, location - 1, location - env->cols};
    int forward = candidates[direction];
    int new_direction = direction;
    if (forward>=0 && forward < env->map.size() && validateMove(forward,location))
        neighbors.emplace_back(make_pair(forward,new_direction));
    //turn left
    new_direction = direction-1;
    if (new_direction == -1)
        new_direction = 3;
    neighbors.emplace_back(make_pair(location,new_direction));
    //turn right
    new_direction = direction+1;
    if (new_direction == 4)
        new_direction = 0;
    neighbors.emplace_back(make_pair(location,new_direction));
    neighbors.emplace_back(make_pair(location,direction)); //wait
    return neighbors;
}
