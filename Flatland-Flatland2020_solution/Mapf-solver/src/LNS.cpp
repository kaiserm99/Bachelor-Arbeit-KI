/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/
#include "LNS.h"


bool LNS::run(float _hard_time_limit, float _soft_time_limit, float success_rate, int max_iterations)
{
    start_time = Time::now();

    hard_time_limit = _hard_time_limit;
    soft_time_limit = min(_soft_time_limit, hard_time_limit);
    if(! skip_pp) {
        if (!getInitialSolution(success_rate,max_iterations)) { // get initial solution
            return false;
        }
    }

    int solution_cost = 0;
    int sum_of_showup_time = 0;
    for (const auto& path : al.paths_all)
    {
        if (path.empty())
        {
            solution_cost += al.constraintTable.length_max;
            initial_makespan = al.constraintTable.length_max;
        }
        else
        {
            solution_cost += (int)path.size() - 1;
            initial_makespan = max((int)path.size() - 1, initial_makespan);
            for (int t  = 0; t < (int)path.size(); t++)
            {
                if (path[t].location >= 0)
                {
                    sum_of_showup_time += t;
                    break;
                }
            }
        }
    }
    runtime = ((fsec)(Time::now() - start_time)).count();
    initial_runtime = runtime;
    initial_sum_of_costs = solution_cost;
    makespan = initial_makespan;

    if (options1.debug)
        cout << "Initial solution cost = " << solution_cost << ", "
         << "travel time = " << solution_cost - sum_of_showup_time << ", "
         << "makespan = " << initial_makespan << ", "
         << "runtime = " << runtime << endl;
    // iteration_stats.emplace_back(al.agents_all.size(), 0, runtime, runtime, makespan, solution_cost,
    //        destroy_strategy, (double)(solution_cost) / max_timestep / al.agents_all.size(), 0, 0, 0);
    if(pp_only || al.getNumOfAllAgents() == 1) {
        if (max_iterations>0 && this->complete!= nullptr && this->complete->load()<0) {
            this->complete->store(solution_cost);
        }
        else if( max_iterations == 0 && this->complete!= nullptr && this->agent_priority_strategy==3){
            this->complete->store(solution_cost);
        }
        else if( max_iterations == 0 && this->complete!= nullptr && this->agent_priority_strategy!=3){
            if (solution_cost < -this->complete->load()) {
                this->complete->store(-solution_cost);
            }
        }
        return true;
    }
    if (destroy_strategy == 3)
    {
        adaptive_destroy = true;
        iterative_destroy = false;
        destroy_heuristics.assign(3, 1);
    }
    else if (destroy_strategy == 4)
    {
        adaptive_destroy = false;
        iterative_destroy = true;
    }
    else // fixed destroy strategy
    {
        adaptive_destroy = false;
        iterative_destroy = false;
    }

    boost::unordered_set<int> tabu_list;
    bool succ;
    auto old_runtime = runtime;
    iterations = 0;
    while (runtime < soft_time_limit && iterations < max_iterations)
    {
        if (this->complete!= nullptr &&
                 ((this->agent_priority_strategy==3 &&  solution_cost > -this->complete->load())
                  ||(this->agent_priority_strategy!=3 && (this->complete->load()>=0 ||solution_cost > -this->complete->load() )))){
            break;
        }

        iterations++;
        runtime =((fsec)(Time::now() - start_time)).count();
        if (al.getNumOfAllAgents() < 2 * max_group_size)
        {
            neighbors.resize(al.getNumOfAllAgents());
            for (int i = 0; i < al.getNumOfAllAgents(); i++)
                neighbors[i] = i;
            sortNeighborsRandomly();
            neighbors.resize(min(max_group_size, al.getNumOfAllAgents()));
            replanByPP();
            runtime = ((fsec)(Time::now() - start_time)).count();
            solution_cost += delta_costs;
            if (options1.debug)
                cout << "Iteration " << iterations << ", "
                     << "group size = " << neighbors.size() << ", "
                     << "solution cost = " << solution_cost << ", "
                     << "remaining time = " << soft_time_limit - runtime << endl;
            //iteration_stats.emplace_back(neighbors.size(), 0, runtime, runtime - old_runtime, makespan, solution_cost,
            //        destroy_strategy, (double)(solution_cost) / max_timestep / al.agents_all.size(), 0, 0, 0);
            old_runtime = runtime;
            continue;
        }
        if (adaptive_destroy)
        {
            double sum = 0;
            for (const auto& h : destroy_heuristics)
                sum += h;
            if (options1.debug)
            {
                cout << "destroy heuristics = ";
                for (const auto& h : destroy_heuristics)
                    cout << h / sum << ",";
            }
            double r = (double) rand() / RAND_MAX;
            if (r * sum < destroy_heuristics[0])
                destroy_strategy = 0;
            else if (r * sum < destroy_heuristics[0] + destroy_heuristics[1])
                destroy_strategy = 1;
            else
                destroy_strategy = 2;
            if (options1.debug)
                cout << "Choose destroy strategy " << destroy_strategy << endl;
        }
        else if (iterative_destroy)
        {
            destroy_strategy = (destroy_strategy + 1) % 3;
        }

        switch (destroy_strategy)
        {
            case 0:
                generateNeighborByRandomWalk(tabu_list);
                break;
            case 1:
                succ = generateNeighborByStart();
                if(!succ) { // no two agents have the same start locations
                    if(this->complete!= nullptr && this->agent_priority_strategy==3){
                        this->complete->store(solution_cost);
                    }
                    else if( this->complete!= nullptr && this->agent_priority_strategy!=3){
                        if (solution_cost < -this->complete->load()) {
                            this->complete->store(-solution_cost);
                        }
                    }
                    return true;
                }
                break;
            case 2:
                //if (rand() % 2)
                succ = generateNeighborByIntersection();
                //else
                // succ = generateNeighborByTemporalIntersection();
                if(!succ) // the selected intersection has fewer than 2 agents
                    continue;
                break;
            default:
                cout << "Wrong neighbor generation strategy" << endl;
                exit(0);
        }

        assert(replan_strategy == 1);
        switch (prirority_ordering_strategy)  // generate priority ordering for prioritized planning
        {
            case 0:
                sortNeighborsRandomly();
                break;
            case 1:
                sortNeighborsByRegrets();
                break;
            default:
                cout << "Wrong prirority ordering strategy" << endl;
                exit(0);
        }

        replanByPP();

        if (adaptive_destroy) // update destroy heuristics
        {
            if (delta_costs < 0)
                destroy_heuristics[destroy_strategy] = reaction_factor * (-delta_costs)
                                                   + (1 - reaction_factor) * destroy_heuristics[destroy_strategy];
            else
                destroy_heuristics[destroy_strategy] = (1 - decay_factor) * destroy_heuristics[destroy_strategy];
        }
        runtime = ((fsec)(Time::now() - start_time)).count();
        solution_cost += delta_costs;
        if (options1.debug)
            cout << "Iteration " << iterations << ", "
             << "group size = " << neighbors.size() << ", "
             << "solution cost = " << solution_cost << ", "
             << "remaining time = " << soft_time_limit - runtime << endl;
        //iteration_stats.emplace_back(neighbors.size(), 0, runtime, runtime - old_runtime, makespan, solution_cost,
        //        destroy_strategy, (double)(solution_cost) / max_timestep / al.agents_all.size(), 0, 0, 0);
        old_runtime = runtime;
        if (replan_strategy == 0 && max_group_size > al.agents_all.size()) {
            if(this->complete!= nullptr && this->agent_priority_strategy==3){
                this->complete->store(solution_cost);
            }
            else if( this->complete!= nullptr && this->agent_priority_strategy!=3){
                if (solution_cost < -this->complete->load()) {
                    this->complete->store(-solution_cost);
                }
            }
            return true; // CBS has replanned paths for all agents. No need for further iterations
        }
    }
    for (const auto& path : al.paths_all)
    {
        if (path.empty())
        {
            sum_of_costs += al.constraintTable.length_max;
            makespan = al.constraintTable.length_max;
        }
        else
        {
            sum_of_costs += (int)path.size() - 1;
            makespan = max(makespan, (int)path.size() - 1);
        }
    }

    cout << "LNS improves the solution to: Sum of costs = " << sum_of_costs << " and makespan = " << makespan <<" runtime "<< runtime<< " priority "<<this->agent_priority_strategy<< endl;
    if(this->complete!= nullptr && this->agent_priority_strategy==3){
        this->complete->store(solution_cost);
    }
    else if( this->complete!= nullptr && this->agent_priority_strategy!=3){
        if (solution_cost < -this->complete->load()) {
            this->complete->store(-solution_cost);
        }
    }

    return true;
}


//bool LNS::replan(float time_limit)
//{
//    start_time = Time::now();
//    max_timestep = al.constraintTable.length_max;
//    al.num_of_agents = 1;
//    al.agents.resize(1);
//
//    map<int, list<int>> agent_groups;  // key = path length, value = list of agents
//    int makespan = 0;
//    for (int i = 0; i < al.getNumOfAllAgents(); i++)
//    {
//        if (al.agents_all[i].status >= 2)
//            continue;
//        if (al.paths_all[i].empty())
//        {
//            makespan = max_timestep * 2;
//            agent_groups[max_timestep * 2].push_back(i);
//        }
//        else
//        {
//            makespan = max(makespan, (int) al.paths_all[i].size() - 1);
//            if (al.agents_all[i].status == 0)
//                agent_groups[al.paths_all[i].size() - 1].push_back(i);
//        }
//    }
//    bool empty_path = false;
//    for (auto it = agent_groups.rbegin(); it != agent_groups.rend() && !empty_path; ++it) // replan paths from the longest to the shortest
//    {
//
//        for (auto i : it->second)
//        {
//            runtime = ((fsec) (Time::now() - start_time)).count();
//            if (runtime >= time_limit)
//                return true;
//
//            auto copy = al.paths_all[i];
//            al.constraintTable.delete_path(i, al.paths_all[i]);
//            runtime = ((fsec) (Time::now() - start_time)).count();
//            al.agents[0] = &al.agents_all[i];
//            SIPP planner(ml,al,f_w,time_limit - runtime,options1);
//            planner.search();
//            assert(planner.path.size() <= copy.size());
//            if (planner.path.empty())
//            {
//                addAgentPath(i, copy);
//                empty_path = true;
//                if (it->first < max_timestep * 2)
//                    break;
//            }
//            else
//            {
//                addAgentPath(i, planner.path);
//                if (copy.size() == planner.path.size()) // fail to decrease the makespan
//                {
//                    return true;
//                }
//            }
//        }
//    }
//    return true;
//}


bool LNS::replan(float time_limit)
{
    start_time = Time::now();
    max_timestep = al.constraintTable.length_max;
    set<int> tabu_list; // record the agents already been replanned
    al.num_of_agents = 1;
    al.agents.resize(1);
    list<list<pair<int, int>>> agent_groups;
    for (const auto& mal_agent : al.new_malfunction_agents)
    {
        // find the intersections in front of the mal_agent
        list<tuple<int, int, int> > future_intersections; // <location, timestep>
        int t = 0;
        if (al.agents_all[mal_agent].status == 0) // the mal agent is still in the station
        {
            for (;t < (int) al.paths_all[mal_agent].size(); t++)
            {
                if (al.paths_all[mal_agent][t].location > 0)
                {
                    future_intersections.emplace_back(al.agents_all[mal_agent].initial_location,
                            t + 1, al.agents_all[mal_agent].malfunction_left + 1); // replan agents at the start location
                    break;
                }
            }

        }
        for (;t < (int) al.paths_all[mal_agent].size(); t++)
        {
            int loc = al.paths_all[mal_agent][t].location;
            if (loc < 0)
                continue;
            if (ml.getDegree(loc) > 2 && (future_intersections.empty() || get<0>(future_intersections.back()) != loc)) {
                future_intersections.emplace_back(loc, t + 1, t + al.agents_all[mal_agent].malfunction_left + 1);
            }
        }
        for (const auto &intersection : future_intersections)
        {
            // get agents that pass through the intersection after the mal agent
            list<pair<int, int>> agents; // <agent_id, timestep>
            al.constraintTable.get_agents(agents, mal_agent, intersection); // TODO: get this information from MCP instead of constraint table
            runtime = ((fsec) (Time::now() - start_time)).count();
            for (const auto &agent : agents) // replan the agents one by one
            {
                if (runtime >= time_limit)
                    return true;
                int i = agent.first;
                int t = agent.second;
                if (tabu_list.count(i) > 0 || // the agent has already been replanned, or
                    (get<1>(intersection) - 1 > 0 && // the agent is following the mal_agent. We do not replan them
                     al.paths_all[i][t - 1].location >= 0 &&
                     al.paths_all[i][t - 1].location == al.paths_all[mal_agent][get<1>(intersection) - 2].location)
                        )
                    continue;
                auto copy = al.paths_all[i];
                al.constraintTable.delete_path(i, al.paths_all[i]);
                runtime = ((fsec) (Time::now() - start_time)).count();
                al.agents[0] = &al.agents_all[i];
                SIPP planner(ml, al, f_w, time_limit - runtime, options1);
                planner.search();
                replan_times++;
                tabu_list.insert(i);
                if (!planner.path.empty()) {
                    addAgentPath(i, planner.path);
                } else if (!copy.empty() &&
                           (al.agents_all[i].status > 0 || copy.back().location == al.agents_all[i].goal_location)) {
                    addAgentPath(i, copy);
                } else {
                    al.paths_all[i].clear();
                }
                runtime = ((fsec) (Time::now() - start_time)).count();
                //if (runtime < time_limit && planner.path.empty())
                //    dead_agent = true;
            }
            agent_groups.push_back(agents);
        }
    }

    // replan the skipped agents
    /*for (const auto &agents : agent_groups)
    {
        for (const auto &agent : agents) // replan the agents one by one
        {
            runtime = ((fsec) (Time::now() - start_time)).count();
            if (runtime >= time_limit)
                return true;
            int i = agent.first;
            int t = agent.second;
            if (tabu_list.count(i) > 0)
                continue;
            auto copy = al.paths_all[i];
            al.constraintTable.delete_path(i, al.paths_all[i]);
            runtime = ((fsec) (Time::now() - start_time)).count();
            al.agents[0] = &al.agents_all[i];
            SIPP planner(ml,al,f_w,time_limit - runtime,options1);
            planner.search();
            replan_times++;
            if (planner.path.empty())
            {
                addAgentPath(i, copy);
            }
            else
            {
                addAgentPath(i, planner.path);
                tabu_list.insert(i);
            }
        }
    }*/

    return true;
}

bool LNS::replan(list<int>& to_be_replanned, float time_limit)
{
    start_time = Time::now();
    max_timestep = al.constraintTable.length_max;
    set<int> tabu_list; // record the agents already been replanned
    al.num_of_agents = 1;
    al.agents.resize(1);
    runtime = ((fsec)(Time::now() - start_time)).count();
    while (!to_be_replanned.empty() && runtime < time_limit)
    {
        int i = to_be_replanned.front();
        to_be_replanned.pop_front();
        if (tabu_list.count(i) > 0)
            continue;
        auto copy = al.paths_all[i];
        al.constraintTable.delete_path(i, al.paths_all[i]);
        runtime = ((fsec) (Time::now() - start_time)).count();
        al.agents[0] = &al.agents_all[i];
        SIPP planner(ml, al, f_w, time_limit - runtime, options1);
        planner.search();
        //cout << ((fsec)(Time::now() - start_time)).count() - runtime << ",";
        runtime = ((fsec) (Time::now() - start_time)).count();
        if (!planner.path.empty()) {
            addAgentPath(i, planner.path);
        }
        else if (runtime > time_limit)
        {
            to_be_replanned.push_back(i);
        }
    }


    return true;
}


bool LNS::getInitialSolution(float success_rate, int max_iterations)
{
    if (options1.debug)
        cout << "Prioritized planning" << endl;
    int screen;
    screen = options1.debug;
    if (options1.debug)
        cout << "Sort the agents" << endl;
    neighbors.resize(al.agents_all.size());
    for (int i = 0; i < (int)al.agents_all.size(); i++)
        neighbors[i] = i;
    sortNeighborsByStrategy();

    al.num_of_agents = 1;
    al.agents.resize(1);
    int remaining_agents = (int)neighbors.size();
    int dead_agents = 0;
    int sum_of_costs = al.sum_of_distances;
    int makespan = 0;
    runtime = ((fsec)(Time::now() - start_time)).count();
    for (auto agent : neighbors)
    {
        if ( ( this->complete!= nullptr &&
                    ((this->agent_priority_strategy==3 &&  sum_of_costs > -this->complete->load())
                    ||(this->agent_priority_strategy!=3 && (this->complete->load()>=0 ||sum_of_costs > -this->complete->load() ))))
            || runtime >= hard_time_limit
            || al.getNumOfAllAgents() - remaining_agents >= success_rate * al.getNumOfAllAgents())
        {
            cout << "Find a solution for " << al.getNumOfAllAgents() - remaining_agents - dead_agents << " agents" <<
                    " with " << dead_agents << " agents dead and " << remaining_agents << " agents unplanned" << "agent priority "<< agent_priority_strategy<< endl;
            cout << "Sum of costs = " << sum_of_costs <<  " and makespan = " << makespan << endl;
            if(this->complete!= nullptr && this->agent_priority_strategy==3 ){
                this->complete->store(sum_of_costs);
            }
            //if agent_priority_strategy != 3 and stop here, means it's solution is worse no signal needed

            return false;
        }
        al.agents[0] = &al.agents_all[agent];
        if (options1.debug)
            cout << "Remaining agents = " << remaining_agents <<
             ", remaining time = " << hard_time_limit - runtime << " seconds. " << endl
                    << "Agent " << al.agents[0]->agent_id << endl;
        //runtime = ((fsec)(Time::now() - start_time)).count();
        SIPP planner(ml, al, f_w, hard_time_limit - runtime, options1);
        planner.search();
        //cout << ((fsec)(Time::now() - start_time)).count() - runtime << ",";
        updateCBSResults(planner);
        addAgentPath(agent, planner.path);
        runtime = ((fsec)(Time::now() - start_time)).count();
        if (!planner.path.empty()) // TODO: can be deleted in the submission verison
        {
            sum_of_costs += (int) planner.path.size() - 1 - al.agents_all[agent].distance_to_goal;
            makespan = max((int) planner.path.size() - 1, makespan);
            remaining_agents--;
        }
        else if (runtime < hard_time_limit)
        {
            dead_agents++;
            sum_of_costs += al.constraintTable.length_max - al.agents_all[agent].distance_to_goal;
            makespan = al.constraintTable.length_max;
            remaining_agents--;
        }
    }

    cout << endl << "Find a solution for " << al.getNumOfAllAgents() - remaining_agents - dead_agents << " agents" <<
         " with " << dead_agents << " agents dead and " << remaining_agents << " agents unplanned" <<  "agent priority "<< agent_priority_strategy<<endl;
    cout << "Sum of costs = " << sum_of_costs <<  " and makespan = " << makespan << "runtime ="<<runtime<< endl;
    return true;
}

bool LNS::generateNeighborByStart()
{
    if (start_locations.empty())
    {
        for (int i = 0; i < (int)al.agents_all.size(); i++)
        {
            auto start = al.agents_all[i].initial_location;
            start_locations[start].push_back(i);
        }
        auto it = start_locations.begin();
        while(it != start_locations.end()) // delete start locations that have only one agent
        {
            if (it->second.size() == 1)
                it = start_locations.erase(it);
            else
                ++it;
        }
    }
    if (start_locations.empty())
        return false;
    auto step = rand() % start_locations.size();
    auto it = start_locations.begin();
    advance(it, step);
    neighbors.assign(it->second.begin(), it->second.end());
    if (neighbors.size() > max_group_size ||
        (replan_strategy == 0 && neighbors.size() > max_group_size)) // resize the group for CBS
    {
        sortNeighborsRandomly();
        neighbors.resize(max_group_size);
    }
    if (options1.debug)
        cout << "Generate " << neighbors.size() << " neighbors by start location " << it->first << endl;
    return true;
}

bool LNS::generateNeighborByTemporalIntersection()
{
    if (intersections.empty())
    {
        for (int i = 0; i < ml.map_size(); i++)
        {
            if (ml.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    int location = intersections[rand() % intersections.size()];
    al.constraintTable.get_agents(neighbors_set, max_group_size, location);
    if (neighbors_set.size() <= 1)
        return false;
    neighbors.assign(neighbors_set.begin(), neighbors_set.end());
    if (options1.debug)
        cout << "Generate " << neighbors.size() << " neighbors by intersection " << location << endl;
    return true;
}

bool LNS::generateNeighborByIntersection()
{
    if (intersections.empty())
    {
        for (int i = 0; i < ml.map_size(); i++)
        {
            if (ml.getDegree(i) > 2)
                intersections.push_back(i);
        }
    }

    set<int> neighbors_set;
    int location = intersections[rand() % intersections.size()];
    al.constraintTable.get_agents(neighbors_set, location);
    if (neighbors_set.size() <= 1)
        return false;
    neighbors.assign(neighbors_set.begin(), neighbors_set.end());
    if (neighbors.size() > max_group_size || (replan_strategy == 0 && neighbors.size() > max_group_size)) // resize the group for CBS
    {
        sortNeighborsRandomly();
        neighbors.resize(max_group_size);
    }
    if (options1.debug)
        cout << "Generate " << neighbors.size() << " neighbors by intersection " << location << endl;
    return true;
}

void LNS::generateNeighborByRandomWalk(boost::unordered_set<int>& tabu_list)
{
    if (max_group_size >= (int)al.paths_all.size())
    {
        neighbors.resize(al.paths_all.size());
        for (int i = 0; i < (int)al.paths_all.size(); i++)
            neighbors[i] = i;
        return;
    }

    // find the agent with max regret
    int a = -1;
    for (int i = 0; i < al.paths_all.size(); i++)
    {
        if (tabu_list.find(i) != tabu_list.end())
            continue;
        if (a < 0 || compareByRegrets(i, a))
        {
            a = i;
        }
    }
    if (tabu_list.size() > al.paths_all.size() / 2)
        tabu_list.clear();
    else
        tabu_list.insert(a);

    set<int> neighbors_set;
    neighbors_set.insert(a);
    int T = al.paths_all[a].size();
    int count = 0;
    while (neighbors_set.size() < max_group_size && count < 10 && T > 0)
    {
        int t = rand() % T;
        randomWalk(a, al.paths_all[a][t], t, neighbors_set, max_group_size, (int) al.paths_all[a].size() - 1);
        T = t;
        count++;
    }
    while (neighbors_set.size() < max_group_size)
    {
        int new_agent = rand() % al.paths_all.size();
        neighbors_set.insert(new_agent);
    }

    neighbors.assign(neighbors_set.begin(), neighbors_set.end());
    if (options1.debug)
        cout << "Generate " << neighbors.size() << " neighbors by random walks of agent " << a
        << "(" << al.agents_all[a].distance_to_goal << "->" << al.paths_all[a].size() - 1 << ")" << endl;
}


void LNS::replanByPP()
{
    updateNeighborPaths();
    deleteNeighborPaths();
    al.num_of_agents = 1;
    al.agents.resize(1);
    list<Path> new_paths;
    int sum_of_costs = 0;
    int sum_of_showup_time = 0;
    int makespan = 0;
    for (const auto& agent : neighbors)
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
        if (runtime >= soft_time_limit)
        { // change back to the original paths
            auto path = neighbor_paths.begin();
            for (const auto& agent : neighbors)
            {
                al.paths_all[agent] = *path;
                ++path;
            }
            return;
        }
        al.agents[0] = &al.agents_all[agent];
//        MultiMapICBSSearch<FlatlandLoader> icbs(&ml, &al, f_w, c, 0, options1.debug? 3 : 0, options1);
//        icbs.runICBSSearch();
//        updateCBSResults(icbs);
//        addAgentPath(agent, *icbs.paths[0]);
        SIPP planner(ml,al,f_w,0,options1);
        planner.search();
        updateCBSResults(planner);
        addAgentPath(agent, planner.path);
        assert(planner.path.back().location == al.paths_all[agent].back().location);

        if (planner.path.empty())
        {
            sum_of_costs += max_timestep;
            makespan = max_timestep;
        }
        else
        {
            sum_of_costs += (int)planner.path.size() - 1;
            makespan = max(makespan, (int)planner.path.size() - 1);
            for (int t  = 0; t < (int)planner.path.size(); t++)
            {
                if (planner.path.at(t).location >= 0)
                {
                    sum_of_showup_time += t;
                    break;
                }
            }
        }
    }
    if (sum_of_costs < neighbor_sum_of_costs ||
        (sum_of_costs == neighbor_sum_of_costs && sum_of_showup_time > neighbor_sum_of_showup_time) ||
        (sum_of_costs == neighbor_sum_of_costs && sum_of_showup_time == neighbor_sum_of_showup_time && makespan < neighbor_makespan))
    {
        delta_costs = sum_of_costs - neighbor_sum_of_costs;
    }
    else
    { // change back to the original paths
        deleteNeighborPaths();
        auto path = neighbor_paths.begin();
        for (auto agent : neighbors)
        {
            addAgentPath(agent, *path);
            ++path;
        }
        delta_costs = 0;
    }
}


void LNS::updateNeighborPaths()
{
    if (options1.debug)
        cout << "Agents ids: ";
    neighbor_sum_of_costs = 0;
    neighbor_sum_of_showup_time = 0;
    neighbor_makespan = 0;
    neighbor_paths.clear();
    for (auto i : neighbors)
    {
        if (options1.debug)
            cout << i << ",";
        neighbor_paths.emplace_back(al.paths_all[i]);
        if (al.paths_all[i].empty())
        {
            neighbor_sum_of_costs += max_timestep;
            neighbor_makespan = max_timestep;
        }
        else
        {
            neighbor_sum_of_costs += (int)al.paths_all[i].size() - 1;
            neighbor_makespan = max(neighbor_makespan, (int)al.paths_all[i].size() - 1);
            for (int t  = 0; t < (int)al.paths_all[i].size(); t++)
            {
                if (al.paths_all[i][t].location >= 0)
                {
                    neighbor_sum_of_showup_time += t;
                    break;
                }
            }
        }
    }
    if (options1.debug)
        cout << endl;
}

void LNS::updateNeighborPathsCosts()
{
    if (options1.debug)
        cout << "Agents ids: ";
    neighbor_sum_of_costs = 0;
    neighbor_makespan = 0;
    neighbor_paths.clear();
    for (auto i : neighbors)
    {
        if (options1.debug)
            cout << i << ",";
        neighbor_sum_of_costs += (int)al.paths_all[i].size() - 1;
        neighbor_makespan = max(neighbor_makespan, (int)al.paths_all[i].size() - 1);
    }
    if (options1.debug)
        cout << endl;
}

void LNS::addAgentPath(int agent, const Path& path)
{
    assert(agent == al.agents_all[agent].agent_id);
    if(!al.constraintTable.insert_path(agent, path))
        exit(10);
    al.paths_all[agent] = path;
}

void LNS::deleteNeighborPaths()
{
    for (auto i : neighbors)
    {
        assert(i == al.agents_all[i].agent_id);
        al.constraintTable.delete_path(i, al.paths_all[i]);
    }
}

void LNS::sortNeighborsRandomly()
{
    std::random_shuffle(neighbors.begin(), neighbors.end());
    if (options1.debug) {
        for (auto agent : neighbors) {
            cout << agent << "(" << al.agents_all[agent].distance_to_goal << "->" << al.paths_all[agent].size() - 1
                 << "), ";
        }
        cout << endl;
    }
}

void LNS::sortNeighborsByRegrets()
{
    quickSort(neighbors, 0, neighbors.size() - 1, true);
    if (options1.debug) {
        for (auto agent : neighbors) {
            cout << agent << "(" << al.agents_all[agent].distance_to_goal << "->" << al.paths_all[agent].size() - 1
                 << "), ";
        }
        cout << endl;
    }
}

void LNS::sortNeighborsByStrategy()
{
    if (agent_priority_strategy == 5)
    {
        // decide the agent priority for agents at the same start location
        start_locations.clear(); // map the agents to their start locations
        for (auto i : neighbors)
            start_locations[al.agents_all[i].initial_location].push_back(i);
        for (auto& agents : start_locations)
        {
            vector<int> agents_vec(agents.second.begin(), agents.second.end());
            quickSort(agents_vec, 0, agents_vec.size() - 1, false);
            for (int i = 0; i < (int)agents.second.size(); i++)
            {
                al.agents_all[agents_vec[i]].priority = i;
            }
        }
    }
    else if (agent_priority_strategy == 6)
    {
        // decide the agent priority for agents at the same start location
        start_locations.clear(); // map the agents to their start locations
        for (auto i : neighbors)
            start_locations[al.agents_all[i].initial_location].push_back(i);
        int idx = 0;
        for (auto& agents : start_locations)
        {
            for (auto& i : agents.second)
            {
                al.agents_all[i].priority = idx;
            }
            idx++;
        }
    }
    // sort the agents
    if (agent_priority_strategy != 0)
        quickSort(neighbors, 0, (int)neighbors.size() - 1, false);
}


void LNS::quickSort(vector<int>& agent_order, int low, int high, bool regret)
{
    if (low >= high)
        return;
    int pivot = agent_order[high];    // pivot
    int i = low;  // Index of smaller element
    for (int j = low; j <= high - 1; j++)
    {
        // If current element is smaller than or equal to pivot
        if ((regret && compareByRegrets(agent_order[j], pivot)) ||
            al.compareAgent(al.agents_all[agent_order[j]], al.agents_all[pivot], agent_priority_strategy))
        {
            std::swap(agent_order[i], agent_order[j]);
            i++;    // increment index of smaller element
        }
    }
    std::swap(agent_order[i], agent_order[high]);

    quickSort(agent_order, low, i - 1, regret);  // Before i
    quickSort(agent_order, i + 1, high, regret); // After i
}

void LNS::randomWalk(int agent_id, const PathEntry& start, int start_timestep,
        set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    // a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
    int speed = al.agents_all[agent_id].speed;
    const auto& heuristics = *al.agents_all[agent_id].heuristics;
    int loc = start.location;
    int heading = start.heading;
    auto position_fraction = start.position_fraction;
    auto exit_heading = start.exit_heading;
    int exit_loc = start.exit_loc;
    int h_val;
    if (loc < 0)
    {
        int initial_location = al.agents_all[agent_id].initial_location;
        h_val = heuristics[initial_location].get_hval(heading) / speed + 1;
    }
    else
    {
        h_val = heuristics[loc].get_hval(heading) / speed;
        if (exit_loc >= 0 && speed < 1)
        {
            int h1 = heuristics[loc].get_hval(heading);
            int h2 = heuristics[exit_loc].get_hval(exit_heading);
            h_val = h1 / speed - (h2 - h1)*position_fraction;
        }
    }


    for (int t = start_timestep; t < upperbound; t++)
    {
        list<Transition> transitions;
        if(loc == -1){
            Transition move;
            move.location = loc;
            move.heading = heading;
            move.position_fraction = position_fraction;
            move.exit_loc = exit_loc;
            move.exit_heading = exit_heading;
            transitions.push_back(move);

            Transition move2;
            move2.location = al.agents_all[agent_id].initial_location;
            move2.heading = heading;
            move2.position_fraction = position_fraction;
            move2.exit_loc = exit_loc;
            move2.exit_heading = exit_heading;
            transitions.push_back(move2);
        }
        else if (position_fraction + speed >= 0.97)
        {
            if (position_fraction == 0)
            {
                ml.get_transitions(transitions, loc, heading, false);
                assert(!transitions.empty());
            }
            else {
                Transition move;
                move.location = exit_loc;
                move.heading = exit_heading;
                move.position_fraction = 0;
                transitions.push_back(move);
            }
        }
        else if (position_fraction == 0)
        {
            ml.get_exits(transitions, loc, heading, speed, false);
            assert(!transitions.empty());
        }
        else { //<0.97 and po_frac not 0
            Transition move2;
            move2.location = loc;
            move2.heading = heading;
            move2.position_fraction = position_fraction + al.agents[agent_id]->speed;
            move2.exit_loc = exit_loc;
            move2.exit_heading = exit_heading;
            transitions.push_back(move2);
        }

        while(!transitions.empty())
        {
            int step = rand() % transitions.size();
            auto it = transitions.begin();
            advance(it, step);
            int next_h_val;
            if (it->location == -1)
                next_h_val = h_val;
            else if (exit_loc >= 0 && speed < 1)
            {
                int h1 = heuristics[it->location].get_hval(it->heading);
                int h2 = heuristics[it->exit_loc].get_hval(it->exit_heading);
                next_h_val = h1 / speed - (h2 - h1) * (it->position_fraction / speed);
            }
            else
                next_h_val = heuristics[it->location].get_hval(it->heading) / speed;

            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                loc = it->location;
                heading = it->heading;
                position_fraction = it->position_fraction;
                exit_heading = it->exit_heading;
                exit_loc = it->exit_loc;
                h_val = next_h_val;
                assert(agent_id == al.agents_all[agent_id].agent_id);
                al.constraintTable.get_conflicting_agents(agent_id, conflicting_agents, loc, t + 1);
                break;
            }
            transitions.erase(it);
        }
        if (transitions.empty() || conflicting_agents.size() >= neighbor_size || h_val == 0)
            break;
    }
}