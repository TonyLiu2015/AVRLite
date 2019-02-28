// AgentPlus.cpp : Defines the entry point for the console application.
/* Copyright (C) 2015 Xuesong Zhou - All Rights Reserved*/

//add your names here

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>
#include <string>
#include <time.h>
#include "AgentPlus.h"
#include "CSVParser.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// The one and only application object
CWinApp theApp;
using namespace std;

FILE* g_pFileDebugLog = NULL;
FILE* g_pFileGAMSinputSetLog = NULL;
FILE* g_pFileGAMSinputParameterLog = NULL;

FILE* g_pFileGAMSinputSetCsv = NULL;
FILE* g_pFileGAMSinputParaVehPathCostCsv = NULL;
FILE* g_pFileGAMSinputParaVehPathSelectionCsv = NULL;
FILE* g_pFileGAMSinputParaIncidVehPathPaxCsv = NULL;
FILE* g_pFileGAMSinputParaIncidVehPathArcCsv = NULL;
FILE* g_pFileGAMSinputParaArcCapCsv = NULL;

FILE* g_pFile_OutputAgentLog = NULL;
FILE* g_pFile_OutputNodeLog = NULL;
FILE* g_pFile_OutputLinkLog = NULL;

FILE* g_pFileOutputLog = NULL;
//FILE* g_pFileAgentPathLog = NULL;

FILE* g_pFile_Output_paxprofitLog = NULL;
FILE* g_pFile_PathLog = NULL;

FILE* g_pFile_Vehicle_Path_PassengerLog = NULL;
FILE* g_pFile_OutputAgentPathArcLog = NULL;

std::map<int, int> g_internal_node_no_map;
std::map<int, int> g_external_node_id_map;
std::map<int, int> g_internal_agent_no_map;
std::map<int, int> g_internal_vehicle_no_map;

std::map<int, int> g_external_passenger_id_map;
std::map<int, int> g_external_vehicle_id_map;
std::map<int, int> g_external_agent_id_to_vehicle_id_map;

int g_number_of_threads = 4;
// passenger info in Node as pickup and dropoff node are all passengers 
int g_node_passenger_id[_MAX_NUMBER_OF_NODES];
int g_node_type[_MAX_NUMBER_OF_NODES];  //key nodeid; 1: pick up, 2: drop off
int g_node_depot_type[_MAX_NUMBER_OF_NODES];  // 10 depot
int served_passenger_id[_MAX_NUMBER_OF_NODES];// served passenger

int g_node_timestamp[_MAX_NUMBER_OF_NODES];  //key nodeid;, values type 1: pick up:ready time, 2: order time
float g_node_baseprofit[_MAX_NUMBER_OF_NODES];  // type 2; = 0
//the number of outbound nodes. key nodeid;, values type 1: pick up:ready time, 2: order time
int g_outbound_node_size[_MAX_NUMBER_OF_NODES];
//std::map<int, int> pax_id_to_seq_no_map;

float g_passenger_base_profit[_MAX_NUMBER_OF_PASSENGERS] = { -7 };
float local_vehicle_passenger_additional_profit[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };

float g_passenger_order_time[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
//used in profit(LR multipliers) update: summation of variable y
int g_passenger_number_of_visits[_MAX_NUMBER_OF_PASSENGERS] = { 0 };
int g_passenger_vehicle_visit_flag[_MAX_NUMBER_OF_PASSENGERS][_MAX_NUMBER_OF_VEHICLES] = { 0 };
//whether vehicle v can take passenger p; 1 ok 0 no; in prohibit links
//prohibt_visit and passernger can only be carried once control this one
int g_vehicle_passenger_visit_allowed_flag[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_PASSENGERS] = { 0 };

int g_max_vehicle_capacity = 1;
int g_number_of_passengers = 0;

int g_outbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_outbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_activity_node_flag[_MAX_NUMBER_OF_NODES] = { 0 };
int g_activity_node_ending_time[_MAX_NUMBER_OF_NODES] = { 99999 };
int g_activity_node_starting_time[_MAX_NUMBER_OF_NODES] = { 99999 };

int g_inbound_node_size[_MAX_NUMBER_OF_NODES];
int g_inbound_node_id[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];
int g_inbound_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_OUTBOUND_NODES];

int g_link_cap[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];// time-dependent link capacity
int g_link_time_dependent_travel_time[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS];

float g_link_time_dependent_margianl_cost[_MAX_NUMBER_OF_LINKS][_MAX_NUMBER_OF_TIME_INTERVALS] = {0.0};
float g_passenger_marginal_cost[_MAX_NUMBER_OF_PASSENGERS] = { 0.0 };
float g_vehicle_marginal_cost[_MAX_NUMBER_OF_VEHICLES] = { 0.0 };

int g_link_free_flow_travel_time[_MAX_NUMBER_OF_LINKS];
float g_link_free_flow_travel_time_float_value[_MAX_NUMBER_OF_LINKS];

float g_link_link_length[_MAX_NUMBER_OF_LINKS];
int g_link_number_of_lanes[_MAX_NUMBER_OF_LINKS];
int g_link_mode_code[_MAX_NUMBER_OF_LINKS];
float g_link_capacity_per_time_interval[_MAX_NUMBER_OF_LINKS];
float g_link_jam_density[_MAX_NUMBER_OF_LINKS];
int g_link_service_code[_MAX_NUMBER_OF_LINKS] = { 0 };

float g_link_speed[_MAX_NUMBER_OF_LINKS];
int g_link_from_node_number[_MAX_NUMBER_OF_LINKS];//internal from node id
int g_link_to_node_number[_MAX_NUMBER_OF_LINKS];// internal to node id
int g_link_no[_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_NODES];

float g_VOIVTT_per_hour[_MAX_NUMBER_OF_VEHICLES];
float g_VOWT_per_hour[_MAX_NUMBER_OF_VEHICLES];

int g_vehicle_path_number_of_nodes[_MAX_NUMBER_OF_VEHICLES] = { 0 };

int g_vehicle_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
int g_vehicle_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_depot_origin_node[_MAX_NUMBER_OF_VEHICLES];  // for vehcile routings
int g_vehicle_depot_destination_node[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_depot_tree_id[_MAX_NUMBER_OF_VEHICLES];  // vehicle id

int g_vehicle_departure_time_beginning[_MAX_NUMBER_OF_VEHICLES] = { 0 };
int g_vehicle_departure_time_ending[_MAX_NUMBER_OF_VEHICLES];
int g_vehicle_arrival_time_beginning[_MAX_NUMBER_OF_VEHICLES] = { 120 };
int g_vehicle_arrival_time_ending[_MAX_NUMBER_OF_VEHICLES];

int g_passenger_origin_node[_MAX_NUMBER_OF_PASSENGERS];  // traveling passengers
int g_passenger_destination_node[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_dummy_destination_node[_MAX_NUMBER_OF_PASSENGERS] = { -1 };

int g_passenger_departure_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_departure_time_ending[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_arrival_time_beginning[_MAX_NUMBER_OF_PASSENGERS];
int g_passenger_arrival_time_ending[_MAX_NUMBER_OF_PASSENGERS];

int g_vehicle_capacity[_MAX_NUMBER_OF_VEHICLES] = { 1 };
float g_vehicle_path_cost[_MAX_NUMBER_OF_VEHICLES] = { 1.0 };

int g_number_of_links = 0;
int g_number_of_nodes = 0;
int g_number_of_depots = 0;

int g_number_of_physical_nodes = 0;
int g_number_of_time_intervals = 10;
int g_number_of_k_paths_iterations = 10;

int g_number_of_vehicles = 0;

float g_best_upper_bound = 99999;
float g_best_lower_bound = -99999;

int g_number_of_LR_iterations = 1;
double g_minimum_subgradient_step_size = 0.01;

int g_shortest_path_debugging_flag = 1;
//float g_waiting_time_ratio = 0.005;
//float g_dummy_vehicle_cost_per_hour = 100;

//float g_travel_time_budget = 100;
//float g_idle_vehicle_benefit = -10;

CTime g_SolutionStartTime;

int g_add_new_node_origin(int passenger_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_number = g_number_of_nodes + 1;
	g_outbound_node_size[new_node_number] = 0;
	g_inbound_node_size[new_node_number] = 0;
	g_node_passenger_id[new_node_number] = passenger_id;
	g_activity_node_flag[new_node_number] = 1;
	g_activity_node_starting_time[new_node_number] = beginning_time;
	g_activity_node_ending_time[new_node_number] = end_time;
	g_node_type[new_node_number] = 1;

	g_number_of_nodes++;
	if (g_number_of_nodes > _MAX_NUMBER_OF_NODES - 1)
	{
		cout << "g_number_of_nodes = " << g_number_of_nodes << " is out of range of _MAX_NUMBER_OF_NODES " << _MAX_NUMBER_OF_NODES << endl;
		g_ProgramStop();
	}

	return g_number_of_nodes;
}

int g_add_new_node(int vehicle_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_number = g_number_of_nodes + 1;
	g_outbound_node_size[new_node_number] = 0;
	g_inbound_node_size[new_node_number] = 0;
	g_node_passenger_id[new_node_number] = 0;
	g_activity_node_flag[new_node_number] = 1;
	g_activity_node_starting_time[new_node_number] = beginning_time;
	g_activity_node_ending_time[new_node_number] = end_time;
	g_node_type[new_node_number] = 3;

	g_number_of_nodes++;
	if (g_number_of_nodes > _MAX_NUMBER_OF_NODES - 1)
	{
		cout << "g_number_of_nodes = " << g_number_of_nodes << " is out of range of _MAX_NUMBER_OF_NODES " << _MAX_NUMBER_OF_NODES << endl;
		g_ProgramStop();
	}
	return g_number_of_nodes;
}

int g_add_new_node_destination(int passenger_id, int beginning_time = -1, int end_time = -1)
{
	int new_node_number = g_number_of_nodes + 1;
	g_outbound_node_size[new_node_number] = 0;
	g_inbound_node_size[new_node_number] = 0;
	g_node_passenger_id[new_node_number] = passenger_id;
	g_activity_node_flag[new_node_number] = 1;
	g_activity_node_starting_time[new_node_number] = beginning_time;
	g_activity_node_ending_time[new_node_number] = end_time;
	g_node_type[new_node_number] = 2;

	g_number_of_nodes++;
	if (g_number_of_nodes > _MAX_NUMBER_OF_NODES - 1)
	{
		cout << "g_number_of_nodes = " << g_number_of_nodes << " is out of range of _MAX_NUMBER_OF_NODES " << _MAX_NUMBER_OF_NODES << endl;
		g_ProgramStop();
	}
	return g_number_of_nodes;
}

int g_add_new_link(int from_node_id, int to_node_id, int passenger_id = 0, int travel_time = 1, double link_length = 1, int number_of_lanes = 1, int mode_code = 0,
	int capacity_per_time_interval = 99999, double speed = 60)
{
	int new_link_id = g_number_of_links;
	g_outbound_node_id[from_node_id][g_outbound_node_size[from_node_id]] = to_node_id;
	g_outbound_link_no[from_node_id][g_outbound_node_size[from_node_id]] = new_link_id;

	g_outbound_node_size[from_node_id]++;

	g_inbound_node_id[to_node_id][g_inbound_node_size[to_node_id]] = from_node_id;
	g_inbound_link_no[to_node_id][g_inbound_node_size[to_node_id]] = new_link_id;
	g_inbound_node_size[to_node_id]++;

	g_link_from_node_number[new_link_id] = from_node_id;
	g_link_to_node_number[new_link_id] = to_node_id;
	g_link_no[from_node_id][to_node_id] = new_link_id;

	g_link_free_flow_travel_time[new_link_id] = max(1, travel_time);

	g_link_link_length[g_number_of_links] = link_length;
	g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
	g_link_mode_code[g_number_of_links] = mode_code;
	g_link_capacity_per_time_interval[g_number_of_links] = capacity_per_time_interval;
	g_link_speed[g_number_of_links] = speed;
	g_link_service_code[g_number_of_links] = passenger_id;

	g_number_of_links++;

	if (g_number_of_links > _MAX_NUMBER_OF_LINKS - 1)
	{
		cout << "g_number_of_links = " << g_number_of_links << " is out of range of _MAX_NUMBER_OF_LINKS " << _MAX_NUMBER_OF_LINKS << endl;
		g_ProgramStop();

	}
	return g_number_of_links;
}

vector<int> ParseLineToIntegers(string line)
{
	vector<int> SeperatedIntegers;
	string subStr;
	istringstream ss(line);

	char Delimiter = ';';

	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedIntegers.push_back(integer);
	}
	return SeperatedIntegers;
}


void g_ProgramStop()
{
	cout << "Agent+ Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
}

//class for vehicle scheduling states
class CVSState
{
public:
	int current_node_id;  // space dimension
						  //passengerID, nodeType 
	std::map<int, int> passenger_service_state;
	std::map<int, int> passenger_service_time;
	std::map<int, int> passenger_service_begin_time;

	std::map<int, int> passenger_carrying_state;
	//visit nodes 
	std::vector<int> m_visit_node_sequence;  // store nodes f
	std::vector<int> m_visit_link_sequence;  // store nodes f
	std::vector<int> m_visit_time_sequence;  // store passing nodes times
	std::vector<string> m_visit_state_sequence;  // store passing nodes states
	std::vector<int> m_visit_passenger_sequence;  // store passing nodes states

	int m_vehicle_capacity;
	float LabelCost;  // with LR price
	float PrimalLabelCost;  // without LR price

	int m_final_arrival_time;   // for ending states


	CVSState()
	{
		m_final_arrival_time = 0;
		LabelCost = _MAX_LABEL_COST;
		m_vehicle_capacity = 0;
	}

	void Copy(CVSState* pSource)
	{
		current_node_id = pSource->current_node_id;
		passenger_service_state.clear();
		passenger_service_state = pSource->passenger_service_state;

		passenger_service_time.clear();
		passenger_service_time = pSource->passenger_service_time;

		passenger_service_begin_time.clear();
		passenger_service_begin_time = pSource->passenger_service_begin_time;

		passenger_carrying_state.clear();
		passenger_carrying_state = pSource->passenger_carrying_state;

		m_visit_node_sequence.clear();
		m_visit_node_sequence = pSource->m_visit_node_sequence;

		m_visit_link_sequence.clear();
		m_visit_link_sequence = pSource->m_visit_link_sequence;

		m_visit_time_sequence.clear();
		m_visit_time_sequence = pSource->m_visit_time_sequence;

		m_visit_state_sequence.clear();
		m_visit_state_sequence = pSource->m_visit_state_sequence;

		m_vehicle_capacity = pSource->m_vehicle_capacity;
		LabelCost = pSource->LabelCost;
	}
	int GetPassengerServiceState(int passenger_id)
	{
		if (passenger_service_state.find(passenger_id) != passenger_service_state.end())
			return passenger_service_state[passenger_id];  // 1 or 2
		else
			return 0;
	}

	void StartCarryingService(int passenger_id, int service_time)
	{
		passenger_carrying_state[passenger_id] = 1;
		m_vehicle_capacity += 1;

		passenger_service_begin_time[passenger_id] = service_time;
	}

	void CompleteCarryingService(int passenger_id, int service_time)
	{
		map<int, int>::iterator iter = passenger_carrying_state.find(passenger_id);
		if (iter != passenger_carrying_state.end())
		{
			passenger_carrying_state.erase(iter);
			//m_vehicle_capacity -= 1;
			m_vehicle_capacity -= 0;
		}
		passenger_service_time[passenger_id] = service_time;
	}

	//Start or Complete service
	void MarkCarryingService(int passenger_id, int node_type, int ServiceTime)
	{
		if (node_type == 1)
			StartCarryingService(passenger_id, ServiceTime);
		if (node_type == 2)
			CompleteCarryingService(passenger_id, ServiceTime);
	}

	bool IsAllServiceComplete()
	{
		if (passenger_carrying_state.size() == 0)
			return true;
		else
			return false;
	}

	//state actually node in SP. label cost
	void CalculateLabelCost(int vehicle_id)
	{
		LabelCost = 0- g_vehicle_marginal_cost[vehicle_id];
		PrimalLabelCost = 0;
		int i = 0;
		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			if (it->second == 2)  // complete
			{
				i++;
				int passenger_id = it->first;
				LabelCost -= g_passenger_base_profit[passenger_id]- g_passenger_marginal_cost[passenger_id];//LBd
				PrimalLabelCost -= g_passenger_base_profit[passenger_id] - g_passenger_marginal_cost[passenger_id];
				 // the waiting cost of passenger is 0.3
				// Tony: I comment the waiting cost of passengers
				/*LabelCost += 0.3*max(0, (passenger_service_begin_time[passenger_id] - g_passenger_order_time[passenger_id]));
				PrimalLabelCost += 0.3*max(0, (passenger_service_begin_time[passenger_id] - g_passenger_order_time[passenger_id]));*/
			}
		}

		// total travel time
		LabelCost += m_visit_time_sequence[m_visit_time_sequence.size() - 1] - g_vehicle_departure_time_beginning[1];
		PrimalLabelCost += m_visit_time_sequence[m_visit_time_sequence.size() - 1] - g_vehicle_departure_time_beginning[1];

		// the difference between waiting cost and transportation cost

		//Tony: I comment the waiting cost of vehicles
		for (int it = 1; it < m_visit_node_sequence.size(); it++)
		{
			if (m_visit_node_sequence[it - 1] == m_visit_node_sequence[it]) //waiting arc
			{
				LabelCost -= (1 - 0.5)*(m_visit_time_sequence[it] - m_visit_time_sequence[it - 1]); //cost of waiting arc is 0.5
				PrimalLabelCost -= (1 - 0.5)*(m_visit_time_sequence[it] - m_visit_time_sequence[it - 1]);
			}
		}
	}

	//class CVSState  record pax and vehicle's completed service 
	void CountPassengerNumberOfVisits(int vehicle_id)
	{
		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			if (it->second == 2)  // complete
			{
				int passenger_id = it->first;
#pragma omp critical 
				{
					g_passenger_number_of_visits[passenger_id] += 1;
					g_passenger_vehicle_visit_flag[passenger_id][vehicle_id] = 1;
				}
			}
		}
	}

	std::string generate_string_key()
	{
		stringstream s;

		s << " ";

		for (std::map<int, int>::iterator it = passenger_service_state.begin(); it != passenger_service_state.end(); ++it)
		{
			s << "_";

			s << it->first << "[" << it->second << "]";
		}
		string converted(s.str());
		return converted;
	}

	bool operator<(const CVSState &other) const
	{
		return LabelCost < other.LabelCost;
	}

};

class C_time_indexed_state_vector
{

public:
	int current_time;

	std::vector<CVSState> m_VSStateVector;
	//state string 1_1_1,state index 
	std::map<std::string, int> m_state_map;

	void Reset()
	{
		current_time = 0;
		m_VSStateVector.clear();
		m_state_map.clear();
	}

	int m_find_state_index(std::string string_key)
	{

		if (m_state_map.find(string_key) != m_state_map.end())
		{
			return m_state_map[string_key];
		}
		else
			return -1;  // not found
	}

	void update_state(CVSState new_element)
	{
		std::string string_key = new_element.generate_string_key();//if it is new, string is n100, no state index
		int state_index = m_find_state_index(string_key);

		if (state_index == -1)  // no such state at this time
		{
			// add new state
			state_index = m_VSStateVector.size();
			m_VSStateVector.push_back(new_element);
			m_state_map[string_key] = state_index;
		}
		else
		{//DP 
			if (new_element.LabelCost < m_VSStateVector[state_index].LabelCost)
			{
				m_VSStateVector[state_index].Copy(&new_element);
			}
		}
	}

	void Sort()
	{
		std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

		m_state_map.clear(); // invalid
	}

	void SortAndCleanEndingState(int BestKValue)
	{
		if (m_VSStateVector.size() > 2 * BestKValue)
		{
			std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

			m_state_map.clear(); // invalid
			m_VSStateVector.erase(m_VSStateVector.begin() + BestKValue, m_VSStateVector.end());
		}
	}

	float GetBestValue(int DualPriceFlag, int vehicle_id)
	{
		// LabelCost not PrimalCost when sorting
		std::sort(m_VSStateVector.begin(), m_VSStateVector.end());

		if (m_VSStateVector.size() >= 1)
		{
			std::string state_str = m_VSStateVector[0].generate_string_key();
			//0 means least cost 
			m_VSStateVector[0].CountPassengerNumberOfVisits(vehicle_id);
			if (DualPriceFlag == 1)
			{
				fprintf(g_pFileDebugLog, "Dual \t{{%s}}; Label Cost %f\n ",
					state_str.c_str(), m_VSStateVector[0].LabelCost);
			}

			else
			{
				fprintf(g_pFileDebugLog, "Primal \t{{%s}}; Label Cost %f\n",
					state_str.c_str(), m_VSStateVector[0].PrimalLabelCost);
			}

			if (DualPriceFlag == 1)
				return m_VSStateVector[0].LabelCost;
			else
				return m_VSStateVector[0].PrimalLabelCost;
		}
		else
			return _MAX_LABEL_COST;
	}
};

//vehicle state at time t
//C_time_indexed_state_vector g_time_dependent_state_vector[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_NODES][_MAX_NUMBER_OF_TIME_INTERVALS];

C_time_indexed_state_vector*** g_time_dependent_state_vector;

// for collecting the final feasible states accesible to the depot
C_time_indexed_state_vector g_ending_state_vector[_MAX_NUMBER_OF_VEHICLES];
C_time_indexed_state_vector g_ending_state_depot_vector[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_NODES];
C_time_indexed_state_vector g_ending_state_node_vector[_MAX_NUMBER_OF_VEHICLES][_MAX_NUMBER_OF_NODES];

float g_optimal_time_dependenet_dynamic_programming(
	int vehicle_id,
	int origin_node,
	int departure_time_beginning,
	int departure_time_ending,
	int destination_node,
	int arrival_time_beginning,
	int arrival_time_ending,
	int vehicle_capacity,
	//maximum choose
	int BestKSize,
	int DualCostFlag,
	int base_tree_vehicle_no
)
// time-dependent label correcting algorithm with double queue implementation
{
	if (arrival_time_ending > g_number_of_time_intervals || g_outbound_node_size[origin_node] == 0)
	{
		return _MAX_LABEL_COST;
	}

	for (int p = 1; p < g_number_of_passengers; p++)
	{//release all passengers to this vehicle 
		g_passenger_vehicle_visit_flag[p][vehicle_id] = 0;
	}
	//step 2: Initialization for origin node at the preferred departure time, at departure time
	for (int i = 0; i < g_number_of_nodes; i++)
	{
		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time

		{
			g_time_dependent_state_vector[vehicle_id][i][t].Reset();
		}

		if (g_node_depot_type[i] == 10)
		{
			g_ending_state_depot_vector[vehicle_id][i].Reset();
		}

		g_ending_state_node_vector[vehicle_id][i].Reset();

	}
	g_ending_state_vector[vehicle_id].Reset();

	if (base_tree_vehicle_no >= 1)// got the infor from tree for this vehicle
	{
		// copy the values

		g_ending_state_vector[vehicle_id] = g_ending_state_depot_vector[base_tree_vehicle_no][destination_node];//base_tree_vehicle_no is the vehicle class NO to generate the tree; destination node is to choose the path of the tree for all depots.
		g_ending_state_vector[vehicle_id].SortAndCleanEndingState(BestKSize);

		return g_ending_state_vector[vehicle_id].GetBestValue(DualCostFlag, vehicle_id);//Tony: we should sort the states to get the k best value.
	}

	CVSState element;

	element.current_node_id = origin_node;
	g_time_dependent_state_vector[vehicle_id][origin_node][departure_time_beginning].update_state(element);

	// step 3: //dynamic programming
	for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
	{
		int state_count = 0;
		cout << "v:" << vehicle_id << " time " << t << endl;
		for (int n = 0; n < g_number_of_nodes; n++)
		{
			// step 1: sort m_VSStateVector by labelCost for scan best k elements in step2 
			g_time_dependent_state_vector[vehicle_id][n][t].Sort();

			int state_node_size = min(BestKSize, g_time_dependent_state_vector[vehicle_id][n][t].m_VSStateVector.size());

			if (state_node_size >= 3 || BestKSize >= 3)
			{
				int trace_flag = 1;
			}
			// step 2: scan the best k elements
			for (int w_index = 0; w_index < state_node_size; w_index++)
			{
				CVSState* pElement = &(g_time_dependent_state_vector[vehicle_id][n][t].m_VSStateVector[w_index]);
				state_count++;

				if (state_count >= 1000 || BestKSize >= 3)
				{
					int trace_flag = 1;
				}

				int from_node = pElement->current_node_id;//current_node_id is not node n?

				// step 2.1 link from node to toNode
				for (int i = 0; i < g_outbound_node_size[from_node]; i++)
				{
					int to_node = g_outbound_node_id[from_node][i];
					int to_node_passenger_id = g_node_passenger_id[to_node];
					int to_node_type = g_node_type[to_node];
					//int link_no = g_outbound_link_no[from_node][i];//Tony: also the internal link no
					int link_no = g_link_no[from_node][to_node];

					int next_time = max(g_node_timestamp[to_node], t + g_link_time_dependent_travel_time[link_no][t]);
					//				int next_time = max(g_node_timestamp[to_node], t + g_link_free_flow_travel_time[link_no]);					

					if (to_node == 76)
					{
						int check_flag = 1;
					}

					// step 2.2. check feasibility of node type with the current element
					if (next_time <= arrival_time_ending)
					{

						if (to_node_passenger_id >= 1 && g_activity_node_starting_time[to_node] >= 0 && g_activity_node_ending_time[to_node] >= 0 &&
							g_vehicle_passenger_visit_allowed_flag[vehicle_id][to_node_passenger_id] == 1)
						{// address the passengers' state transitions
						 // passegner activity node: origin or destination

							// skip scanning when the origin/destination nodes arrival time is out of time window
							if (next_time > g_activity_node_ending_time[to_node])
							{
								continue;
							}

							//feasible state transitions 
							if ((to_node_type == 1 && pElement->GetPassengerServiceState(to_node_passenger_id) == 0)//pickup
								|| (to_node_type == 2 && pElement->GetPassengerServiceState(to_node_passenger_id) == 1))  // delivery)
							{//origin state is 0, then to 1; origin state is 1, then to 2.
								// step 2.3 label cost updating inside this function 

								// for pickup process
								if (to_node_type == 1)
								{
									// skip pickup when the vehicle if on its capacity
									
									if (g_time_dependent_state_vector[vehicle_id][n][t].m_VSStateVector[w_index].m_vehicle_capacity >= vehicle_capacity)// Tony: to_node? or from_node?
										continue;
									//waiting 
									if (next_time < g_activity_node_starting_time[to_node])
									{
										CVSState new_element;
										new_element.Copy(pElement);
										//new_element.MarkCarryingService(to_node_passenger_id, to_node_type, 0);//Tony: time 0 means the service doesn't begin

										new_element.current_node_id = to_node;
										new_element.passenger_service_state[to_node_passenger_id] = to_node_type;

										//for arriving at activity node and begin wait
										new_element.m_visit_time_sequence.push_back(next_time);
										new_element.m_visit_node_sequence.push_back(to_node);
										new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());

										//for wait until activity node's depature time
										new_element.m_visit_time_sequence.push_back(g_activity_node_starting_time[to_node]);
										new_element.m_visit_node_sequence.push_back(to_node);
										new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());
										new_element.MarkCarryingService(to_node_passenger_id, to_node_type, g_activity_node_starting_time[to_node]);

										new_element.CalculateLabelCost(vehicle_id);

										//int link_no = g_outbound_link_no[from_node][i];// Tony: link_no, from_node, i are internal node id
										new_element.m_visit_link_sequence.push_back(link_no);
										new_element.m_visit_link_sequence.push_back(-1);  // waiting arc

										g_time_dependent_state_vector[vehicle_id][to_node][g_activity_node_starting_time[to_node]].update_state(new_element);

										g_ending_state_node_vector[vehicle_id][to_node].update_state(new_element);
										continue;
									}
								}

								if (to_node_type == 2)
								{
									//waiting 
									if (next_time < g_activity_node_starting_time[to_node])
									{
										CVSState new_element;
										new_element.Copy(pElement);
										//new_element.MarkCarryingService(to_node_passenger_id, to_node_type, g_activity_node_starting_time[to_node]);

										new_element.current_node_id = to_node;
										new_element.passenger_service_state[to_node_passenger_id] = to_node_type;

										//for arriving at activity node and begin wait
										new_element.m_visit_time_sequence.push_back(next_time);
										new_element.m_visit_node_sequence.push_back(to_node);
										new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());

										//for wait until activity node's depature time
										new_element.m_visit_time_sequence.push_back(g_activity_node_starting_time[to_node]);
										new_element.m_visit_node_sequence.push_back(to_node);
										new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());
										new_element.m_visit_passenger_sequence.push_back(to_node_passenger_id);
										new_element.MarkCarryingService(to_node_passenger_id, to_node_type, g_activity_node_starting_time[to_node]);

										new_element.CalculateLabelCost(vehicle_id);

										//int link_no = g_outbound_link_no[from_node][i];
										new_element.m_visit_link_sequence.push_back(link_no);
										new_element.m_visit_link_sequence.push_back(-1);  // waiting arc

										g_time_dependent_state_vector[vehicle_id][to_node][g_activity_node_starting_time[to_node]].update_state(new_element);
										g_ending_state_node_vector[vehicle_id][to_node].update_state(new_element);
										continue;
									}

								}
								// do not need waiting
								CVSState new_element;
								new_element.Copy(pElement);
								new_element.MarkCarryingService(to_node_passenger_id, to_node_type, next_time);

								new_element.current_node_id = to_node;
								new_element.passenger_service_state[to_node_passenger_id] = to_node_type;
								new_element.m_visit_time_sequence.push_back(next_time);
								new_element.m_visit_node_sequence.push_back(to_node);
								new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());
								new_element.m_visit_passenger_sequence.push_back(to_node_passenger_id);

								new_element.CalculateLabelCost(vehicle_id);

								//int link_no = g_outbound_link_no[from_node][i];
								new_element.m_visit_link_sequence.push_back(link_no);

								g_time_dependent_state_vector[vehicle_id][to_node][next_time].update_state(new_element);// Tony: we can use this vehice-dependent node-dependent time-dependent state vector to output the state for each vehicle based on its path node sequence and path node time sequence.
								g_ending_state_node_vector[vehicle_id][to_node].update_state(new_element);
							}
						}

						else if (to_node_passenger_id < 1)
						{
							CVSState new_element;
							new_element.Copy(pElement);
							if (to_node != destination_node)
							{
								new_element.current_node_id = to_node;
								new_element.m_visit_time_sequence.push_back(next_time);
								new_element.m_visit_node_sequence.push_back(to_node);
								new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());

								new_element.CalculateLabelCost(vehicle_id);

								//int link_no = g_outbound_link_no[from_node][i];
								new_element.m_visit_link_sequence.push_back(link_no);

								g_time_dependent_state_vector[vehicle_id][to_node][next_time].update_state(new_element);
								g_ending_state_node_vector[vehicle_id][to_node].update_state(new_element);
							}
							else
							{
								if (next_time < arrival_time_beginning)
								{
									//for arriving at activity node and begin wait
									new_element.m_visit_time_sequence.push_back(next_time);
									new_element.m_visit_node_sequence.push_back(to_node);
									new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());
									int link_no = g_outbound_link_no[from_node][i];
									new_element.m_visit_link_sequence.push_back(link_no);

									//for wait until activity node's depature time
									new_element.m_visit_time_sequence.push_back(arrival_time_beginning);
									new_element.m_visit_node_sequence.push_back(to_node);
									new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());
									
									new_element.m_visit_link_sequence.push_back(-1);  // waiting arc
									new_element.CalculateLabelCost(vehicle_id);
									//g_ending_state_node_vector[vehicle_id][to_node].update_state(new_element);
								}
								else
								{
									new_element.m_visit_time_sequence.push_back(next_time);
									new_element.m_visit_node_sequence.push_back(to_node);
									new_element.m_visit_state_sequence.push_back(new_element.generate_string_key());
									
									//int link_no = g_outbound_link_no[from_node][i];
									new_element.m_visit_link_sequence.push_back(link_no);

									new_element.CalculateLabelCost(vehicle_id);
									//g_ending_state_node_vector[vehicle_id][to_node].update_state(new_element);
								}
								g_ending_state_vector[vehicle_id].update_state(new_element);// Tony: that is the ending state at the destination of this vechile;
								g_ending_state_vector[vehicle_id].SortAndCleanEndingState(BestKSize);
								g_ending_state_node_vector[vehicle_id][to_node].update_state(new_element);
							}
						}
					}

					// step 2.3. check depot type with the current element
					if (base_tree_vehicle_no == -100 /*this is the base tree to be created here*/ && next_time <= arrival_time_ending && g_node_depot_type[to_node] == 10)  // for all d
					{
						g_ending_state_depot_vector[vehicle_id][to_node] = g_ending_state_node_vector[vehicle_id][to_node];//Tony: it does not record  all depots but all to_node (depots) from the orign depopt to its vehilce ending time using k-beam searching, so it is possible that some depots cannot be served.
						g_ending_state_depot_vector[vehicle_id][to_node].SortAndCleanEndingState(BestKSize);
					}				
				}
			}
		}  // for all nodes

		cout << "state count:" << state_count << endl;

	} // for all time t

// no back
	return g_ending_state_vector[vehicle_id].GetBestValue(DualCostFlag, vehicle_id);// Tony: the ending state is based on the destionation of that vehicle;
}

void g_ReadInputData()
{
	// initialization
	for (int i = 0; i < _MAX_NUMBER_OF_NODES; i++)
	{
		g_outbound_node_size[i] = 0;
		g_inbound_node_size[i] = 0;
	}

	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0
	int interval_node_no = 1;

	// step 1: read node file 
	CCSVParser parser;
	if (parser.OpenCSVFile("input_node.csv", true))
	{
		std::map<int, int> node_id_map;

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			string name;

			int node_type = 0;
			int node_id;
			double X;
			double Y;
			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;

			if (node_id <= 0 || g_number_of_nodes >= _MAX_NUMBER_OF_NODES)
			{
				cout << "node_id " << node_id << " is out of range" << endl;
				g_ProgramStop();
			}
			g_internal_node_no_map[node_id] = interval_node_no;

			g_external_node_id_map[interval_node_no] = node_id;
			parser.GetValueByFieldName("node_type", node_type);
			if (node_type != 0)
			{
				cout << "node_type in input_node.csv should be 0!" << endl;
			}
			g_node_type[interval_node_no] = node_type;

			int depot_type = 0;
			parser.GetValueByFieldName("depot_type", depot_type);
			g_node_depot_type[interval_node_no] = depot_type;

			if (depot_type == 10)
				g_number_of_depots++;

			parser.GetValueByFieldName("x", X);
			parser.GetValueByFieldName("y", Y);

			g_number_of_nodes++;
			interval_node_no++;

			if (g_number_of_nodes % 1000 == 0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}

		cout << "number of nodes = " << g_number_of_nodes << "; # of depots =" << g_number_of_depots << endl;

		g_number_of_physical_nodes = g_number_of_nodes;

		fprintf(g_pFileOutputLog, "number of nodes =,%d\n", g_number_of_nodes);
		parser.CloseCSVFile();
	}

	// step 2: read link file 
	if (parser.OpenCSVFile("input_link.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			if (parser.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;

			if (from_node_id <= 0)
			{
				cout << "from_node_id " << from_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (to_node_id <= 0)
			{
				cout << "to_node_id " << to_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (g_internal_node_no_map.find(from_node_id) == g_internal_node_no_map.end())
			{
				cout << "from_node_id " << from_node_id << " has not been defined in node block" << endl;
				g_ProgramStop();
			}

			if (g_internal_node_no_map.find(to_node_id) == g_internal_node_no_map.end())
			{
				cout << "to_node_id " << to_node_id << " has not been defined in node block" << endl;
				g_ProgramStop();
			}

			if (g_internal_node_no_map[from_node_id] >= _MAX_NUMBER_OF_NODES)
			{
				cout << "from_node_id " << from_node_id << " is out of range" << endl;
				g_ProgramStop();
			}
			if (g_internal_node_no_map[to_node_id] >= _MAX_NUMBER_OF_NODES)
			{
				cout << "to_node_id " << to_node_id << " is out of range" << endl;
				g_ProgramStop();
			}
			// add the to node id into the outbound (adjacent) node list

			int direction = 1;
			parser.GetValueByFieldName("direction", direction);

			if (direction <= -2 || direction >= 2)
			{
				cout << "direction " << direction << " is out of range" << endl;
				g_ProgramStop();
			}

			for (int link_direction = -1; link_direction <= 1; link_direction += 2)  // called twice; -1 direction , 1 direction 
			{
				if (direction == -1 && link_direction == 1)
					continue; // skip

				if (direction == 1 && link_direction == -1)
					continue; // skip

				// then if  direction == 0 or 2 then create the corresponding link

				int directional_from_node_id = g_internal_node_no_map[from_node_id];
				int directional_to_node_id = g_internal_node_no_map[to_node_id];

				if (link_direction == -1) // reverse direction;
				{
					directional_from_node_id = g_internal_node_no_map[to_node_id];
					directional_to_node_id = g_internal_node_no_map[from_node_id];
				}

				g_outbound_node_id[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = directional_to_node_id;
				g_outbound_link_no[directional_from_node_id][g_outbound_node_size[directional_from_node_id]] = g_number_of_links;

				g_outbound_node_size[directional_from_node_id]++;

				g_inbound_node_id[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = directional_from_node_id;
				g_inbound_link_no[directional_to_node_id][g_inbound_node_size[directional_to_node_id]] = g_number_of_links;
				g_inbound_node_size[directional_to_node_id]++;

				float link_length = 1;
				int number_of_lanes = 1;
				int mode_code = 0;
				float capacity_per_time_interval = 1;
				float travel_time = 1.0;
				float speed = 1;
				float jam_density = 200;

				parser.GetValueByFieldName("length", link_length);
				parser.GetValueByFieldName("number_of_lanes", number_of_lanes);
				parser.GetValueByFieldName("mode_code", mode_code);
				parser.GetValueByFieldName("lane_capacity_in_vhc_per_hour", capacity_per_time_interval);

				parser.GetValueByFieldName("speed_limit_in_mph", speed);
				if (speed >= 70)
					speed = 70;

				if (speed <= 25)
					speed = 25;

				travel_time = link_length * 60 / max(1, speed);

				parser.GetValueByFieldName("jam_density", jam_density);

				if (travel_time > 100)
				{
					cout << "travel_time > 100";
					g_ProgramStop();
				}

				g_link_from_node_number[g_number_of_links] = directional_from_node_id;
				g_link_to_node_number[g_number_of_links] = directional_to_node_id;
				g_link_no[directional_from_node_id][directional_to_node_id] = g_number_of_links;

				g_link_free_flow_travel_time[g_number_of_links] = max(1, travel_time + 0.5);   // at least 1 min, round to nearest integers
				g_link_free_flow_travel_time_float_value[g_number_of_links] = travel_time;

				g_link_link_length[g_number_of_links] = link_length;
				g_link_number_of_lanes[g_number_of_links] = number_of_lanes;
				g_link_mode_code[g_number_of_links] = mode_code;
				g_link_capacity_per_time_interval[g_number_of_links] = capacity_per_time_interval;
//				g_link_capacity_per_time_interval_nodes[g_internal_node_no_map[from_node_id]][g_internal_node_no_map[to_node_id]]= capacity_per_time_interval;
				g_link_speed[g_number_of_links] = speed;

				// increase the link counter by 1
				g_number_of_links++;

				if (g_number_of_links % 1000 == 0)
					cout << "reading " << g_number_of_links << " links.. " << endl;
			}
		}

		cout << "number of links = " << g_number_of_links << endl;

		fprintf(g_pFileOutputLog, "number of links =,%d\n", g_number_of_links);
		parser.CloseCSVFile();
	}

	//step 3: read agent file including vehicle and passenger
	if (parser.OpenCSVFile("input_agent.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int agent_id;
			parser.GetValueByFieldName("agent_id", agent_id);

			if (agent_id == 0)  // break for empty line
				break;

			int agent_type = 0;
			parser.GetValueByFieldName("agent_type", agent_type);
			int external_from_node_id;
			int external_to_node_id;
			int from_node_id;
			int to_node_id;

			if (agent_type == 0) //passenger
			{
				int pax_no = g_number_of_passengers + 1;

				g_internal_agent_no_map[agent_id] = pax_no;
				g_external_passenger_id_map[pax_no] = agent_id;

				if (pax_no >= _MAX_NUMBER_OF_PASSENGERS)
				{
					cout << "Agent+ can handle  " << _MAX_NUMBER_OF_PASSENGERS << "passengers" << endl;
					g_ProgramStop();
				}

				parser.GetValueByFieldName("from_node_id", external_from_node_id);
				parser.GetValueByFieldName("to_node_id", external_to_node_id);

				from_node_id = g_internal_node_no_map[external_from_node_id];
				to_node_id = g_internal_node_no_map[external_to_node_id];

				g_passenger_origin_node[pax_no] = from_node_id;
				g_passenger_destination_node[pax_no] = to_node_id;

				parser.GetValueByFieldName("departure_time_start", g_passenger_departure_time_beginning[pax_no]);
				int departure_time_window = 0;
				parser.GetValueByFieldName("departure_time_window", departure_time_window);
				g_passenger_departure_time_ending[pax_no] = max(0, departure_time_window) + g_passenger_departure_time_beginning[pax_no];
				g_passenger_order_time[pax_no] = g_passenger_departure_time_beginning[pax_no];
				parser.GetValueByFieldName("arrival_time_start", g_passenger_arrival_time_beginning[pax_no]);

				int arrival_time_window = 0;
				parser.GetValueByFieldName("arrival_time_window", arrival_time_window);

				g_passenger_arrival_time_ending[pax_no] = g_passenger_arrival_time_beginning[pax_no] + arrival_time_window;
				g_passenger_arrival_time_ending[pax_no] = max(g_passenger_arrival_time_ending[pax_no], g_passenger_arrival_time_beginning[pax_no]);
				g_number_of_time_intervals = max(g_passenger_arrival_time_ending[pax_no] + 10, g_number_of_time_intervals);

				if (g_number_of_time_intervals > _MAX_NUMBER_OF_TIME_INTERVALS - 1)
				{
					cout << "g_number_of_time_intervals = " << g_number_of_time_intervals << " is out of range of _MAX_NUMBER_OF_TIME_INTERVALS " << _MAX_NUMBER_OF_TIME_INTERVALS << endl;
					g_ProgramStop();
				}

				//add virtual node and link for passenger
				int new_artifical_pasenger_origin_id = g_add_new_node_origin(pax_no, g_passenger_departure_time_beginning[pax_no], g_passenger_departure_time_ending[pax_no]);
				g_add_new_link(g_passenger_origin_node[pax_no], new_artifical_pasenger_origin_id, pax_no);  // pick up link
				g_add_new_link(new_artifical_pasenger_origin_id, g_passenger_origin_node[pax_no], pax_no);

				int new_artifical_pasenger_destination_id = g_add_new_node_destination(pax_no, g_passenger_arrival_time_beginning[pax_no], g_passenger_arrival_time_ending[pax_no]);
				g_add_new_link(g_passenger_destination_node[pax_no], new_artifical_pasenger_destination_id, pax_no*(-1));  // delivery link
				g_add_new_link(new_artifical_pasenger_destination_id, g_passenger_destination_node[pax_no], pax_no*(-1));

				parser.GetValueByFieldName("base_profit", g_passenger_base_profit[pax_no]);

				int request_vehicle_id = -1;
				parser.GetValueByFieldName("requested_vehicle_id", request_vehicle_id);

				std::vector<int> prohibited_vehicle_id;
				string prohibited_vehicle_id_list;
				parser.GetValueByFieldName("prohibited_vehicle_id_list", prohibited_vehicle_id_list);

				g_number_of_passengers++;

				cout << "g_number_of_passengers = " << g_number_of_passengers << endl;
			}
			else
			{  // vehicle

				int vehicle_no = g_number_of_vehicles + 1;

				if (vehicle_no >= _MAX_NUMBER_OF_VEHICLES - 1)
				{
					cout << "please check vehicle_no >= _MAX_NUMBER_OF_VEHICLES =" << _MAX_NUMBER_OF_VEHICLES << endl;
					g_ProgramStop();
				}

				g_external_vehicle_id_map[vehicle_no] = agent_id;
				g_external_agent_id_to_vehicle_id_map[agent_id] = vehicle_no;

				if (agent_type == 1)
				{
					parser.GetValueByFieldName("from_node_id", external_from_node_id);
					parser.GetValueByFieldName("to_node_id", external_to_node_id);

					from_node_id = g_internal_node_no_map[external_from_node_id];// internal from_node_id
					to_node_id = g_internal_node_no_map[external_to_node_id];

					g_vehicle_origin_node[vehicle_no] = from_node_id;
					g_vehicle_destination_node[vehicle_no] = to_node_id;

					int tree_agent_id = 0;
					parser.GetValueByFieldName("tree_agent_id", tree_agent_id, false);

					g_vehicle_depot_tree_id[vehicle_no] = -1;  // default value, internal value

					if (tree_agent_id == -100)
						g_vehicle_depot_tree_id[vehicle_no] = -100;   // base tree flag 
					else
					{
						int tree_vehicle_id = tree_agent_id;

						if (tree_agent_id >= 0 && g_external_agent_id_to_vehicle_id_map.find(tree_agent_id) != g_external_agent_id_to_vehicle_id_map.end())
						{
							tree_vehicle_id = g_external_agent_id_to_vehicle_id_map[tree_agent_id];

							if (g_vehicle_depot_tree_id[tree_vehicle_id] != -100)
							{
								// he is the the parent with base tree info
								cout << "please check agent_id" << agent_id << " with tree_agent_id = " << tree_agent_id << " which is not the parent vehicle with base tree data" << endl;
								g_ProgramStop();
							}

						}
						g_vehicle_depot_tree_id[vehicle_no] = tree_vehicle_id;  // convert tree agent id to the internal vehicle id regarding the base tree id to copy from 
					}

					parser.GetValueByFieldName("departure_time_start", g_vehicle_departure_time_beginning[vehicle_no]);
					int departure_time_window = 0;
					parser.GetValueByFieldName("departure_time_window", departure_time_window);
					g_vehicle_departure_time_ending[vehicle_no] = g_vehicle_departure_time_beginning[vehicle_no] + max(1, departure_time_window);
					g_vehicle_arrival_time_beginning[vehicle_no] = -1;
					parser.GetValueByFieldName("arrival_time_start", g_vehicle_arrival_time_beginning[vehicle_no]);

					if (g_vehicle_arrival_time_beginning[vehicle_no] < 0)
					{
						cout << "Vehicle data must have values in field arrival_time_start in file input_agent.csv!" << endl;
						g_ProgramStop();
					}

					int arrival_time_window = -1;
					parser.GetValueByFieldName("arrival_time_window", arrival_time_window);

					if (arrival_time_window < 0)
					{
						cout << "Vehicle data must have values in field arrival_time_window in file input_agent.csv!" << endl;
						g_ProgramStop();
					}
					g_vehicle_arrival_time_ending[vehicle_no] = g_vehicle_arrival_time_beginning[vehicle_no] + max(1, arrival_time_window);

					g_number_of_time_intervals = max(g_vehicle_arrival_time_ending[vehicle_no] + 10, g_number_of_time_intervals);

					if (g_number_of_time_intervals > _MAX_NUMBER_OF_TIME_INTERVALS - 1)
					{
						cout << "g_number_of_time_intervals = " << g_number_of_time_intervals << " is out of range of _MAX_NUMBER_OF_TIME_INTERVALS " << _MAX_NUMBER_OF_TIME_INTERVALS << endl;
						g_ProgramStop();
					}

					if (g_vehicle_arrival_time_ending[vehicle_no] < g_vehicle_departure_time_beginning[vehicle_no] + 13)  // we should use a shortest path travel time to check. 
					{
						cout << "warning: Arrival time for vehicle " << vehicle_no << " should be " << g_vehicle_departure_time_beginning[vehicle_no] + 120 << endl;
						g_vehicle_arrival_time_ending[vehicle_no] = g_vehicle_departure_time_beginning[vehicle_no] + 60;
						//				g_ProgramStop();
					}

					if (tree_agent_id <= 0)  // if we do not need to use the base tree results, create virtual origin and destination depots
					{
						//add virtual node and link for vehicle
						int new_artifical_vehicle_origin_id = g_add_new_node(vehicle_no, g_vehicle_departure_time_beginning[vehicle_no], g_vehicle_departure_time_ending[vehicle_no]);
						g_add_new_link(new_artifical_vehicle_origin_id, g_vehicle_origin_node[vehicle_no], 100);
						//g_vehicle_depot_origin_node[vehicle_no] = g_number_of_nodes;

						g_vehicle_depot_origin_node[vehicle_no] = g_vehicle_origin_node[vehicle_no];

						int new_artifical_vehicle_destination_id = g_add_new_node(vehicle_no, g_vehicle_arrival_time_beginning[vehicle_no], g_vehicle_arrival_time_ending[vehicle_no]);
						g_add_new_link(g_vehicle_destination_node[vehicle_no], new_artifical_vehicle_destination_id, 101);  // delivery link
						//g_vehicle_depot_destination_node[vehicle_no] = g_number_of_nodes;

						g_vehicle_depot_destination_node[vehicle_no] = g_vehicle_destination_node[vehicle_no];

					}
					else  // tree based calculation
					{
						g_vehicle_depot_origin_node[vehicle_no] = g_vehicle_origin_node[vehicle_no];//origin node--physical node
						g_vehicle_depot_destination_node[vehicle_no] = g_vehicle_destination_node[vehicle_no];// destinaiton node--physical node
					}
					g_activity_node_flag[g_vehicle_depot_origin_node[vehicle_no]] = 1;
					g_activity_node_flag[g_vehicle_depot_destination_node[vehicle_no]] = 1;
					g_activity_node_ending_time[g_vehicle_depot_origin_node[vehicle_no]] = g_vehicle_departure_time_ending[vehicle_no];
					g_activity_node_ending_time[g_vehicle_depot_destination_node[vehicle_no]] = g_vehicle_arrival_time_ending[vehicle_no];

					g_vehicle_capacity[vehicle_no] = -1;

					int capacity = -1;
					parser.GetValueByFieldName("capacity", capacity);
					g_vehicle_capacity[vehicle_no] = min(1, capacity);
					if (g_vehicle_capacity[vehicle_no] < 0)
					{
						cout << "Vehicle data must have values in field capacity in file input_agent.csv!" << endl;
						g_ProgramStop();
					}
					parser.GetValueByFieldName("VOIVTT_per_hour", g_VOIVTT_per_hour[vehicle_no]);
					parser.GetValueByFieldName("VOWT_per_hour", g_VOWT_per_hour[vehicle_no]);

					if (g_max_vehicle_capacity < g_vehicle_capacity[vehicle_no])
						g_max_vehicle_capacity = g_vehicle_capacity[vehicle_no];
				}
				g_number_of_vehicles++;

				cout << "g_number_of_vehicles = " << g_number_of_vehicles << endl;
				//g_updated_number_of_time_intervals = g_number_of_time_intervals;
			}
		}
		parser.CloseCSVFile();
	}

	cout << "Updated g_number_of_nodes = " << g_number_of_nodes << endl;
	cout << "Updated g_number_of_links = " << g_number_of_links << endl;

	fprintf(g_pFileOutputLog, "number of passengers =,%d\n", g_number_of_passengers);
	fprintf(g_pFileOutputLog, "number of vehicles =,%d\n", g_number_of_vehicles);

	cout << "read " << g_number_of_nodes << " nodes, " << g_number_of_links << " links" << ", " << g_number_of_passengers << " passengers, " << g_number_of_vehicles << "vehicles" << endl;
	fprintf(g_pFileDebugLog, "Network has %d nodes, %d links, %d  passengers, %d vehicles\n\n",
		g_number_of_nodes, g_number_of_links, g_number_of_passengers, g_number_of_vehicles);

	//output activity service link(virtual) in debug.txt
	for (int link = 0; link < g_number_of_links; link++)  // for each link (i,j)
	{
		int from_node = g_link_from_node_number[link];

		int to_node = g_link_to_node_number[link];

		if (g_link_service_code[link] != 0)
		{
			fprintf(g_pFileDebugLog, "link no.%d,  %d->%d, service code %d\n",
				link + 1,
				from_node,
				to_node,
				g_link_service_code[link]);
		}
	}

	// initialization
	for (int t = 0; t < g_number_of_time_intervals; t++)
	{
		for (int link = 0; link < g_number_of_links; link++)
		{
			g_link_time_dependent_travel_time[link][t] = g_link_free_flow_travel_time[link]- g_link_time_dependent_margianl_cost[link][t];
			g_link_cap[link][t] = g_link_capacity_per_time_interval[link];
		}
	}
	fprintf(g_pFileDebugLog, "\n");
}

void TD_link_travel_time_update(int vehicle_id)
{
	int m_from_node;
	//int m_to_node;
	int m_from_time;
	//int m_to_time;
	int m_link_no;
	if (g_ending_state_vector[vehicle_id].m_VSStateVector.size() >= 1)
	{
		for (int i = 0; i < g_ending_state_vector[vehicle_id].m_VSStateVector[0].m_visit_link_sequence.size(); i++)
		{
			m_from_time = g_ending_state_vector[vehicle_id].m_VSStateVector[0].m_visit_time_sequence[i];
			//m_to_time = g_ending_state_vector[vehicle_id].m_VSStateVector[0].m_visit_time_sequence[i + 1];

			int link_no = g_ending_state_vector[vehicle_id].m_VSStateVector[0].m_visit_link_sequence[i];

			if (link_no >= 0)  // no waiting arc of -1
			{
				g_link_cap[link_no][m_from_time] = max(0, g_link_cap[link_no][m_from_time] - 1);

				if (g_link_cap[link_no][m_from_time] == 0)
				{
					g_link_time_dependent_travel_time[link_no][m_from_time] = _MAX_LABEL_COST;
				}
			}
		}
	}
}

bool g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables()  // with varaible y only
{
	cout << "Start scheduling passengers by Lagrangian Relaxation method" << endl;
	g_SolutionStartTime = CTime::GetCurrentTime();

	g_number_of_LR_iterations = 1;
	float StepSize = 1;

	//loop for each LR iteration
	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{
		if ((LR_iteration + 1) % 5 == 0)
			std::cout << LR_iteration + 1 << "/" << g_number_of_LR_iterations << std::endl;

		// reset the vertex visit count
		double LR_global_lower_bound = 0;

		float TotalTripPrice = 0;
		float TotalTravelTime = 0;
		double TotalWaitingTimeCost = 0;

		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_passenger_number_of_visits[p] = 0;
		}

		for (int v = 1; v <= g_number_of_vehicles; v++)
		{

			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				g_vehicle_passenger_visit_allowed_flag[v][p] = 1;
				local_vehicle_passenger_additional_profit[v][p] = 0;
			}
		}

		//#pragma omp parallel for
		for (int v = 1; v <= g_number_of_vehicles; v++)
		{
			fprintf(g_pFileDebugLog,
				"\nDebug: LB iteration %d, Vehicle %d performing DP: origin %d -> destination %d\n ",
				LR_iteration,
				v,
				g_vehicle_origin_node[v],
				g_vehicle_destination_node[v]);

			float path_cost_by_vehicle_v =
				g_optimal_time_dependenet_dynamic_programming  //DP for each vehicle
				(
					v,
					g_vehicle_depot_origin_node[v],
					g_vehicle_departure_time_beginning[v],
					g_vehicle_departure_time_ending[v],
					g_vehicle_depot_destination_node[v],
					g_vehicle_arrival_time_beginning[v],
					g_vehicle_arrival_time_ending[v],
					g_vehicle_capacity[v],
					2,
					1,
					g_vehicle_depot_tree_id[v]);

			if (path_cost_by_vehicle_v < _MAX_LABEL_COST)
				LR_global_lower_bound += path_cost_by_vehicle_v;

			//step 1: Extract the time-dependent path link sequence
			//step 2: Update time-dependent link capacity. if the capacity is 0, the link travel cost is infinite.

			// output vehicle passing node sequence
			if (g_vehicle_depot_tree_id[v] >= 1) // using base tree
				TD_link_travel_time_update(g_vehicle_depot_tree_id[v]);
			else
				TD_link_travel_time_update(v);

			//fprintf(g_pFileDebugLog, "LR_global_lower_bound += path_cost_by_vehicle_v, %f, %f", path_cost_by_vehicle_v, LR_global_lower_bound);
		}

		/*for (int p = 1; p <= g_number_of_passengers; p++)
		{
			LR_global_lower_bound += g_passenger_base_profit[p];
		}*/

		fprintf(g_pFileDebugLog, "LR_global_lower_bound = , %f", LR_global_lower_bound);
		g_best_lower_bound = max(g_best_lower_bound, LR_global_lower_bound);  // keep the best lower bound till current iteration

		//update step size for the next LR_iteration
		//for (int p = 1; p <= g_number_of_passengers; p++)
		//{
		//	StepSize = 1 / (LR_iteration + 1.0f);
		//	g_minimum_subgradient_step_size = 0.1;
		//	if (StepSize < g_minimum_subgradient_step_size)  //1.3.1 keep the minimum step size
		//	{
		//		StepSize = g_minimum_subgradient_step_size;
		//	}

		//	int constant = 10;
		//	g_passenger_base_profit[p] -= constant*StepSize * (g_passenger_number_of_visits[p] - 1);
		//}
	}
	// end of LR iterations
	return true;
}

bool g_upper_bound_generation()
{
	// generate upper bound
	fprintf(g_pFileDebugLog, "\nGenerate upper bound\n");

	double LR_global_upper_bound = 0;
	for (int p = 1; p <= g_number_of_passengers; p++)
	{
		g_passenger_number_of_visits[p] = 0;
	}

	for (int v = 1; v <= g_number_of_vehicles; v++)
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			g_vehicle_passenger_visit_allowed_flag[v][p] = 1;
		}

	// sequential DP for each vehicle, based on the LR prices 
	for (int v = 1; v <= g_number_of_vehicles; v++)
	{
		float path_cost_by_vehicle_v =
			g_optimal_time_dependenet_dynamic_programming
			(
				v,
				g_vehicle_depot_origin_node[v],
				g_vehicle_departure_time_beginning[v],
				g_vehicle_departure_time_ending[v],
				g_vehicle_depot_destination_node[v],
				g_vehicle_arrival_time_beginning[v],
				g_vehicle_arrival_time_ending[v],
				g_vehicle_capacity[v],
				5,
				0,
				g_vehicle_depot_tree_id[v]);
		LR_global_upper_bound += path_cost_by_vehicle_v;
		for (int p = 1; p <= g_number_of_passengers; p++)
		{
			if (g_passenger_number_of_visits[p] == 0)
				LR_global_upper_bound += 30;
		}

		g_vehicle_path_cost[v] = path_cost_by_vehicle_v;
		fprintf(g_pFileDebugLog, "LR_global_upper_bound += path_cost_by_vehicle_%d, %f, %f", v, path_cost_by_vehicle_v, LR_global_upper_bound);

		for (int t = 0; t < _MAX_NUMBER_OF_TIME_INTERVALS; t++)
		{
			for (int i = 0; i < g_number_of_nodes; i++)
			{
				for (int j = 0; j < g_time_dependent_state_vector[v][i][t].m_VSStateVector.size(); j++)
				{
					string temp = g_time_dependent_state_vector[v][i][t].m_VSStateVector[j].generate_string_key();
					fprintf(g_pFile_PathLog, "\n%d,%d,%d,%s",
						v, i, t, g_time_dependent_state_vector[v][i][t].m_VSStateVector[j].generate_string_key().c_str());
				}
			}
		}
		if (v < g_number_of_vehicles+1)  // mark the passsengers have been visited
		{
			for (int p = 1; p <= g_number_of_passengers; p++)
			{
				if (g_passenger_number_of_visits[p] >= 1)
				{
					g_vehicle_passenger_visit_allowed_flag[v + 1][p] = 0;  // not allowed to visit
					fprintf(g_pFileDebugLog, "\nupper bound generation, for vehicle %d, pax %d is not allowed or no needed to serve",
						v + 1, p);
				}
			}
		}
		TD_link_travel_time_update(v);
	}  //end of vehicle v

	g_best_upper_bound = min(g_best_upper_bound, LR_global_upper_bound);  // keep the best lower bound till current iteration

	CTimeSpan ctime = CTime::GetCurrentTime() - g_SolutionStartTime;
	cout << "\nComputational time:," << ctime.GetTotalSeconds() << endl;
	return true;
}

void g_output_optimization_result()
{
	//output vehicle's path_node_seq and path_time_seq for Upperbound
	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		if (g_ending_state_vector[j].m_VSStateVector.size() == 0)
			continue;
		/*string ss = to_string(j);
		char *cstr = &ss[0u];*/

		//output vehicle passing node sequence
		fprintf(g_pFile_OutputAgentLog, "\n%d", j);
		fprintf(g_pFile_OutputAgentLog, ",");
		fprintf(g_pFile_OutputAgentLog, "%d", g_external_vehicle_id_map[j]);
		fprintf(g_pFile_OutputAgentLog, ",");
		fprintf(g_pFile_OutputAgentLog, "%d", 1);
		fprintf(g_pFile_OutputAgentLog, ",");
		fprintf(g_pFile_OutputAgentLog, "%d;", g_external_node_id_map[g_vehicle_origin_node[j]]);
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size(); i++)
		{
			if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]] == 0) //physical nodes
			{
				
				fprintf(g_pFile_OutputAgentLog, "%d;", g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]]);
			}
			else //pickup and drop-off nodes
			{
				
				fprintf(g_pFile_OutputAgentLog, "%d;", g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]);
			}
		}
		fprintf(g_pFile_OutputAgentLog, ",");

		//output vehicle passing node time sequence
		fprintf(g_pFile_OutputAgentLog, "%d;", g_vehicle_departure_time_beginning[j]);
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence.size(); i++)
		{			
			fprintf(g_pFile_OutputAgentLog, "%d;", g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i]);
		}
		fprintf(g_pFile_OutputAgentLog, ",");

		//output vehicle passing node state sequence
		fprintf(g_pFile_OutputAgentLog, "%s;", " ");
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_state_sequence.size(); i++)
		{
			std::string str = g_ending_state_vector[j].m_VSStateVector[0].m_visit_state_sequence[i];

			char *cstr = &str[0u];
			fprintf(g_pFile_OutputAgentLog, "%s;", cstr);
		}
		fprintf(g_pFile_OutputAgentLog, ",");

		//output vehicle passing link sequence		
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_link_sequence.size(); i++)
		{
			int link_no = g_ending_state_vector[j].m_VSStateVector[0].m_visit_link_sequence[i];
			fprintf(g_pFile_OutputAgentLog, "%d;", link_no);
		}
		fprintf(g_pFile_OutputAgentLog, ",");

		// output vehicle path cost
		
		fprintf(g_pFile_OutputAgentLog, "%.2f;", g_vehicle_path_cost[j]);
		fprintf(g_pFile_OutputAgentLog, ",");

		//output vehicle passing passenger sequence
		
		vector <int> served_passenger_id;
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size(); i++)
		{
			int passenger_id = g_node_passenger_id[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]];
			if (g_ending_state_vector[j].m_VSStateVector[0].passenger_service_state[passenger_id] != 2)
				continue;
			served_passenger_id.push_back(g_node_passenger_id[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]]);			
		}
		
		sort(served_passenger_id.begin(), served_passenger_id.end());
		served_passenger_id.erase(unique(served_passenger_id.begin(), served_passenger_id.end()), served_passenger_id.end());

		for (int i = 0; i < served_passenger_id.size(); i++)
		{
			fprintf(g_pFile_OutputAgentLog, "[%d]", g_external_passenger_id_map[served_passenger_id[i]]);
		}
	}

	//output vehicle's path_node_time_incidence matrix
	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		if (g_ending_state_vector[j].m_VSStateVector.size() == 0)
			continue;

		fprintf(g_pFile_OutputAgentPathArcLog, "\n%d", g_external_vehicle_id_map[j]);
		fprintf(g_pFile_OutputAgentPathArcLog, ",");
		fprintf(g_pFile_OutputAgentPathArcLog, "%d", 1);
		fprintf(g_pFile_OutputAgentPathArcLog, ",");
		fprintf(g_pFile_OutputAgentPathArcLog, "%.2f", g_ending_state_vector[j].m_VSStateVector[0].PrimalLabelCost);
		fprintf(g_pFile_OutputAgentPathArcLog, ",");

		int arc_origin, arc_origin_next, arc_dep, arc_dep_next;
		float arc_capacity;
		arc_origin = g_external_node_id_map[g_vehicle_origin_node[j]];
		if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[0]] == 0) //physical nodes
		{
			arc_origin_next = g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[0]];
		}
		else //pickup and drop-off nodes
		{
			arc_origin_next = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[0];
		}

		arc_dep = g_vehicle_departure_time_beginning[j];
		arc_dep_next= g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[0];
		
		fprintf(g_pFile_OutputAgentPathArcLog, "%d.%d.%d.%d", arc_origin, arc_origin_next, arc_dep, arc_dep_next);
		fprintf(g_pFile_OutputAgentPathArcLog, ",");
		fprintf(g_pFile_OutputAgentPathArcLog, "%d", 1);
		fprintf(g_pFile_OutputAgentPathArcLog, ",");
		int link_no_new = g_ending_state_vector[j].m_VSStateVector[0].m_visit_link_sequence[0];
		if (link_no_new == -1)
		{
			arc_capacity = 99999.00;
		}
		else
		{
			arc_capacity = g_link_capacity_per_time_interval[link_no_new];
		}
		
		fprintf(g_pFile_OutputAgentPathArcLog, "%.2f", arc_capacity);		

		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size()-1; i++)
		{
			fprintf(g_pFile_OutputAgentPathArcLog, "\n%d", g_external_vehicle_id_map[j]);
			fprintf(g_pFile_OutputAgentPathArcLog, ",");
			fprintf(g_pFile_OutputAgentPathArcLog, "%d", 1);
			fprintf(g_pFile_OutputAgentPathArcLog, ",");
			fprintf(g_pFile_OutputAgentPathArcLog, "%.2f", g_ending_state_vector[j].m_VSStateVector[0].PrimalLabelCost);
			fprintf(g_pFile_OutputAgentPathArcLog, ",");
			int arc_i, arc_j, arc_t, arc_s;

			if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]] == 0) //physical nodes
			{
				arc_i = g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]];				
			}
			else //pickup and drop-off nodes
			{
				arc_i = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i];				
			}

			if (g_node_type[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i+1]] == 0) //physical nodes
			{
				arc_j = g_external_node_id_map[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i + 1]];
			}
			else //pickup and drop-off nodes
			{								
				arc_j = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i + 1];
			}

			arc_t = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i];
			arc_s = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i+1];

			fprintf(g_pFile_OutputAgentPathArcLog, "%d.%d.%d.%d", arc_i, arc_j, arc_t, arc_s);
			fprintf(g_pFile_OutputAgentPathArcLog, ",");
			fprintf(g_pFile_OutputAgentPathArcLog, "%d", 1);
			fprintf(g_pFile_OutputAgentPathArcLog, ",");

			int link_no_new = g_ending_state_vector[j].m_VSStateVector[0].m_visit_link_sequence[i+1];
			if (link_no_new == -1)
			{
				arc_capacity = 99999.00;
			}
			else
			{
				arc_capacity = g_link_capacity_per_time_interval[link_no_new];
			}

			fprintf(g_pFile_OutputAgentPathArcLog, "%.2f", arc_capacity);
		}
	}

	fprintf(g_pFileDebugLog, "Summary: Lower Bound = %f, upper Bound = %f, gap = %f, relative_gap = %.3f%%\n",

		g_best_lower_bound,
		g_best_upper_bound,
		(g_best_upper_bound - g_best_lower_bound),
		(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0

	);
	fprintf(g_pFileDebugLog, "******************************************************************************************************************\n\n");
	fprintf(g_pFileOutputLog, "%f,%f,%f,%.3f%%,",

		g_best_lower_bound,
		g_best_upper_bound,
		(g_best_upper_bound - g_best_lower_bound),
		(g_best_upper_bound - g_best_lower_bound) / max(1, g_best_upper_bound) *100.0
	);
	cout << "End of Lagrangian Iteration Process " << endl;

}

void g_Output_GAMS_Set_needs()
{
	int i_max, t_max, v_max, k_max, p_max;

	i_max = g_number_of_nodes;
	t_max = g_number_of_time_intervals;
	v_max = g_number_of_vehicles;
	k_max = g_number_of_k_paths_iterations;
	p_max = g_number_of_passengers;
	
	fprintf(g_pFileGAMSinputSetLog, "OPTIONS  MIP = CPLEX;\n");
	fprintf(g_pFileGAMSinputSetLog, "Set i /1*%d/;\n", i_max);
	fprintf(g_pFileGAMSinputSetLog, "Set t /1*%d/;\n", t_max);
	fprintf(g_pFileGAMSinputSetLog, "Set v /1*%d/;\n", v_max);
	fprintf(g_pFileGAMSinputSetLog, "Set k /0*%d/;\n", k_max);
	fprintf(g_pFileGAMSinputSetLog, "Set p /1*%d/;\n", p_max);
	fprintf(g_pFileGAMSinputSetLog, "alias(i, j);\n");
	fprintf(g_pFileGAMSinputSetLog, "alias(t, s);\n");

	fprintf(g_pFileGAMSinputSetCsv, "%s,%s,%s,%s,%s\n", "set_i", "set_t", "set_v", "set_k", "set_p");
	fprintf(g_pFileGAMSinputSetCsv, "%d,%d,%d,%d,%d", i_max, t_max, v_max, k_max, p_max);

}

void g_Output_GAMS_Parameter_needs()
{
	//current iteration number
	int current_iteration_num = 1;
	/*fprintf(g_pFileGAMSinputParameterLog, "parameter iteration_num/\n");
	fprintf(g_pFileGAMSinputParameterLog, "%d\n", current_iteration_num);
	fprintf(g_pFileGAMSinputParameterLog, "/;\n");*/

	// path cost of each vehicle
	fprintf(g_pFileGAMSinputParameterLog, "parameter c(v,k)/\n");
	fprintf(g_pFileGAMSinputParaVehPathCostCsv, "parameter c(v_k)\n");
	//int k = 1;


	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		fprintf(g_pFileGAMSinputParameterLog, "%d.%d %.2f\n", j, 0, 0);
		fprintf(g_pFileGAMSinputParaVehPathCostCsv, "%d.%d %.2f\n", j, 0, 0);
		fprintf(g_pFileGAMSinputParameterLog, "%d.%d %.2f\n", j, current_iteration_num, g_vehicle_path_cost[j]);
		fprintf(g_pFileGAMSinputParaVehPathCostCsv, "%d.%d %.2f\n", j, current_iteration_num, g_vehicle_path_cost[j]);
	}
	fprintf(g_pFileGAMSinputParameterLog, "/;\n");

	// path flow: agent-based 0 or 1
	fprintf(g_pFileGAMSinputParameterLog, "parameter x(v,k)/\n");
	fprintf(g_pFileGAMSinputParaVehPathSelectionCsv, "parameter x(v_k)\n");

	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		fprintf(g_pFileGAMSinputParameterLog, "%d.%d %d\n", j, current_iteration_num, 1);
		fprintf(g_pFileGAMSinputParaVehPathSelectionCsv, "%d.%d %d\n", j, current_iteration_num, 1);
	}
	fprintf(g_pFileGAMSinputParameterLog, "/;\n");

	// vehicle-passenger incidence
	fprintf(g_pFileGAMSinputParameterLog, "parameter delta(v,k,p)/\n");	
	fprintf(g_pFileGAMSinputParaIncidVehPathPaxCsv, "parameter delta(v_k_p)\n");

	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		vector <int> served_passenger_id;
		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size(); i++)
		{
			int passenger_id = g_node_passenger_id[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]];
			if (g_ending_state_vector[j].m_VSStateVector[0].passenger_service_state[passenger_id] != 2)
				continue;
			served_passenger_id.push_back(g_node_passenger_id[g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i]]);
		}

		sort(served_passenger_id.begin(), served_passenger_id.end());
		served_passenger_id.erase(unique(served_passenger_id.begin(), served_passenger_id.end()), served_passenger_id.end());
		for (int i = 0; i < served_passenger_id.size(); i++)
		{
			fprintf(g_pFileGAMSinputParameterLog, "%d.%d.%d %d\n", j, current_iteration_num, served_passenger_id[i],1);
			fprintf(g_pFileGAMSinputParaIncidVehPathPaxCsv, "%d.%d.%d %d\n", j, current_iteration_num, served_passenger_id[i], 1);
		}	
	}
	fprintf(g_pFileGAMSinputParameterLog, "/;\n");

	// vehicle-space-time arc incidence
	fprintf(g_pFileGAMSinputParameterLog, "parameter beta(v,k,i,j,t,s)/\n");
	fprintf(g_pFileGAMSinputParaIncidVehPathArcCsv, "parameter beta(v_k_i_j_t_s)\n");

	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		if (g_ending_state_vector[j].m_VSStateVector.size() == 0)
			continue;

		int arc_origin, arc_origin_next, arc_dep, arc_dep_next;
		float arc_capacity;
		arc_origin = g_vehicle_origin_node[j];
		arc_origin_next = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[0];

		arc_dep = g_vehicle_departure_time_beginning[j];
		arc_dep_next = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[0];

		fprintf(g_pFileGAMSinputParameterLog, "%d.%d.%d.%d.%d.%d %d\n", j, current_iteration_num, arc_origin, arc_origin_next, arc_dep, arc_dep_next, 1);
		fprintf(g_pFileGAMSinputParaIncidVehPathArcCsv, "%d.%d.%d.%d.%d.%d %d\n", j, current_iteration_num, arc_origin, arc_origin_next, arc_dep, arc_dep_next, 1);

		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size() - 1; i++)
		{
			int arc_i, arc_j, arc_t, arc_s;

			arc_i = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i];
			arc_j = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i + 1];

			arc_t = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i];
			arc_s = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i + 1];

			fprintf(g_pFileGAMSinputParameterLog, "%d.%d.%d.%d.%d.%d %d\n", j, current_iteration_num, arc_i, arc_j, arc_t, arc_s, 1);
			fprintf(g_pFileGAMSinputParaIncidVehPathArcCsv, "%d.%d.%d.%d.%d.%d %d\n", j, current_iteration_num, arc_i, arc_j, arc_t, arc_s, 1);
		}
	}
	fprintf(g_pFileGAMSinputParameterLog, "/;\n");

	// network-arc capacity
	fprintf(g_pFileGAMSinputParameterLog, "parameter cap(i,j,t,s)/\n");
	fprintf(g_pFileGAMSinputParaArcCapCsv, "parameter cap(i_j_t_s)\n");

	vector <string> generatedArcCapacity;
	//stringstream s[150][20];
	string arc_gams[200][100];
	for (int j = 1; j <= g_number_of_vehicles; j++)
	{
		if (g_ending_state_vector[j].m_VSStateVector.size() == 0)
			continue;
	
		int arc_origin, arc_origin_next, arc_dep, arc_dep_next;
		float arc_capacity;
		
		arc_origin = g_vehicle_origin_node[j];
		arc_origin_next = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[0];

		arc_dep = g_vehicle_departure_time_beginning[j];
		arc_dep_next = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[0];

		int link_no_new = g_ending_state_vector[j].m_VSStateVector[0].m_visit_link_sequence[0];
		if (link_no_new == -1)
		{
			arc_capacity = 99999.00;
		}
		else
		{
			arc_capacity = g_link_capacity_per_time_interval[link_no_new] ;
		}
		
		//int m;
		//m = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size()*j;
		//s[j][0] << arc_origin <<"."<< arc_origin_next << "." << arc_dep<< "." << arc_dep_next<< " " << arc_capacity;
		arc_gams[j][0] = to_string(arc_origin) + "." + to_string(arc_origin_next) + "." + to_string(arc_dep) + "." + to_string(arc_dep_next) + " " + to_string(arc_capacity);
		//generatedArcCapacity.push_back(s[j][0].str());
		generatedArcCapacity.push_back(arc_gams[j][0]);

		for (int i = 0; i < g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size() - 1; i++)
		{
			int arc_i, arc_j, arc_t, arc_s;

			arc_i = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i];
			arc_j = g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence[i + 1];

			arc_t = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i];
			arc_s = g_ending_state_vector[j].m_VSStateVector[0].m_visit_time_sequence[i + 1];

			int link_no_new = g_ending_state_vector[j].m_VSStateVector[0].m_visit_link_sequence[i + 1];
			if (link_no_new == -1)
			{
				arc_capacity = 99999.00;
			}
			else
			{
				arc_capacity = g_link_capacity_per_time_interval[link_no_new];
			}

			int n;
			n = i + 1 + j*g_ending_state_vector[j].m_VSStateVector[0].m_visit_node_sequence.size();

			//s[j][i+1] << arc_i << "." << arc_j << "." << arc_t << "." << arc_s << " " << arc_capacity;
			arc_gams[j][i + 1]= to_string(arc_i) + "." + to_string(arc_j) + "." + to_string(arc_t) + "." + to_string(arc_s) + " " + to_string(arc_capacity);
			//generatedArcCapacity.push_back(s[j][i + 1].str());
			generatedArcCapacity.push_back(arc_gams[j][i + 1]);
		}
	}

	sort(generatedArcCapacity.begin(), generatedArcCapacity.end());
	generatedArcCapacity.erase(unique(generatedArcCapacity.begin(), generatedArcCapacity.end()), generatedArcCapacity.end());
	for (int i = 0; i < generatedArcCapacity.size(); i++)
	{
		std::string str = generatedArcCapacity[i];
		char *cstr = &str[0u];
		fprintf(g_pFileGAMSinputParameterLog, "%s\n", cstr);
		fprintf(g_pFileGAMSinputParaArcCapCsv, "%s\n", cstr);
	}
	fprintf(g_pFileGAMSinputParameterLog, "/;\n");
}

void g_ReadGamsMargData()
{
	// step 1: read arc_capacity_margianl_cost file 
	CCSVParser parser;
	if (parser.OpenCSVFile("GAMS_arc_cap_marg.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			int internal_from_node_id;
			int internal_to_node_id;

			int from_node_time;

			float marginal_value;

			if (parser.GetValueByFieldName("from_node", internal_from_node_id) == false)
				continue;

			if (internal_from_node_id <= 0)
			{
				cout << "from_node " << internal_from_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (parser.GetValueByFieldName("to_node", internal_to_node_id) == false)
				continue;

			if (internal_to_node_id <= 0)
			{
				cout << "to_node " << internal_to_node_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (parser.GetValueByFieldName("from_time", from_node_time) == false)
				continue;

			if (from_node_time <= 0)
			{
				cout << "from_time " << from_node_time << " is out of range" << endl;
				g_ProgramStop();
			}


			if (parser.GetValueByFieldName("marginal_value", marginal_value) == false)
				continue;

			if (marginal_value < 0)
			{
				cout << "marginal_value " << marginal_value << " is out of range" << endl;
				g_ProgramStop();
			}

			int link_internal_No = g_link_no[internal_from_node_id][internal_to_node_id];
			if (link_internal_No == 0)
			{
				continue;
			}
			else
			{
				g_link_time_dependent_margianl_cost[link_internal_No][from_node_time] = marginal_value;
			}
			
		}
		parser.CloseCSVFile();
	}

	// step 2: read passenger_margianl_cost file 
	if (parser.OpenCSVFile("GAMS_passenger_pickup_marg.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int internal_passenger_id;
			float passenger_pickup_marginal_cost;

			if (parser.GetValueByFieldName("passenger", internal_passenger_id) == false)
				continue;

			if (internal_passenger_id <= 0)
			{
				cout << "passenger " << internal_passenger_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (parser.GetValueByFieldName("pickup marginal_value", passenger_pickup_marginal_cost) == false)
				continue;

			if (passenger_pickup_marginal_cost < 0)
			{
				cout << "pickup marginal_value " << passenger_pickup_marginal_cost << " is out of range" << endl;
				g_ProgramStop();
			}
			g_passenger_marginal_cost[internal_passenger_id] = passenger_pickup_marginal_cost;
		}
		parser.CloseCSVFile();
	}

	// step 3: read vehicle_margianl_cost file 
	if (parser.OpenCSVFile("GAMS_vehicle_weight_marg.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int internal_vehicle_id;
			float vehicle_marginal_cost;

			if (parser.GetValueByFieldName("vehicle", internal_vehicle_id) == false)
				continue;

			if (internal_vehicle_id <= 0)
			{
				cout << "vehicle " << internal_vehicle_id << " is out of range" << endl;
				g_ProgramStop();
			}

			if (parser.GetValueByFieldName("vehicle_weight marginal_value", vehicle_marginal_cost) == false)
				continue;

			if (vehicle_marginal_cost < 0)
			{
				cout << "vehicle_weight marginal_value " << vehicle_marginal_cost << " is out of range" << endl;
				g_ProgramStop();
			}
			g_vehicle_marginal_cost[internal_vehicle_id] = vehicle_marginal_cost;
		}
		parser.CloseCSVFile();
	}
}


int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	g_time_dependent_state_vector = Allocate3DDynamicArray<C_time_indexed_state_vector>(_MAX_NUMBER_OF_VEHICLES, _MAX_NUMBER_OF_NODES, _MAX_NUMBER_OF_TIME_INTERVALS);

	//output file for Lagrandian and DP updating process
	g_pFileDebugLog = fopen("Debug.txt", "w");
	if (g_pFileDebugLog == NULL)
	{
		cout << "File Debug.txt cannot be opened." << endl;
		g_ProgramStop();
	}

	//output file for GAMS input set
	g_pFileGAMSinputSetLog = fopen("GAMS_input_set.txt", "w");
	if (g_pFileGAMSinputSetLog == NULL)
	{
		cout << "File GAMS_Input.txt cannot be opened." << endl;
		g_ProgramStop();
	}

	//output csv file for GAMS input set
	g_pFileGAMSinputSetCsv = fopen("Internal_GAMS_input_set.csv", "w");
	if (g_pFileGAMSinputSetCsv == NULL)
	{
		cout << "File Internal_GAMS_input_set.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//output csv file for GAMS input parameter: VehPathCost
	g_pFileGAMSinputParaVehPathCostCsv = fopen("Internal_GAMS_input_veh_path_cost.csv", "w");
	if (g_pFileGAMSinputParaVehPathCostCsv == NULL)
	{
		cout << "File Internal_GAMS_input_veh_path_cost.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//output csv file for GAMS input parameter:VehPathSelection
	g_pFileGAMSinputParaVehPathSelectionCsv = fopen("Internal_GAMS_input_veh_path_selection.csv", "w");
	if (g_pFileGAMSinputParaVehPathSelectionCsv == NULL)
	{
		cout << "File Internal_GAMS_input_veh_path_selection.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//output csv file for GAMS input parameter: IncidVehPathPax
	g_pFileGAMSinputParaIncidVehPathPaxCsv = fopen("Internal_GAMS_input_incid_veh_path_pax.csv", "w");
	if (g_pFileGAMSinputParaIncidVehPathPaxCsv == NULL)
	{
		cout << "File Internal_GAMS_input_incid_veh_path_pax.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//output csv file for GAMS input parameter: IncidVehPathArc
	g_pFileGAMSinputParaIncidVehPathArcCsv = fopen("Internal_GAMS_input_incid_veh_path_arc.csv", "w");
	if (g_pFileGAMSinputParaIncidVehPathArcCsv == NULL)
	{
		cout << "File Internal_GAMS_input_incid_veh_path_arc.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//output csv file for GAMS input parameter: ArcCap
	g_pFileGAMSinputParaArcCapCsv = fopen("Internal_GAMS_input_arc_cap.csv", "w");
	if (g_pFileGAMSinputParaArcCapCsv == NULL)
	{
		cout << "File Internal_GAMS_input_arc_cap.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	//output file for GAMS input parameters
	g_pFileGAMSinputParameterLog = fopen("GAMS_input_parameter.txt", "w");
	if (g_pFileGAMSinputParameterLog == NULL)
	{
		cout << "File GAMS_Input.txt cannot be opened." << endl;
		g_ProgramStop();
	}

	//output file for vehicle's path_node_seq and path_time_seq
	g_pFile_OutputAgentLog = fopen("output_agent.csv", "w");
	if (g_pFile_OutputAgentLog == NULL)
	{
		cout << "File output_agent_initial.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputAgentLog, "Internal_veh_ID,Vehicle ID,path_No, path_node_seq,path_time_sequence, path_node_state_sequence,path_link_sequence, path_cost, served passengers");

	//output file for vehicle's path_node_seq and path_time_seq
	g_pFile_OutputAgentPathArcLog = fopen("output_agent_path_arc_incidence.csv", "w");
	if (g_pFile_OutputAgentPathArcLog == NULL)
	{
		cout << "File output_agent_path_arc_incidence.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputAgentPathArcLog, "agent_id,path_no,path_cost, arc(i.j.t.s), path_arc_incidence, arc_cap(i.j.t.s)");

	//output file for vehicle state update during DP process
	g_pFile_PathLog = fopen("PathLog.csv", "w");
	if (g_pFile_PathLog == NULL)
	{
		cout << "File PassengerProfit.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_PathLog, "vehicle No.,Time,state");

	//output overview of your program data
	g_pFileOutputLog = fopen("output_solution.csv", "w");
	if (g_pFileOutputLog == NULL)
	{
		cout << "File output_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	g_ReadInputData();

	// definte timestamps
	clock_t start_t, end_t, total_t;
	start_t = clock();

	//Lagrangian function---the major function 
	//g_Optimization_Lagrangian_Method_Vehicle_Routing_Problem_Simple_Variables();
	g_upper_bound_generation();
	g_output_optimization_result();

	//Output for GAMS
	g_Output_GAMS_Set_needs();
	g_Output_GAMS_Parameter_needs();
	//g_ReadGamsMargData();

	end_t = clock();
	total_t = (end_t - start_t);

	cout << "CPU Running Time = " << total_t << " milliseconds" << endl;

	fprintf(g_pFileDebugLog, "CPU Running Time = %ld milliseconds\n", total_t);
	fprintf(g_pFileOutputLog, "CPU Running Time =,%ld, milliseconds\n", total_t);

	//output_node.csv with virtual node 
	g_pFile_OutputNodeLog = fopen("output_node.csv", "w");
	if (g_pFile_OutputNodeLog == NULL)
	{
		cout << "File output_node.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	fprintf(g_pFile_OutputNodeLog, "node_id,node_type\n");
	for (int i = 1; i <= g_number_of_nodes; i++)
	{
		fprintf(g_pFile_OutputNodeLog, "%d,%d\n", i, g_node_type[i]);
	}

	//output_link.csv with virtual link
	g_pFile_OutputLinkLog = fopen("output_link.csv", "w");
	if (g_pFile_OutputLinkLog == NULL)
	{
		cout << "File output_link.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputLinkLog, "from_node_id,to_node_id,link_type\n");
	for (int i = 0; i < g_number_of_links; i++)
	{
		fprintf(g_pFile_OutputLinkLog, "%d,%d,%d\n", g_link_from_node_number[i], g_link_to_node_number[i], g_link_service_code[i]);
	}//if link is virtual link, passenger:origin pax_no destination -pax_no ; vehicle:origin 100 destination 101

	fclose(g_pFile_OutputNodeLog);
	fclose(g_pFile_OutputLinkLog);
	fclose(g_pFile_OutputAgentLog);
	fclose(g_pFile_OutputAgentPathArcLog);
	fclose(g_pFileOutputLog);
	fclose(g_pFileDebugLog);	
	fclose(g_pFile_PathLog);

	fclose(g_pFileGAMSinputSetLog);
	fclose(g_pFileGAMSinputSetCsv);
	fclose(g_pFileGAMSinputParaVehPathCostCsv);
	fclose(g_pFileGAMSinputParaVehPathSelectionCsv);
	fclose(g_pFileGAMSinputParaIncidVehPathPaxCsv);
	fclose(g_pFileGAMSinputParaIncidVehPathArcCsv);
	fclose(g_pFileGAMSinputParaArcCapCsv);

	fclose(g_pFileGAMSinputParameterLog);

	Deallocate3DDynamicArray<C_time_indexed_state_vector>(g_time_dependent_state_vector, _MAX_NUMBER_OF_VEHICLES, _MAX_NUMBER_OF_NODES);

	cout << "End of Optimization " << endl;
	cout << "free memory.." << endl;
	//getchar();
	return 1;
}