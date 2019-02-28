// AgentbasedADMM.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "CSVParser.h"
#include <string> 
#include <algorithm>
#include <list>  

using namespace std;

#define _MAX_NUMBER_OF_NODES 2000
#define _MAX_NUMBER_OF_LINKS 7000
#define _MAX_LABEL_COST 99999

#define _MAX_NUMBER_OF_PHYSICAL_NODES 400000
#define _MAX_STATES 5

int g_number_of_links;
int g_number_of_nodes;
int g_number_of_agents;
int g_number_of_optimization_time_intervals = 60;

int enum_road_capacity_link_type = 0;
int g_initial_state_no = 0;   // customized
int g_number_of_LR_iterations = 2;

int g_CurrentLRIterationNumber = 0;
float g_best_lower_bound = -99999;
float g_stepSize = 0;//ADMM uses rho to update the LR multiplier
float g_penalty_RHO = 5;
float g_minimum_subgradient_step_size = 0.1;

std::map<int, int> g_internal_node_seq_no_map;  // hush table, map external node number to internal node sequence no. 
std::map<int, int> g_internal_node_seq_no_to_node_id_map;  // hush table, map internal node number to external node sequence no. 
std::map<int, int> g_link_key_to_seq_no_map;  // hush table, map key to internal link sequence no. 


class CLink
{
public:
	CLink()  // construction 
	{
		external_link_id = 0;
		service_type = 0;
		service_price = 0;
		VRP_load_id = -1;
		base_price = 1;
		VRP_load_difference = 0;

		link_capacity = 1800/60;
		free_flow_travel_time_in_min = 1;
		flow_volume = 0;
		number_of_lanes = 1;
		speed_limit = 60;

		VRP_time_window_begin = -1;
		VRP_time_window_end = 10000;
	}

	std::list<int>  m_waiting_traveler_queue;

	int VRP_time_window_begin, VRP_time_window_end;
	float base_price, speed_limit;
	std::vector<int> travel_time_vector;
	std::vector<float> time_dependent_LR_multiplier_vector, time_dependent_ADMM_multiplier_vector;

	std::vector<int> time_dependent_visit_counts, time_dependent_ADMM_visit_counts, time_depedent_capacity_vector;

	int max_allowed_waiting_time;

	void Setup_State_Dependent_Data_Matrix(int number_of_optimization_time_intervals)
	{
		for (int t = 0; t < number_of_optimization_time_intervals; t++)
		{
			time_dependent_visit_counts.push_back(0);
			time_dependent_ADMM_visit_counts.push_back(0);
			time_dependent_LR_multiplier_vector.push_back(0);
			time_depedent_capacity_vector.push_back(link_capacity);


			time_dependent_ADMM_multiplier_vector.push_back(0);
			travel_time_vector.push_back((int)(free_flow_travel_time_in_min));  //assume simulation time interval as free-flow travel time per cell 
		}

		VRP_time_window_begin = max(0, VRP_time_window_begin);
		VRP_time_window_end = min(number_of_optimization_time_intervals - 1, VRP_time_window_end);
	}

	int external_link_id;
	int link_seq_no;  // internal seq no
	int from_node_seq_no;
	int to_node_seq_no;
	float cost;
	float free_flow_travel_time_in_min;

	int number_of_lanes;
	int type;
	int service_type; // 0: moving; 1, pick up

	float service_price; // for pick up or drop off
	int VRP_load_id;
	int VRP_group_id;
	int VRP_load_difference; // we use a single point time window now

	float link_capacity;
	float flow_volume;
	float travel_time;
	float length;
};

class CNode
{
public:
	CNode()
	{
		zone_id = 0;
	}

	int node_seq_no;  // sequence number 
	int external_node_id;      //external node number 
	int zone_id;
	double x;
	double y;

	std::vector<CLink> m_outgoing_node_vector;

};

class CAgent
{
public:

	CAgent()
	{
		agent_vector_seq_no = -1;
		agent_service_type = 0;  //0: pax vehicle 1: travler 2: scheduled transportation vehicle

		vehicle_seat_capacity = 1;

		path_cost = 0;
		departure_time_in_min = 0;

		earliest_departure_time = 0;
		departure_time_window = 1;
		latest_arrival_time = 0;
		arrival_time_window = 1;
	}

	int agent_id;
	int agent_vector_seq_no;
	int agent_service_type;

	int origin_node_id;
	int destination_node_id;

	float departure_time_in_min;

	float path_cost;
	std::vector<int> path_link_seq_no_vector;
	int m_path_link_seq_no_vector_size;

	std::vector<int> path_node_id_vector;

	int vehicle_seat_capacity;
	int VRP_group_id;

	float earliest_departure_time;
	int departure_time_in_simulation_interval;
	float departure_time_window;

	float latest_arrival_time;
	float arrival_time_window;
	std::vector<int> path_timestamp_vector;

	std::map<int, int> m_VRP_ADMM_link_time_map;

};

std::vector<CNode> g_node_vector;
std::vector<CLink> g_link_vector;
std::vector<CAgent> g_agent_vector;

void g_ProgramStop()
{
	cout << "Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};

long g_GetLinkSeqNo(int from_node_id, int to_node_id)
{
	if (g_internal_node_seq_no_map.find(from_node_id) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	if (g_internal_node_seq_no_map.find(from_node_id) == g_internal_node_seq_no_map.end())
	{
		return -1; //have not been defined
	}

	int from_node_seq_no = g_internal_node_seq_no_map[from_node_id];
	int to_node_seq_no = g_internal_node_seq_no_map[to_node_id];

	long link_key = from_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + to_node_seq_no;

	if (g_link_key_to_seq_no_map.find(link_key) != g_link_key_to_seq_no_map.end())
		return g_link_key_to_seq_no_map[link_key];
	else
		return -1;
}

void g_ReadInputData()
{
	g_number_of_nodes = 0;
	g_number_of_links = 0;  // initialize  the counter to 0

	int internal_node_seq_no = 0;
	double x, y;

	// step 1: read node file 
	CCSVParser parser;
	if (parser.OpenCSVFile("input_node.csv", true))
	{

		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			string name;

			int node_type;
			int node_id;

			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;

			if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
			{
				continue; //has been defined
			}

			// map between external_node_id and internal_node_id
			g_internal_node_seq_no_map[node_id] = internal_node_seq_no;
			g_internal_node_seq_no_to_node_id_map[internal_node_seq_no] = node_id;

			parser.GetValueByFieldName("x", x, false);
			parser.GetValueByFieldName("y", y, false);

			CNode node;  // create a node object 

			node.external_node_id = node_id;
			node.node_seq_no = internal_node_seq_no;

			node.x = x;
			node.y = y;
			internal_node_seq_no++;

			g_node_vector.push_back(node);  // push it to the global node vector

			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 == 0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}

		cout << "number of nodes = " << g_number_of_nodes << endl;

		parser.CloseCSVFile();
	}
	else
	{
		cout << "input_node.csv is not opened." << endl;
		g_ProgramStop();
	}

	// step 2: read link file 

	CCSVParser parser_link;

	if (parser_link.OpenCSVFile("input_link.csv", true))
	{
		while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			if (parser_link.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser_link.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;

			// convert to be the internal_node_id
			int internal_from_node_seq_no = g_internal_node_seq_no_map[from_node_id];
			int internal_to_node_seq_no = g_internal_node_seq_no_map[to_node_id];

			CLink link;  // create a link object 

			parser_link.GetValueByFieldName("link_id", link.external_link_id);

			link.from_node_seq_no = internal_from_node_seq_no;
			link.to_node_seq_no = internal_to_node_seq_no;
			link.link_seq_no = g_number_of_links;

			parser_link.GetValueByFieldName("link_type", link.type);
			parser_link.GetValueByFieldName("service_type", link.service_type, false);

			// pickup link
			if (link.service_type != 0)
			{
				parser_link.GetValueByFieldName("VRP_load_id", link.VRP_load_id, false);
				parser_link.GetValueByFieldName("VRP_group_id", link.VRP_group_id, false);
				parser_link.GetValueByFieldName("VRP_time_window_begin", link.VRP_time_window_begin, false);
				parser_link.GetValueByFieldName("VRP_time_window_end", link.VRP_time_window_end, false);
				parser_link.GetValueByFieldName("VRP_load_difference", link.VRP_load_difference, false);
			}

			float length = 1; // km or mile
			float speed_limit = 1;
			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("speed_limit", speed_limit);
			parser_link.GetValueByFieldName("base_price", link.base_price);

			int number_of_lanes = 1;
			float lane_cap = 1000;
			parser_link.GetValueByFieldName("number_of_lanes", link.number_of_lanes);
			parser_link.GetValueByFieldName("lane_cap", lane_cap);

			link.link_capacity = lane_cap* number_of_lanes/60;

			link.free_flow_travel_time_in_min = length / speed_limit * 60;

			float external_travel_time = -1;
			parser_link.GetValueByFieldName("external_travel_time", external_travel_time, false);

			if (external_travel_time >= 0.1)
			{  // reset 
				link.free_flow_travel_time_in_min = external_travel_time;
			}

			link.length = length;
			link.cost = length / speed_limit * 60; // min

			// add this link to the corresponding node as part of outgoing link
			g_node_vector[internal_from_node_seq_no].m_outgoing_node_vector.push_back(link);

			long link_key = internal_from_node_seq_no * _MAX_NUMBER_OF_PHYSICAL_NODES + internal_to_node_seq_no;

			g_link_key_to_seq_no_map[link_key] = link.link_seq_no;
			g_link_vector.push_back(link);

			g_number_of_links++;

			if (g_number_of_links % 1000 == 0)
				cout << "reading " << g_number_of_links << " links.. " << endl;
		}
	}

	else
	{
		cout << "input_link.csv is not opened." << endl;
		g_ProgramStop();
	}

	cout << "number of links = " << g_number_of_links << endl;

	parser_link.CloseCSVFile();

	// read agent file
	g_number_of_agents = 0;
	CCSVParser parser_agent;

	if (parser_agent.OpenCSVFile("input_agent.csv", true))   // read agent as demand input 
	{
		while (parser_agent.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			CAgent agent;  // create an agent object 
			if (parser_agent.GetValueByFieldName("agent_id", agent.agent_id) == false)
				continue;

			parser_agent.GetValueByFieldName("agent_service_type", agent.agent_service_type);

			int origin_node_id = 0;
			int destination_node_id = 0;
			parser_agent.GetValueByFieldName("from_origin_node_id", origin_node_id, false);

			agent.origin_node_id = origin_node_id;
			parser_agent.GetValueByFieldName("to_destination_node_id", destination_node_id, false);
			agent.destination_node_id = destination_node_id;

			if (g_internal_node_seq_no_map.find(origin_node_id) == g_internal_node_seq_no_map.end() || g_internal_node_seq_no_map.find(destination_node_id) == g_internal_node_seq_no_map.end())
				continue;

			parser_agent.GetValueByFieldName("VRP_group_id", agent.VRP_group_id);
			parser_agent.GetValueByFieldName("vehicle_seat_capacity", agent.vehicle_seat_capacity);
			parser_agent.GetValueByFieldName("departure_time_in_min", agent.departure_time_in_min);

			//current service_type is always 2
			if (agent.agent_service_type == 2)
			{
				parser_agent.GetValueByFieldName("earliest_departure_time", agent.earliest_departure_time);
				parser_agent.GetValueByFieldName("departure_time_window", agent.departure_time_window);
				parser_agent.GetValueByFieldName("latest_arrival_time", agent.latest_arrival_time);
				parser_agent.GetValueByFieldName("arrival_time_window", agent.arrival_time_window);
			}

			if (agent.latest_arrival_time >= g_number_of_optimization_time_intervals)
				g_number_of_optimization_time_intervals = agent.latest_arrival_time + 1;

			g_agent_vector.push_back(agent);
			g_number_of_agents++;
			if (g_number_of_agents % 1000 == 0)
				cout << "reading = " << g_number_of_agents / 1000 << " k agents..." << endl;
		}
	}
	else
	{
		cout << "input_agent.csv is not opened." << endl;
		g_ProgramStop();
	}

	cout << "number of agents = " << g_agent_vector.size() << endl;

	parser_agent.CloseCSVFile();
}

template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}

	for (int x = 0; x < nX; x++)
	{
		if (x % 1000 == 0)
		{
			cout << "allocating 3D memory for " << x << endl;
		}

		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}
		}
	}

	for (int x = 0; x < nX; x++)
		for (int y = 0; y < nY; y++)
			for (int z = 0; z < nZ; z++)
			{
				dynamicArray[x][y][z] = 0;
			}
	return dynamicArray;
}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}
		delete[] dArray[x];
	}
	delete[] dArray;
}

class STSNetwork  // mainly for STS shortest path calculation
{
public:

	std::vector<int>  m_agent_vector; // assigned agents for computing 

	int m_number_of_nodes, m_number_of_time_intervals, m_number_of_states;

	int m_origin_node;
	int m_departure_time_beginning;
	int m_arrival_time_ending;

	float*** m_label_cost;
	float*** m_physical_label_cost;
	int*** m_node_predecessor;
	int*** m_time_predecessor;
	int*** m_state_predecessor;

	int*** m_pax_id_record;  // the memory of visted pax id

	STSNetwork()
	{
		m_origin_node = -1;
		m_label_cost = NULL;
		m_node_predecessor = NULL;
		m_time_predecessor = NULL;
		m_state_predecessor = NULL;
		m_pax_id_record = NULL;
	}

	void AllocateSTSMemory(int number_of_nodes, int number_of_time_intervals, int number_of_states)
	{
		m_number_of_nodes = number_of_nodes;
		m_number_of_time_intervals = number_of_time_intervals;
		m_number_of_states = number_of_states;

		m_label_cost = Allocate3DDynamicArray<float>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_physical_label_cost = Allocate3DDynamicArray<float>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_node_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_time_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_state_predecessor = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
		m_pax_id_record = Allocate3DDynamicArray<int>(m_number_of_nodes, m_number_of_time_intervals, m_number_of_states);
	}

	~STSNetwork()
	{
		Deallocate3DDynamicArray<float>(m_label_cost, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<float>(m_physical_label_cost, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_node_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_time_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_state_predecessor, m_number_of_nodes, m_number_of_time_intervals);
		Deallocate3DDynamicArray<int>(m_pax_id_record, m_number_of_nodes, m_number_of_time_intervals);
	}

	void ADMM_penalty_method(int this_agent_id)
	{
		// step 1: reset
		for (int link_no = 0; link_no < g_link_vector.size(); link_no++)
		{
			for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
			{
				g_link_vector[link_no].time_dependent_ADMM_visit_counts[t] = 0;
			}
		}

		// step 2: scan all the other agents
		for (int j = 0; j < g_agent_vector.size(); j++)
		{
			CAgent* p_agent_other = &(g_agent_vector[j]);
			if (p_agent_other->agent_id != this_agent_id)  // not the current agent. 
			{
				// loop through the map
				for (std::map<int, int>::iterator it = p_agent_other->m_VRP_ADMM_link_time_map.begin();
					it != p_agent_other->m_VRP_ADMM_link_time_map.end(); ++it)
				{
					int this_link_no = it->first;
					int this_time = it->second;
					g_link_vector[this_link_no].time_dependent_ADMM_visit_counts[this_time] += 1;
				}
			}
		}

		// step 3: calculate ADMM capacity penalty 
		for (int link_no = 0; link_no < g_link_vector.size(); link_no++)
		{
			for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
			{
				if (g_link_vector[link_no].service_type == enum_road_capacity_link_type)
				{
					int time_dependent_visit_counts_used_by_other_vehicles = g_link_vector[link_no].time_dependent_ADMM_visit_counts[t];

					g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t] = max(0,
						(g_penalty_RHO / 2.0)*(2 * time_dependent_visit_counts_used_by_other_vehicles - g_link_vector[link_no].time_depedent_capacity_vector[t]));
				}
			}  // end for each t
		} //end for each link

		  // step 4: calculate ADMM demand penalty 
		for (int link_no = 0; link_no < g_link_vector.size(); link_no++)
		{
			if (g_link_vector[link_no].service_type == 1)   // pick up
			{  // visited once, then no one is allowed to visit this link anymore
				int time_dependent_visit_counts_used_by_other_vehicles = 0;

				for (int t = g_link_vector[link_no].VRP_time_window_begin; t <= g_link_vector[link_no].VRP_time_window_end; t++)
				{
					time_dependent_visit_counts_used_by_other_vehicles += g_link_vector[link_no].time_dependent_ADMM_visit_counts[t];
				}

				// across all the timestamps in the time window
				for (int t = g_link_vector[link_no].VRP_time_window_begin; t <= g_link_vector[link_no].VRP_time_window_end; t++)
				{

					g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t] =
						(g_penalty_RHO / 2.0)*(2 * time_dependent_visit_counts_used_by_other_vehicles - 1);

				}  // end for each t
			}  // if condition for demand link
		} //end for each link
	}

	//parallel computing version
	float optimal_STS_dynamic_programming(int VRP_group_id, int departure_time_beginning, int arrival_time_ending)
	{
		if (m_origin_node < 0)
			return -1;

		std::map<int, int> node_scan_eligible_list_flag;
		float total_cost = _MAX_LABEL_COST;

		if (g_node_vector[m_origin_node].m_outgoing_node_vector.size() == 0)
		{
			return _MAX_LABEL_COST;
		}

		if (arrival_time_ending > m_number_of_time_intervals - 1)
		{
			return _MAX_LABEL_COST;
		}

		CAgent* p_agent = &(g_agent_vector[m_agent_vector[0]]);

		// step 1: Initialization for all nodes
		for (int i = 0; i < m_number_of_nodes; i++)
		{
			// to do: only update node label on the agent path
			for (int t = departure_time_beginning; t <= arrival_time_ending; t++)
			{
				for (int w = 0; w <= p_agent->vehicle_seat_capacity; w++)
				{
					m_label_cost[i][t][w] = _MAX_LABEL_COST;
					m_physical_label_cost[i][t][w] = _MAX_LABEL_COST;
					m_node_predecessor[i][t][w] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
					m_time_predecessor[i][t][w] = -1;  // pointer to previous TIME INDEX from the current label at current node and time
					m_state_predecessor[i][t][w] = -1;
					m_pax_id_record[i][t][w] = -1;
				}
			}
		}

		//step 2: Initialization for origin node at the preferred departure time, at departure time
		int w0 = g_initial_state_no;  // start from empty

		for (int t = departure_time_beginning; t < min(arrival_time_ending, g_number_of_optimization_time_intervals - 1); t++)  //first loop: time
		{
			m_label_cost[m_origin_node][t][w0] = t - departure_time_beginning;  // waiting at origin
			m_physical_label_cost[m_origin_node][t][w0] = t - departure_time_beginning;
		}
		node_scan_eligible_list_flag[m_origin_node] = 1;

		// step 3: //dynamic programming , absoluate time

		for (int t = departure_time_beginning; t <= arrival_time_ending; t++)  //first loop: time
		{
			for (std::map<int, int>::iterator it = node_scan_eligible_list_flag.begin(); it != node_scan_eligible_list_flag.end(); ++it)
			{
				int n = it->first;

				for (int link = 0; link < g_node_vector[n].m_outgoing_node_vector.size(); link++)
				{
					int link_no = g_node_vector[n].m_outgoing_node_vector[link].link_seq_no;
					int from_node = g_node_vector[n].m_outgoing_node_vector[link].from_node_seq_no;
					int to_node = g_node_vector[n].m_outgoing_node_vector[link].to_node_seq_no;

					if (g_link_vector[link_no].service_type != 0)   // pick up or drop off
					{ // reset dynamically the waiting time
						if (t > g_link_vector[link_no].VRP_time_window_end)
							continue;

						if (g_link_vector[link_no].VRP_group_id != VRP_group_id)
							continue;
					}

					for (int w1 = 0; w1 <= p_agent->vehicle_seat_capacity; w1++) // for each state
					{
						if (m_label_cost[from_node][t][w1] < _MAX_LABEL_COST - 1)  //for feasible time-space point only
						{
							int travel_time = 0;
							//										travel_time = p_agent->time_dependent_link_travel_time[link_no][t] + wait_time;
							travel_time = g_link_vector[link_no].free_flow_travel_time_in_min;  // to do: consider time-dependent travel time:L XS.
							int travel_cost = 0;

							int from_load_state = w1;

							int to_load_state = max(0, from_load_state + g_link_vector[link_no].VRP_load_difference);

							if (to_load_state > p_agent->vehicle_seat_capacity)// this is the vehicle capacity constraint
								continue;

							int w2 = to_load_state;

							int new_to_node_arrival_time = min(t + travel_time, g_number_of_optimization_time_intervals - 1); // plus the waiting time on the link

							if (new_to_node_arrival_time == t) // XUESONG: we do not allow update at the same time interval
								continue;

							if (g_link_vector[link_no].service_type != 0 && new_to_node_arrival_time < g_link_vector[link_no].VRP_time_window_begin)
							{
								new_to_node_arrival_time = g_link_vector[link_no].VRP_time_window_begin; // reset the arrival time to the service node, 
							}

							int passenger_id = g_link_vector[link_no].VRP_load_id;

							float temporary_label_cost = 0;
							float temporary_physical_label_cost = 0;

							float sum_of_multipliers = 0;

							sum_of_multipliers += g_link_vector[link_no].time_dependent_LR_multiplier_vector[t]
								+ g_link_vector[link_no].time_dependent_ADMM_multiplier_vector[t];

							temporary_label_cost = m_label_cost[from_node][t][w1]
								+ travel_time + sum_of_multipliers;

							temporary_physical_label_cost = m_physical_label_cost[from_node][t][w1] + travel_time;

							if (temporary_label_cost < m_label_cost[to_node][new_to_node_arrival_time][w2]) // we only compare cost at the downstream node ToID at the new arrival time t
							{
								// update cost label and node/time predecessor
								m_label_cost[to_node][new_to_node_arrival_time][w2] = temporary_label_cost;
								m_physical_label_cost[to_node][new_to_node_arrival_time][w2] = temporary_physical_label_cost;
								m_node_predecessor[to_node][new_to_node_arrival_time][w2] = from_node;  // pointer to previous NODE INDEX from the current label at current node and time
								m_time_predecessor[to_node][new_to_node_arrival_time][w2] = t;  // pointer to previous TIME INDEX from the current label at current node and time
								m_state_predecessor[to_node][new_to_node_arrival_time][w2] = w1;

								// copy the visited pax id from 0 to w1
								for (int w1index = 0; w1index <= w1; w1index++)
								{
									m_pax_id_record[to_node][new_to_node_arrival_time][w1index] = m_pax_id_record[from_node][t][w1index];
								}
								m_pax_id_record[to_node][new_to_node_arrival_time][w2] = passenger_id;  // take the record of pax id at position w2

								node_scan_eligible_list_flag[to_node] = 1;
							}
						}  // for all states
					} //for each outgoing link
				}
			}
		} // for all time t
		return total_cost;
	}

	float find_STS_path_for_agents()
	{
		std::vector<int> path_node_sequence, path_link_sequence, path_time_sequence, path_state_sequence;
		std::vector<float> path_cost_sequence;

		float total_cost = _MAX_LABEL_COST;
		float total_physical_cost = _MAX_LABEL_COST;

		// currently only one agent for each STSnetwork
		for (int i = 0; i < m_agent_vector.size(); i++)
		{
			CAgent* p_agent = &(g_agent_vector[m_agent_vector[i]]);

			p_agent->path_link_seq_no_vector.clear();  // reset;
			p_agent->path_timestamp_vector.clear();
			p_agent->path_node_id_vector.clear();  // reset;

			ADMM_penalty_method(p_agent->agent_id);

			int return_value = optimal_STS_dynamic_programming(p_agent->VRP_group_id, m_departure_time_beginning, m_arrival_time_ending);

			if (return_value == -1)
			{
				cout << "agent" << i << " can not find destination node" << endl;;
				continue;
			}

			int destination_node_seq_no = g_internal_node_seq_no_map[p_agent->destination_node_id];

			int min_cost_time_index = m_arrival_time_ending;

			total_cost = m_label_cost[destination_node_seq_no][min_cost_time_index][0];

			int final_state = -1;

			for (int t = m_departure_time_beginning; t < m_arrival_time_ending; t++)
			{
				for (int w = 0; w <= p_agent->vehicle_seat_capacity; w++)// Tony: to find the min_cost label and state, the state is not 0.
					if (m_label_cost[destination_node_seq_no][t][w] < total_cost)
					{
						min_cost_time_index = t;
						final_state = w;
						total_cost = m_label_cost[destination_node_seq_no][t][w];
						total_physical_cost = m_physical_label_cost[destination_node_seq_no][t][w];
					}
			}

			if (final_state == -1)
				return -1;

			//p_agent->path_cost = total_physical_cost;//Tony: physical cost doesn't include the price
			p_agent->path_cost = total_cost;

			// step 2: backtrack to the origin (based on node and time predecessors)
			int	node_size = 0;
			path_node_sequence.push_back(destination_node_seq_no); //record the first node backward, destination node
			path_time_sequence.push_back(min_cost_time_index);
			path_state_sequence.push_back(final_state);
			path_cost_sequence.push_back(m_physical_label_cost[destination_node_seq_no][min_cost_time_index][final_state]);

			node_size++;

			int pred_node = m_node_predecessor[destination_node_seq_no][min_cost_time_index][final_state];
			int pred_time = m_time_predecessor[destination_node_seq_no][min_cost_time_index][final_state];
			int pred_state = m_state_predecessor[destination_node_seq_no][min_cost_time_index][final_state];

			while (pred_node != -1) // scan backward in the predessor array of the shortest path calculation results
			{
				path_node_sequence.push_back(pred_node);
				path_time_sequence.push_back(pred_time);
				path_state_sequence.push_back(pred_state);
				path_cost_sequence.push_back(m_physical_label_cost[pred_node][pred_time][pred_state]);

				node_size++;

				//record current values of node and time predecessors, and update PredNode and PredTime
				int pred_node_record = pred_node;
				int pred_time_record = pred_time;
				int pred_state_record = pred_state;

				pred_node = m_node_predecessor[pred_node_record][pred_time_record][pred_state_record];
				pred_time = m_time_predecessor[pred_node_record][pred_time_record][pred_state_record];
				pred_state = m_state_predecessor[pred_node_record][pred_time_record][pred_state_record];
			}

			//reverse the node sequence 
			std::reverse(std::begin(path_node_sequence), std::end(path_node_sequence));
			std::reverse(std::begin(path_time_sequence), std::end(path_time_sequence));
			std::reverse(std::begin(path_state_sequence), std::end(path_state_sequence));
			std::reverse(std::begin(path_cost_sequence), std::end(path_cost_sequence));

			for (int i = 0; i < path_node_sequence.size(); i++)  // for each node 
			{
				//get the external node id in path
				p_agent->path_node_id_vector.push_back(g_internal_node_seq_no_to_node_id_map[path_node_sequence[i]]);
				p_agent->path_timestamp_vector.push_back(path_time_sequence[i]);
			}

			p_agent->m_VRP_ADMM_link_time_map.clear();

			for (int i = 0; i < path_node_sequence.size(); i++)  // for each node 
			{
				if (i < path_node_sequence.size() - 1)  // for each link
				{
					int link_no = g_GetLinkSeqNo(p_agent->path_node_id_vector[i], p_agent->path_node_id_vector[i + 1]);  // Xuesong Changed
					p_agent->path_link_seq_no_vector.push_back(link_no);
					int entrance_time = p_agent->path_timestamp_vector[i];
					g_link_vector[link_no].time_dependent_visit_counts[entrance_time] += 1;

					p_agent->m_VRP_ADMM_link_time_map[link_no] = entrance_time;
				}
			}
		}
		return total_cost;
	}
};

bool g_Agent_ADMM()
{
	// initilization 
	for (int l = 0; l < g_link_vector.size(); l++)
	{
		g_link_vector[l].Setup_State_Dependent_Data_Matrix(g_number_of_optimization_time_intervals);
	}

	STSNetwork pSTSNetwork;

	pSTSNetwork.AllocateSTSMemory(g_number_of_nodes, g_number_of_optimization_time_intervals, _MAX_STATES);

	//loop for each LR iteration
	for (int LR_iteration = 0; LR_iteration < g_number_of_LR_iterations; LR_iteration++)  // first loop
	{
		cout << LR_iteration + 1 << "/" << g_number_of_LR_iterations << endl;

		g_CurrentLRIterationNumber = LR_iteration + 1;
		g_stepSize = 1.0f / (LR_iteration + 1.0f);

		//keep the minimum step size
		if (g_stepSize < g_minimum_subgradient_step_size)
		{
			g_stepSize = g_minimum_subgradient_step_size;
		}

		// step 1: calaculate LR multipliers
		for (int l = 0; l < g_link_vector.size(); l++)  // for each link
		{
			if (g_link_vector[l].service_type == enum_road_capacity_link_type)  // capacity constraint;
			{
				for (int t = 0; t < g_number_of_optimization_time_intervals; t++) // for each t
				{
					int link_visit_counts = g_link_vector[l].time_dependent_visit_counts[t];

					g_link_vector[l].time_dependent_LR_multiplier_vector[t] = max(0, g_link_vector[l].time_dependent_LR_multiplier_vector[t] +
						g_penalty_RHO * (link_visit_counts - g_link_vector[l].time_depedent_capacity_vector[t]));
				}
			}
			/////
			if (g_link_vector[l].service_type == 1)   // pick up constraint
			{
				int link_visit_counts = 0;
				for (int t = g_link_vector[l].VRP_time_window_begin; t <= g_link_vector[l].VRP_time_window_end; t++) // for each t
				{
					link_visit_counts += g_link_vector[l].time_dependent_visit_counts[t];
				}

				for (int t = g_link_vector[l].VRP_time_window_begin; t <= g_link_vector[l].VRP_time_window_end; t++) // for each t
				{
					g_link_vector[l].time_dependent_LR_multiplier_vector[t] = g_link_vector[l].time_dependent_LR_multiplier_vector[t] +
						g_link_vector[l].base_price*g_penalty_RHO * (link_visit_counts - 1);  
				}

			}
		}

		// step 2: reset time_dependent_visit_counts  =0 
		for (int l = 0; l < g_link_vector.size(); l++)
		{
			for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
			{
				g_link_vector[l].time_dependent_visit_counts[t] = 0;
			}
		}

		// reset local LR lower bound
		float LR_global_lower_bound = 0;
		float total_price = 0;

		// step 3: find DP optimal solution for each agent 
		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			if (p_agent->agent_service_type != 1)  // service vehicle
			{
				cout << "agent id = " << p_agent->agent_id << endl;

				int internal_origin_node_seq_no = g_internal_node_seq_no_map[p_agent->origin_node_id];  // map external node number to internal node seq no. 
				float trip_price = 0;

				pSTSNetwork.m_agent_vector.clear();
				pSTSNetwork.m_origin_node = internal_origin_node_seq_no;
				pSTSNetwork.m_departure_time_beginning = p_agent->earliest_departure_time;
				pSTSNetwork.m_arrival_time_ending = p_agent->latest_arrival_time;
				pSTSNetwork.m_agent_vector.push_back(a);//pSTSNetwork only has one agent once, but it can be reserved for future parallel computing

				// find the time-dependent state-dependent shortest path for each agent
				trip_price = pSTSNetwork.find_STS_path_for_agents();

				// we record the time-dependent link visit count from each agent 
				total_price += trip_price;
			}

		}  // end for each agent 

		// step 4: count total resource price 
		float total_resource_price = 0;

		for (int l = 0; l < g_link_vector.size(); l++)
		{
			if (g_link_vector[l].service_type == enum_road_capacity_link_type)
			{
				for (int t = 0; t < g_number_of_optimization_time_intervals; t++)
				{
					total_resource_price += g_link_vector[l].time_dependent_LR_multiplier_vector[t] * g_link_vector[l].time_depedent_capacity_vector[t];
				}
			}
		}

		LR_global_lower_bound = total_price - total_resource_price;
		g_best_lower_bound = max(g_best_lower_bound, LR_global_lower_bound);

	} //End for LR iteration

	cout << "End of LR Optimization Process. " << endl;
	return true;
}

void g_OutputFiles()
{
	cout << "outputing files... " << endl;

	//output_agent.csv
	FILE* g_pFileAgent = NULL;
	g_pFileAgent = fopen("output_agent.csv", "w");
	if (g_pFileAgent == NULL)
	{
		cout << "File output_agent.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	else
	{
		fprintf(g_pFileAgent, "agent_id,agent_service_type,origin_node_id,destination_node_id,cost,departure_time_in_min,path_node_sequence,path_link_sequence,path_load_sequence,path_time_sequence, path_travel_time\n");

		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			fprintf(g_pFileAgent, "%d,%d,%d,%d,%f,%.1f,",
				p_agent->agent_id,
				p_agent->agent_service_type,
				p_agent->origin_node_id,
				p_agent->destination_node_id,
				p_agent->path_cost,
				p_agent->departure_time_in_min
			);

			// path node id sequence
			for (int i = 0; i < p_agent->path_node_id_vector.size(); i++)
			{
				int external_node_id = p_agent->path_node_id_vector[i];
				fprintf(g_pFileAgent, "%d;", external_node_id);
			}

			fprintf(g_pFileAgent, ",");
			// path link id sequence 
			for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
			{
				int internal_link_id = p_agent->path_link_seq_no_vector[i];
				int external_link_id = g_link_vector[internal_link_id].external_link_id;

				if (external_link_id >= 1)
				{
					fprintf(g_pFileAgent, "%d;", external_link_id);
				}
			}
			fprintf(g_pFileAgent, ",");

			// served passenger sequence
			for (int i = 0; i < p_agent->path_link_seq_no_vector.size(); i++)
			{
				int internal_link_id = p_agent->path_link_seq_no_vector[i];
				int load_id = g_link_vector[internal_link_id].VRP_load_id;

				if (load_id >= 1)
				{
					fprintf(g_pFileAgent, "%d;", load_id);
				}
			}
			fprintf(g_pFileAgent, ",");

			// path time sequnence
			if (p_agent->path_timestamp_vector.size() >= 1)
			{
				// path node id sequence
				for (int i = 0; i < p_agent->path_timestamp_vector.size(); i++)
				{
					fprintf(g_pFileAgent, "%d;", p_agent->path_timestamp_vector[i]);
				}
			}

			// agent path travel time
			fprintf(g_pFileAgent, ",");
			if (p_agent->path_timestamp_vector.size() >= 1)
			{
				int first_time = p_agent->path_timestamp_vector[0];
				int last_time = p_agent->path_timestamp_vector[p_agent->path_timestamp_vector.size() - 1];
				fprintf(g_pFileAgent, "%d", last_time - first_time);
			}

			fprintf(g_pFileAgent, "\n");
		}
		//
		fclose(g_pFileAgent);
	}

	FILE* g_pFileArcCap = NULL;
	g_pFileArcCap = fopen("output_arc_cap.csv", "w");
	if (g_pFileArcCap == NULL)
	{
		cout << "File output_arc_cap.csv cannot be opened." << endl;
		g_ProgramStop();
	}

	else
	{
		fprintf(g_pFileArcCap, "from_node,to_node,from_time,to_time,capacity\n");

		std::vector <string> generatedArcCapacity;
		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			CAgent* p_agent = &(g_agent_vector[a]);

			// path node id sequence
			for (int i = 0; i < p_agent->path_node_id_vector.size() - 1; i++)
			{
				if (p_agent->path_node_id_vector.size() >= 2)
				{
					int from_node = p_agent->path_node_id_vector[i];
					int to_node = p_agent->path_node_id_vector[i + 1];
					int from_time = p_agent->path_timestamp_vector[i];
					int to_time = p_agent->path_timestamp_vector[i + 1];
					int capacity = 20;

					string arc_capacity_str = to_string(from_node) + "," + to_string(to_node) + "," + to_string(from_time) + "," + to_string(to_time) + "," + to_string(capacity);
					generatedArcCapacity.push_back(arc_capacity_str);
					//fprintf(g_pFileArcCap, "%d,%d,%d,%d,%d\n", from_node, to_node, from_time, to_time, capacity);

				}
				
			}
		}

		sort(generatedArcCapacity.begin(), generatedArcCapacity.end());
		generatedArcCapacity.erase(unique(generatedArcCapacity.begin(), generatedArcCapacity.end()), generatedArcCapacity.end());

		for (int i = 0; i < generatedArcCapacity.size(); i++)
		{
			std::string str = generatedArcCapacity[i];
			char *cstr = &str[0u];
			fprintf(g_pFileArcCap, "%s\n", cstr);
		}
	}

	fclose(g_pFileArcCap);
}

int main()
{
	cout << "reading input_data" << endl;
	//read input data
	g_ReadInputData();

	// run ADMM algorithm
	g_Agent_ADMM();

	// output agent results
	g_OutputFiles();
}