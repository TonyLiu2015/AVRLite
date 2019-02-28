// FlowbasedADMM.cpp : Defines the entry point for the console application.
//@ Jiangtao Liu, Arizona State University

/*
Process of flow-based ADMM:

Step 1: read input data: input_agent.csv, input_arc_capacity.csv, and input_passenger_demand.csv

Step 2: perform the Newton method for each path/column
	step 2.1: initialization for passenger rho, arc capacity rho, passenger sub_gradient, arc capacity sub_gradient, incidences of path-to-passenger and path-to-arc
	step 2.2: each iteration for ADMM
		step 2.2.1: update the multiplier
		step 2.2.2: update each colulmn flow
		step 2.2.3: calculate the objective value for each iteration

Step 3: output solution
*/

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include "CSVParser.h"
#include <string> 
#include <algorithm>

using namespace std;

FILE* g_pFile_OutputFlow = NULL;
FILE* g_pFile_OutputObj = NULL;
FILE* g_pFile_OutputUBFlow = NULL;
FILE* g_pFile_OutputUBArcFlow = NULL;
FILE* g_pFile_OutputUBObj = NULL;

#define _MAX_NUMBER_OF_DEMAND_GROUPS 10000
#define _MAX_NUMBER_OF_ARCS 60000
#define _MAX_NUMBER_OF_AGENTS 10000
#define _MAX_NUMBER_OF_ITERATIONS 500

int g_number_of_agents = 0;
int g_number_of_arcs = 0;
int g_number_of_pax_groups = 0;

int g_iteration_number = 250;

std::vector<int> g_pax_group_id_vector;
std::vector<float> g_group_demand_vector;
std::vector<string> g_arc_vector;

std::vector<string> g_GAMS_arc_cap;
std::vector<float> g_arc_cap_vector;

map<int, string> g_arc_GAMS_format_map;

float g_optimal_solution[_MAX_NUMBER_OF_ITERATIONS];

int** delta_pax = NULL;
int** delta_arc = NULL;

float roh_pax = 3;
float roh_arc = 1;

float Subgradient_pickup_pax[_MAX_NUMBER_OF_DEMAND_GROUPS];
float Subgradient_visit_arc[_MAX_NUMBER_OF_ARCS];
float served_times_for_pax[_MAX_NUMBER_OF_DEMAND_GROUPS];
float served_times_for_arc[_MAX_NUMBER_OF_ARCS];
float g_arc_flow[_MAX_NUMBER_OF_ARCS];

int g_virtual_vehicle_cost = 60;
float g_upper_bound=0.0;

//clock_t start_t, end_t, total_t;

class CAgent
{
public:
	int agent_id;
	float column_cost;
	float column_flow;
	string served_pax_group;
	std::vector<int> served_pax_group_vector;
	string column_node_seq;
	string column_node_time_seq;
	std::vector<int> column_node_vector;
	std::vector<int> column_node_time_vector;
	std::vector<int> column_node_arc_vector;
	std::vector<string> column_node_time_arc_vector;

	CAgent()
	{
		agent_id = 0;
		column_cost = 0.0;
		column_flow = 10.0;
	}
};

std::vector <CAgent> g_agent_vector;

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

template <typename T>
T **AllocateDynamicArray(int nRows, int nCols, int initial_value = 0)
{
	T **dynamicArray;

	dynamicArray = new (std::nothrow) T*[nRows];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();

	}

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new (std::nothrow) T[nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int j = 0; j < nCols; j++)
		{
			dynamicArray[i][j] = initial_value;
		}
	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray, int nRows, int nCols)
{
	if (!dArray)
		return;

	for (int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete[] dArray;

}

void g_read_input_data()
{
	g_number_of_agents = 0;
	g_number_of_arcs = 0;
	g_number_of_pax_groups = 0;

	// step 1: read agent/column file 
	CCSVParser parser;
	if (parser.OpenCSVFile("input_agent.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			CAgent agent;

			// read column id
			int r_agent_id;
			parser.GetValueByFieldName("agent_id", r_agent_id);
			agent.agent_id = r_agent_id;

			float r_column_cost;
			parser.GetValueByFieldName("column_cost", r_column_cost);
			agent.column_cost = r_column_cost;

			string r_served_pax_group;
			parser.GetValueByFieldName("served_pax_group", r_served_pax_group);
			agent.served_pax_group = r_served_pax_group;
			agent.served_pax_group_vector = ParseLineToIntegers(r_served_pax_group);

			string r_column_node_seq;
			parser.GetValueByFieldName("Path_node_sequence", r_column_node_seq);
			agent.column_node_seq = r_column_node_seq;
			agent.column_node_vector = ParseLineToIntegers(r_column_node_seq);

			string r_column_node_time_seq;
			parser.GetValueByFieldName("path_time_sequence", r_column_node_time_seq);
			agent.column_node_time_seq = r_column_node_time_seq;
			agent.column_node_time_vector = ParseLineToIntegers(r_column_node_time_seq);

			string key_node_time_arc;
			for (int i = 0; i < agent.column_node_vector.size() - 1; i++)
			{
				key_node_time_arc = '(' + NumberToString(agent.column_node_vector[i]) + '_' + NumberToString(agent.column_node_vector[i + 1]) + '_' + NumberToString(agent.column_node_time_vector[i]) + '_' + NumberToString(agent.column_node_time_vector[i + 1]) + ')';
				agent.column_node_time_arc_vector.push_back(key_node_time_arc);
			}

			if (g_number_of_agents % 500 == 0)
			{
				cout << "reading agent number: " << g_number_of_agents << endl;
			}
			
			g_agent_vector.push_back(agent);
			g_number_of_agents++;
		}
		parser.CloseCSVFile();
	}

	if (parser.OpenCSVFile("input_passenger_demand.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int key_pax_group_id;
			parser.GetValueByFieldName("Pax_group_id", key_pax_group_id);
			g_pax_group_id_vector.push_back(key_pax_group_id);

			float key_pax_group_demand;
			parser.GetValueByFieldName("total_demand", key_pax_group_demand);
			g_group_demand_vector.push_back(key_pax_group_demand);

			if (g_number_of_pax_groups % 500 == 0)
			{
				cout << "reading pax group number: " << g_number_of_pax_groups << endl;
			}

			g_number_of_pax_groups++;
		}
		parser.CloseCSVFile();
	}

	if (parser.OpenCSVFile("input_arc_capacity.csv", true))
	{
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node, to_node, from_time, to_time;
			float arc_capacity;

			parser.GetValueByFieldName("from_node", from_node);
			parser.GetValueByFieldName("to_node", to_node);
			parser.GetValueByFieldName("from_time", from_time);
			parser.GetValueByFieldName("to_time", to_time);
			parser.GetValueByFieldName("arc_capacity", arc_capacity);

			string key_arc_cap = '(' + NumberToString(from_node) + '_' + NumberToString(to_node) + '_' + NumberToString(from_time) + '_' + NumberToString(to_time) + ')';
			string GAMS_arc_key = NumberToString(from_node) + '.' + NumberToString(to_node) + '.' + NumberToString(from_time) + '.' + NumberToString(to_time) + ' '+ NumberToString(arc_capacity);
			string arc_GAMS = NumberToString(from_node) + '.' + NumberToString(to_node) + '.' + NumberToString(from_time) + '.' + NumberToString(to_time);
			
			g_arc_GAMS_format_map[g_number_of_arcs] = arc_GAMS;
			
			g_GAMS_arc_cap.push_back(GAMS_arc_key);
			g_arc_vector.push_back(key_arc_cap);
			g_arc_cap_vector.push_back(arc_capacity);

			if (g_number_of_arcs % 1000 == 0)
			{
				cout << "reading arc number: " << g_number_of_arcs << endl;
			}

			g_number_of_arcs++;
		}
		parser.CloseCSVFile();
	}
}

void g_ADMM_Newton()
{
	cout << "Using ADMM method" << endl;
	// dynamic rhos for pax and arc capacity
	float roh_change_pax[_MAX_NUMBER_OF_ITERATIONS];
	float roh_change_arc[_MAX_NUMBER_OF_ITERATIONS];

	for (int i = 0; i < g_iteration_number + 1; i++)
	{
		roh_change_pax[i] = 0.1;
		roh_change_arc[i] = 0.1;
	}

	// multipliers for pax, arc
	float lambda_pax[_MAX_NUMBER_OF_DEMAND_GROUPS];
	float lambda_arc[_MAX_NUMBER_OF_ARCS];

	float lambda_pax_LR[_MAX_NUMBER_OF_DEMAND_GROUPS];
	float lambda_arc_LR[_MAX_NUMBER_OF_ARCS];

	//initilization
	for (int i = 0; i < g_pax_group_id_vector.size(); i++)
	{
		Subgradient_pickup_pax[i] = 0.1;
		lambda_pax[i] = 0.1;
		lambda_pax_LR[i] = 0.1;
	}

	for (int i = 0; i < g_arc_vector.size(); i++)
	{
		Subgradient_visit_arc[i] = 0.1;
		lambda_arc[i] = 0.1;
		lambda_arc_LR[i] = 0.1;
	}

	//int delta_pax[_MAX_NUMBER_OF_AGENTS][_MAX_NUMBER_OF_DEMAND_GROUPS];
	//int delta_arc[_MAX_NUMBER_OF_AGENTS][_MAX_NUMBER_OF_ARCS];

	cout << "generating path-pax and path-arc parameters" << endl;
	// incidence of column/path-to-pax and path-to-arc
	delta_pax = AllocateDynamicArray<int>(g_number_of_agents, g_number_of_pax_groups, 0);
	delta_arc = AllocateDynamicArray<int>(g_number_of_agents, g_number_of_arcs, 0);

	for (int i = 0; i < g_agent_vector.size(); i++)
	{
		for (int j = 0; j < g_pax_group_id_vector.size(); j++)
		{
			delta_pax[i][j] = 0;
		}

		for (int k = 0; k < g_arc_vector.size(); k++)
		{
			delta_arc[i][k] = 0;
		}
	}

	for (int m = 0; m < g_agent_vector.size(); m++)
	{
		for (int k = 0; k < g_pax_group_id_vector.size(); k++)
		{
			for (int n = 0; n < g_agent_vector[m].served_pax_group_vector.size(); n++)
			{
				if (g_agent_vector[m].served_pax_group_vector[n] == g_pax_group_id_vector[k])
				{
					delta_pax[m][k] = 1;
				}
			}
		}
	}

	for (int m = 0; m < g_agent_vector.size(); m++)
	{
		for (int r = 0; r < g_arc_vector.size(); r++)
		{
			for (int s = 0; s < g_agent_vector[m].column_node_time_arc_vector.size(); s++)
			{
				if (g_agent_vector[m].column_node_time_arc_vector[s] == g_arc_vector[r])
				{
					delta_arc[m][r] = 1;
				}
			}
		}
	}

	float StepSize = 1;

	cout << "begin for each iteration" << endl;

	clock_t start_t, end_t, total_t;
	start_t = clock();

	// each iteration
	for (int i = 1; i < g_iteration_number + 1; i++)
	{
		//sum(k, f(k)*delta(k,p))
		for (int j = 0; j < g_pax_group_id_vector.size(); j++)
		{
			served_times_for_pax[j] = 0;
		}

		//sum(k, f(k)*delta(k,i,j,t))
		for (int j = 0; j < g_arc_vector.size(); j++)
		{
			served_times_for_arc[j] = 0;
		}
		//
		float sub_cost_pax[_MAX_NUMBER_OF_AGENTS];
		float sub_cost_arc[_MAX_NUMBER_OF_AGENTS];
		float penalty_subgradient_agent_pax[_MAX_NUMBER_OF_AGENTS];
		float penalty_subgradient_agent_arc[_MAX_NUMBER_OF_AGENTS];
		float first_gradient_agent_pax[_MAX_NUMBER_OF_AGENTS];
		float s_coefficient_pax[_MAX_NUMBER_OF_AGENTS];
		float first_gradient_agent_arc[_MAX_NUMBER_OF_AGENTS];
		float s_coefficient_arc[_MAX_NUMBER_OF_AGENTS];
		float s_coefficient[_MAX_NUMBER_OF_AGENTS];

		for (int j = 0; j < g_agent_vector.size(); j++)
		{
			sub_cost_pax[j] = 0.0;
			sub_cost_arc[j] = 0.0;

			penalty_subgradient_agent_pax[j] = 0.0;
			penalty_subgradient_agent_arc[j] = 0.0;

			first_gradient_agent_pax[j] = 0.0;
			s_coefficient_pax[j] = 0.0;

			first_gradient_agent_arc[j] = 0.0;
			s_coefficient_arc[j] = 0.0;

			s_coefficient[j] = 0.0;
		}

		float obj_1 = 0.0;
		float obj_2 = 0.0;
		float obj_3 = 0.0;
		float obj_4 = 0.0;
		float obj_5 = 0.0;

		StepSize = 1 / (g_iteration_number + 1.0f);

		for (int k = 0; k < g_pax_group_id_vector.size(); k++)
		{
			for (int m = 0; m < g_agent_vector.size(); m++)
			{
				//sum(k, f(k)*delta(k,p))
				served_times_for_pax[k] = served_times_for_pax[k] + g_agent_vector[m].column_flow*delta_pax[m][k];
			}
			// sum(k, f(k)*delta(k,p))- q(p)
			Subgradient_pickup_pax[k] = served_times_for_pax[k] - g_group_demand_vector[k];

			//update lambda for side constriants
			lambda_pax[k] = lambda_pax[k] + roh_pax*Subgradient_pickup_pax[k];
			lambda_pax_LR[k] = lambda_pax_LR[k] + StepSize*Subgradient_pickup_pax[k];
		}

		for (int r = 0; r < g_arc_vector.size(); r++)
		{
			for (int m = 0; m < g_agent_vector.size(); m++)
			{
				//x_a*delat(a,arc(i,j,t,s))
				served_times_for_arc[r] = served_times_for_arc[r] + g_agent_vector[m].column_flow*delta_arc[m][r];
			}
			// sum(a,x_a*delat(a,arc))-cap(arc) 
			Subgradient_visit_arc[r] = served_times_for_arc[r] - g_arc_cap_vector[r];

			//update lambda for side constriants
			lambda_arc[r] = max(0, lambda_arc[r] + roh_arc*Subgradient_visit_arc[r]);
			lambda_arc_LR[r] = max(0, lambda_arc_LR[r] + StepSize*Subgradient_visit_arc[r]);
		}

		for (int m = 0; m < g_agent_vector.size(); m++)
		{
			for (int k = 0; k < g_pax_group_id_vector.size(); k++)
			{
				//sum(p,delta(k, p)*[sum(k, f(k)*delat(k, p)) - q(p)])
				sub_cost_pax[m] = sub_cost_pax[m] + Subgradient_pickup_pax[k] * delta_pax[m][k];
				//sum(p,lambda(p)*delat(k,p))
				penalty_subgradient_agent_pax[m] = penalty_subgradient_agent_pax[m] + lambda_pax[k] * delta_pax[m][k];
			}

			for (int r = 0; r < g_arc_vector.size(); r++)
			{
				//sum(arc,delta(k, arc)*[sum(k, f(k)*delat(k, arc)) - cap(arc)])
				sub_cost_arc[m] = sub_cost_arc[m] + Subgradient_visit_arc[r] * delta_arc[m][r];
				//sum(arc, lambda(arc)*delta(k, arc))
				penalty_subgradient_agent_arc[m] = penalty_subgradient_agent_arc[m] + lambda_arc[r] * delta_arc[m][r];
			}

			//rho_pax*sum(p,delta(k, p)*[sum(k, f(k)*delat(k, p)) - q(p)])
			first_gradient_agent_pax[m] = roh_pax*sub_cost_pax[m];
			// cofficient of pax: second derivative part for pax
			s_coefficient_pax[m] = roh_pax*g_agent_vector[m].served_pax_group_vector.size();

			// roh_arc*sum(arc,delta(k, arc)*[sum(k, f(k)*delat(k, arc)) - cap(arc)])
			first_gradient_agent_arc[m] = roh_arc*sub_cost_arc[m];
			// cofficient of arc: second derivative part for arc
			s_coefficient_arc[m] = roh_arc*g_agent_vector[m].column_node_time_arc_vector.size();

			// the exact cofficent for column k
			s_coefficient[m] = s_coefficient_pax[m] + s_coefficient_arc[m];

			//update column flow based on the newton method
			float previous_column_flow = 0.0;
			previous_column_flow = g_agent_vector[m].column_flow;

			g_agent_vector[m].column_flow = max(0, g_agent_vector[m].column_flow - (1.0 / s_coefficient[m])*(g_agent_vector[m].column_cost + penalty_subgradient_agent_pax[m] + first_gradient_agent_pax[m] + penalty_subgradient_agent_arc[m] + first_gradient_agent_arc[m]));

			float update_column_flow = 0.0;
			update_column_flow = g_agent_vector[m].column_flow;
			/*for (int j = 0; j < g_pax_group_id_vector.size(); j++)
			{
				served_times_for_pax[j] = 0;
			}*/

			//update the subgrident of paxs for the next vehicle in ADMM
			for (int k = 0; k < g_pax_group_id_vector.size(); k++)
			{
				/*for (int n = 0; n < g_agent_vector.size(); n++)
				{
					served_times_for_pax[k] = served_times_for_pax[k] + g_agent_vector[n].column_flow*delta_pax[n][k];					
				}*/

				served_times_for_pax[k] += (update_column_flow- previous_column_flow)*delta_pax[m][k];

				Subgradient_pickup_pax[k] = served_times_for_pax[k] - g_group_demand_vector[k];
			}

			//update the subgrident of arcs for the next vehicle in ADMM
			/*for (int j = 0; j < g_arc_vector.size(); j++)
			{
				served_times_for_arc[j] = 0;
			}*/

			for (int r = 0; r < g_arc_vector.size(); r++)
			{
				/*for (int n = 0; n < g_agent_vector.size(); n++)
				{
					served_times_for_arc[r] = served_times_for_arc[r] + g_agent_vector[n].column_flow*delta_arc[n][r];
				}*/

				served_times_for_arc[r] += (update_column_flow - previous_column_flow)*delta_arc[m][r];
				Subgradient_visit_arc[r] = served_times_for_arc[r] - g_arc_cap_vector[r];
			}
		}

		/*for (int m = 0; m < g_agent_vector.size(); m++)
		{
			cout << "column flow:" << g_agent_vector[m].column_flow << endl;
		}*/
		

		for (int m = 0; m < g_agent_vector.size(); m++)
		{
			obj_1 = obj_1 + g_agent_vector[m].column_flow*g_agent_vector[m].column_cost;
		}

		float obj_6=0.0;
		float obj_7=0.0;

		for (int k = 0; k < g_pax_group_id_vector.size(); k++)
		{
			obj_2 = obj_2 + lambda_pax[k] * Subgradient_pickup_pax[k];
			obj_3 = obj_3 + Subgradient_pickup_pax[k] * Subgradient_pickup_pax[k];
			obj_6 = obj_6 + lambda_pax_LR[k] * Subgradient_pickup_pax[k];
		}

		for (int r = 0; r < g_arc_vector.size(); r++)
		{
			obj_4 = obj_4 + lambda_arc[r] * Subgradient_visit_arc[r];
			obj_5 = obj_5 + Subgradient_visit_arc[r] * Subgradient_visit_arc[r];
			obj_7 = obj_7 + lambda_arc_LR[r] * Subgradient_visit_arc[r];
		}

		//g_optimal_solution[i] = obj_1 + obj_2 + obj_4;
		g_optimal_solution[i] = obj_1;
		//g_optimal_solution[i] = obj_1 + obj_2 + obj_3 + obj_4 + obj_5;

		//g_optimal_solution[i] = obj_1 + obj_6 + obj_7;

		roh_change_pax[i] = obj_3;
		roh_change_arc[i] = obj_5;

		//conditions to change roh for pax and arc
		if (roh_change_pax[i] > 0.4*roh_change_pax[i - 1])
		{
			roh_pax += 0;
		}

		if (roh_change_arc[i] > 0.4*roh_change_arc[i - 1])
		{
			roh_arc += 0;
		}						

		cout << "objective_value is " << g_optimal_solution[i] << " at iteration " << i << endl;
	}

	end_t = clock();
	total_t = (end_t - start_t);

	cout << "CPU Running Time = " << total_t << " milliseconds" << endl;
}

void g_Upper_Bound()
{
	int indicator_infeasiblity_pax = 0;
	int indicator_infeasiblity_arc = 0;
	for (int i = 0; i < g_pax_group_id_vector.size(); i++)
	{
		if (Subgradient_pickup_pax[i]<0.1 && Subgradient_pickup_pax[i]>-0.1)
		{
			//cout << "pax pick-up solution is feasible" << endl;
		}
		else
		{
			indicator_infeasiblity_pax = 1;
		}
	}

	for (int i = 0; i < g_arc_vector.size(); i++)
	{
		if (Subgradient_visit_arc[i] > 0)
		{
			indicator_infeasiblity_arc = 1;
		}
		else
		{
			//cout << "arc capacity is feasible" << endl;
		}
	}

	if (indicator_infeasiblity_pax == 1)
	{
		cout << "pax_service is infeasible" << endl;
	}

	if (indicator_infeasiblity_arc == 1)
	{
		cout << "arc_capacity is infeasible" << endl;
	}

	//initialization
	for (int j = 0; j < g_pax_group_id_vector.size(); j++)
	{
		served_times_for_pax[j] = 0;
	}

	for (int j = 0; j < g_arc_vector.size(); j++)
	{
		served_times_for_arc[j] = 0;
	}

	for (int i = 0; i < g_agent_vector.size(); i++)
	{
		for (int j = 0; j < g_pax_group_id_vector.size(); j++)
		{
			served_times_for_pax[j] += g_agent_vector[i].column_flow * delta_pax[i][j];

			if (served_times_for_pax[j] > g_group_demand_vector[j])
			{
				float served_exceeded_times_pax = served_times_for_pax[j] - g_group_demand_vector[j];
				g_agent_vector[i].column_flow = g_agent_vector[i].column_flow - served_exceeded_times_pax;
				served_times_for_pax[j] = served_times_for_pax[j] - served_exceeded_times_pax;
			}
		}

		float served_exceeded_times_arc=0.0;

		for (int j = 0; j < g_arc_vector.size(); j++)
		{
			served_times_for_arc[j] += g_agent_vector[i].column_flow * delta_arc[i][j];

			if (served_times_for_arc[j] > g_arc_cap_vector[j])
			{
				served_exceeded_times_arc = served_times_for_arc[j] - g_arc_cap_vector[j];
				g_agent_vector[i].column_flow = g_agent_vector[i].column_flow - served_exceeded_times_arc;
				served_times_for_arc[j] = served_times_for_arc[j] - served_exceeded_times_arc;
			}
		}

		for (int j = 0; j < g_pax_group_id_vector.size(); j++)
		{
			served_times_for_pax[j] -= served_exceeded_times_arc * delta_pax[i][j];
		}
		//g_upper_bound += g_agent_vector[i].column_flow * g_agent_vector[i].column_cost;
	}

	float needed_virtual_vehicles = 0.0;
	for (int j = 0; j < g_pax_group_id_vector.size(); j++)
	{
		if (served_times_for_pax[j] < g_group_demand_vector[j])
		{			
			needed_virtual_vehicles += (g_group_demand_vector[j] - served_times_for_pax[j]);
			//g_upper_bound += needed_virtual_vehicles* g_virtual_vehicle_cost;
		}
	}

	CAgent agent;
	agent.column_flow = needed_virtual_vehicles;
	agent.column_cost = g_virtual_vehicle_cost;
	g_agent_vector.push_back(agent);

	for (int i = 0; i < g_agent_vector.size(); i++)
	{
		g_upper_bound += g_agent_vector[i].column_flow * g_agent_vector[i].column_cost;
	}
	cout << "the upper bound vaule is " << g_upper_bound << endl;
}

void g_ProgramStop()
{
	cout << "Flow-based ADMM Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
}

void g_output_ADMM_solutions()
{
	cout << "output ADMM solution" << endl;

	g_pFile_OutputFlow = fopen("output_ADMM_flow_solution.csv", "w");
	if (g_pFile_OutputFlow == NULL)
	{
		cout << "File output_flow_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputFlow, "column_id, column_flow\n");

	for (int m = 0; m < g_agent_vector.size(); m++)
	{
		fprintf(g_pFile_OutputFlow, "%d,%f\n", m + 1, g_agent_vector[m].column_flow);
	}

	// output the objective value of each iteration
	g_pFile_OutputObj = fopen("output_ADMM_iterative_solution.csv", "w");
	if (g_pFile_OutputObj == NULL)
	{
		cout << "File output_iterative_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputObj, "iteration_id, objective_value\n");

	for (int i = 1; i < g_iteration_number + 1; i++)
	{
		fprintf(g_pFile_OutputObj, "%d,%f\n", i, g_optimal_solution[i]);
	}

	fclose(g_pFile_OutputFlow);
	fclose(g_pFile_OutputObj);
}

void g_output_GAMS_Parameters()
{
	//output_GAMS_parameter.txt
	cout << "output GAMS parameter results" << endl;

	FILE* g_pFileGAMS = NULL;
	g_pFileGAMS = fopen("output_GAMS_parameter.txt", "w");

	if (g_pFileGAMS == NULL)
	{
		cout << "File output_GAMS_parameter.txt cannot be opened." << endl;
		g_ProgramStop();
	}

	else
	{
		fprintf(g_pFileGAMS, "parameter path_cost(p)/\n");
		for (int a = 0; a < g_agent_vector.size(); a++)
		{
			fprintf(g_pFileGAMS, "%d %.2f\n", g_agent_vector[a].agent_id, g_agent_vector[a].column_cost);
		}
		fprintf(g_pFileGAMS, "/;\n");

		fprintf(g_pFileGAMS, "parameter pax_group(a)/\n");
		for (int a = 0; a < g_pax_group_id_vector.size(); a++)
		{
			fprintf(g_pFileGAMS, "%d %.2f\n", g_pax_group_id_vector[a], g_group_demand_vector[a]);
		}
		fprintf(g_pFileGAMS, "/;\n");

		fprintf(g_pFileGAMS, "parameter arc_cap(i,j,t,s)/\n");
		for (int a = 0; a < g_GAMS_arc_cap.size(); a++)
		{
			std::string str = g_GAMS_arc_cap[a];
			char *cstr = &str[0u];
			fprintf(g_pFileGAMS, "%s \n", cstr);
		}
		fprintf(g_pFileGAMS, "/;\n");

		fprintf(g_pFileGAMS, "parameter path_link_ind(p,i,j,t,s)/\n");
		for (int i = 0; i < g_agent_vector.size(); i++)
		{
			for (int j = 0; j < g_arc_vector.size(); j++)
			{
				if (delta_arc[i][j] == 1)
				{
					std::string str = g_arc_GAMS_format_map[j];
					char *cstr = &str[0u];
					fprintf(g_pFileGAMS, "%d.%s %d\n", g_agent_vector[i].agent_id, cstr, 1);
				}
			}
		}
		fprintf(g_pFileGAMS, "/;\n");

		fprintf(g_pFileGAMS, "parameter path_pax_ind(p,a)/\n");
		for (int i = 0; i < g_agent_vector.size(); i++)
		{
			for (int j = 0; j < g_pax_group_id_vector.size(); j++)
			{
				if (delta_pax[i][j] == 1)
				{
					fprintf(g_pFileGAMS, "%d.%d %d\n", g_agent_vector[i].agent_id, g_pax_group_id_vector[j], 1);
				}
			}
		}
		fprintf(g_pFileGAMS, "/;\n");
	}
}

void g_output_upper_bound_solutions()
{
	g_pFile_OutputUBFlow = fopen("output_UB_flow_solution.csv", "w");
	if (g_pFile_OutputUBFlow == NULL)
	{
		cout << "File output_flow_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputUBFlow, "column_id, column_flow\n");

	for (int m = 0; m < g_agent_vector.size(); m++)
	{
		fprintf(g_pFile_OutputUBFlow, "%d,%f\n", m + 1, g_agent_vector[m].column_flow);
	}

	// output_arc_flow
	g_pFile_OutputUBArcFlow = fopen("output_UB_Arc_flow_solution.csv", "w");
	if (g_pFile_OutputUBArcFlow == NULL)
	{
		cout << "File output_UB_Arc_flow_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputUBArcFlow, "space_time_arc, arc_flow\n");

	for (int j = 0; j < g_arc_vector.size(); j++)
	{
		g_arc_flow[j] = 0.0;
	}

	for (int j = 0; j < g_arc_vector.size(); j++)
	{
		for (int i = 0; i < g_agent_vector.size() - 1; i++)
		{
			if (delta_arc[i][j] == 1)
			{
				g_arc_flow[j] += g_agent_vector[i].column_flow;
			}
		}

		std::string str = g_arc_vector[j];
		char *cstr = &str[0u];
		fprintf(g_pFile_OutputUBArcFlow, "%s,%f\n", cstr, g_arc_flow[j]);
	}

	// output the objective value of each iteration
	g_pFile_OutputUBObj = fopen("output_UB.csv", "w");
	if (g_pFile_OutputUBObj == NULL)
	{
		cout << "File output_iterative_solution.csv cannot be opened." << endl;
		g_ProgramStop();
	}
	fprintf(g_pFile_OutputUBObj, "objective_value\n");

	fprintf(g_pFile_OutputUBObj, "%f\n", g_upper_bound);

	fclose(g_pFile_OutputUBFlow);
	fclose(g_pFile_OutputUBArcFlow);
	fclose(g_pFile_OutputUBObj);
}

int main()
{
	cout << "reading input_data" << endl;
	g_read_input_data();

	g_ADMM_Newton();

	g_output_ADMM_solutions();

	g_output_GAMS_Parameters();

	g_Upper_Bound();

	g_output_upper_bound_solutions();

	DeallocateDynamicArray<int>(delta_pax, g_number_of_agents, g_number_of_pax_groups);
	DeallocateDynamicArray<int>(delta_arc, g_number_of_agents, g_number_of_arcs);

	cout << "well done!" << endl;

	getchar();
}

