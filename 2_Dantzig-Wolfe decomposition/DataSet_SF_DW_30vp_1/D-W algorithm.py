#Dantzig-Wolfe algorithm for Autonomous vehicle assignment with road capacity @ Jiangtao Liu

from gams import *
import os
import sys
import csv
import timeit

g_number_of_nodes = 0
g_number_of_time_intervals= 0
g_number_of_vehicles= 0
g_number_of_k_paths_iterations= 0
g_number_of_passengers= 0

g_arc_capacity_list = []
g_arc_capacity_new_list = []
g_incid_veh_path_arc_list = []
g_incid_veh_path_arc_new_list = []
g_incid_veh_path_pax_list = []
g_incid_veh_path_pax_new_list = []
g_incid_veh_path_cost_list = []
g_incid_veh_path_cost_new_list = []
g_incid_veh_path_selection_list = []
g_incid_veh_path_selection_new_list = []

g_internal_vehicle_list = []
g_vehicle_list = []
g_Path_No_list = []
g_Path_Node_seq_list = []
g_Path_Node_time_seq_list = []
g_Path_Node_state_seq_list = []
g_Path_link_seq_list = []
g_Path_cost_list = []
g_Passenger_served_list = []

g_vehicle_solution_list= []
g_path_solution_list= []

def g_UpdateNetworkData():
    global g_number_of_nodes 
    global g_number_of_time_intervals
    global g_number_of_vehicles
    global g_number_of_k_paths_iterations
    global g_number_of_passengers
    global g_arc_capacity_list
    global g_arc_capacity_new_list
    global g_incid_veh_path_arc_list
    global g_incid_veh_path_arc_new_list
    global g_incid_veh_path_pax_list
    global g_incid_veh_path_pax_new_list
    global g_incid_veh_path_cost_list
    global g_incid_veh_path_cost_new_list
    global g_incid_veh_path_selection_list
    global g_incid_veh_path_selection_new_list

    
    with open('Internal_GAMS_input_arc_cap.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_arc_capacity_list.append(l[0])
        
    with open('Sub_GAMS_input_arc_cap.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_arc_capacity_list.append(l[0])
  
    with open('Internal_GAMS_input_incid_veh_path_arc.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_arc_list.append(l[0])
        
    with open('Sub_GAMS_input_incid_veh_path_arc.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_arc_list.append(l[0])

    with open('Internal_GAMS_input_incid_veh_path_pax.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_pax_list.append(l[0])
        
    with open('Sub_GAMS_input_incid_veh_path_pax.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_pax_list.append(l[0])

    with open('Internal_GAMS_input_veh_path_cost.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_cost_list.append(l[0])
        
    with open('Sub_GAMS_input_veh_path_cost.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_cost_list.append(l[0])

    with open('Internal_GAMS_input_veh_path_selection.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_selection_list.append(l[0])
        
    with open('Sub_GAMS_input_veh_path_selection.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_incid_veh_path_selection_list.append(l[0])

    with open('Internal_GAMS_input_set.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:2]:
            l = l.strip().split(',')
            g_number_of_nodes = int(l[0]) 
            g_number_of_time_intervals = int(l[1])
            g_number_of_vehicles = int(l[2])
            g_number_of_k_paths_iterations = int(l[3])
            g_number_of_passengers = int(l[4])

    g_arc_capacity_new_list=g_remove_duplication(g_arc_capacity_list)
    g_incid_veh_path_arc_new_list=g_remove_duplication(g_incid_veh_path_arc_list)
    g_incid_veh_path_pax_new_list=g_remove_duplication(g_incid_veh_path_pax_list)
    g_incid_veh_path_cost_new_list=g_remove_duplication(g_incid_veh_path_cost_list)
    g_incid_veh_path_selection_new_list=g_remove_duplication(g_incid_veh_path_selection_list)
            

def g_remove_duplication(lst):
    res = []
    for x in lst:
        if x not in res:
            res.append(x)
    return res

def g_output_ite_num_VRP(m,n):
    with open("Py_output_num_VRP.csv", "w") as output:
        output.write("Current_iteration_number,Total_number_of_iterations\n")
        output.write("%d,%d\n" % (m,n))

def g_output_GAMS_inputs():
    global g_number_of_nodes 
    global g_number_of_time_intervals
    global g_number_of_vehicles
    global g_number_of_k_paths_iterations
    global g_number_of_passengers
    global g_arc_capacity_new_list   
    global g_incid_veh_path_arc_new_list   
    global g_incid_veh_path_pax_new_list 
    global g_incid_veh_path_cost_new_list 
    global g_incid_veh_path_selection_new_list
    with open("GAMS_input_set.txt", "w") as output:
        output.write("OPTIONS  MIP = CPLEX;"+'\n')
        output.write("Set i /1*%d/;\n" % g_number_of_nodes)
        output.write("Set t /1*%d/;\n" % g_number_of_time_intervals)
        output.write("Set v /1*%d/;\n" % g_number_of_vehicles)
        output.write("Set k /0*%d/;\n" % g_number_of_k_paths_iterations)
        output.write("Set p /1*%d/;\n" % g_number_of_passengers)
        output.write("alias(i, j);\n")
        output.write("alias(t, s);\n")

    with open("GAMS_input_parameter.txt", "w") as output:
        output.write("parameter c(v,k)/\n")
        for j in range(0,len(g_incid_veh_path_cost_new_list)):
            output.write("%s\n" % g_incid_veh_path_cost_new_list[j])
        output.write("/;\n")

        output.write("parameter x(v,k)/\n")
        for j in range(0,len(g_incid_veh_path_selection_new_list)):
            output.write("%s\n" % g_incid_veh_path_selection_new_list[j])
        output.write("/;\n")

        output.write("parameter delta(v,k,p)/\n")
        for j in range(0,len(g_incid_veh_path_pax_new_list)):
            output.write("%s\n" % g_incid_veh_path_pax_new_list[j])
        output.write("/;\n")

        output.write("parameter beta(v,k,i,j,t,s)/\n")
        for j in range(0,len(g_incid_veh_path_arc_new_list)):
            output.write("%s\n" % g_incid_veh_path_arc_new_list[j])
        output.write("/;\n")

        output.write("parameter cap(i,j,t,s)/\n")
        for j in range(0,len(g_arc_capacity_new_list)):
            output.write("%s\n" % g_arc_capacity_new_list[j])
        output.write("/;\n")

def g_output_initial_agent_VRP_solution():
    
    global g_internal_vehicle_list
    global g_Path_No_list
    global g_Path_Node_seq_list
    global g_Path_Node_time_seq_list
    global g_Path_Node_state_seq_list
    global g_Path_link_seq_list
    global g_Path_cost_list
    global g_Passenger_served_list

    with open('output_agent.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_internal_vehicle_list.append(l[0])
            g_vehicle_list.append(l[1])
            g_Path_No_list.append(l[2])
            g_Path_Node_seq_list.append(l[3])
            g_Path_Node_time_seq_list.append(l[4])
            g_Path_Node_state_seq_list.append(l[5])
            g_Path_link_seq_list.append(l[6])
            g_Path_cost_list.append(l[7])
            g_Passenger_served_list.append(l[8])

def g_output_agent_VRP_solution():
    global g_internal_vehicle_list
    global g_vehicle_list
    global g_Path_No_list
    global g_Path_Node_seq_list
    global g_Path_Node_time_seq_list
    global g_Path_Node_state_seq_list
    global g_Path_link_seq_list
    global g_Path_cost_list
    global g_Passenger_served_list
    with open('output_agent_internal.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_internal_vehicle_list.append(l[0])
            g_vehicle_list.append(l[1])
            g_Path_No_list.append(l[2])
            g_Path_Node_seq_list.append(l[3])
            g_Path_Node_time_seq_list.append(l[4])
            g_Path_Node_state_seq_list.append(l[5])
            g_Path_link_seq_list.append(l[6])
            g_Path_cost_list.append(l[7])
            g_Passenger_served_list.append(l[8])

    with open("output_agent.csv", "w") as output:   
        for j in range(0,len(g_vehicle_list)):
            output.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (g_internal_vehicle_list[j],g_vehicle_list[j],g_Path_No_list[j],
            g_Path_Node_seq_list[j],g_Path_Node_time_seq_list[j],g_Path_Node_state_seq_list[j],
            g_Path_link_seq_list[j],g_Path_cost_list[j],g_Passenger_served_list[j]))

def g_output_GAMS_solution():
    global g_internal_vehicle_list
    global g_vehicle_list
    global g_Path_No_list
    global g_Path_Node_seq_list
    global g_Path_Node_time_seq_list
    global g_Path_Node_state_seq_list
    global g_Path_link_seq_list
    global g_Path_cost_list
    global g_Passenger_served_list

    global g_vehicle_solution_list
    global g_path_solution_list

    with open('GAMS_vehicle_path_solution.csv', 'r') as fp:
        lines = fp.readlines()
        for l in lines[1:]:
            l = l.strip().split(',')
            g_vehicle_solution_list.append(l[0])
            g_path_solution_list.append(l[1])

    
    with open("output_GAMS_solution_agent.csv", "w") as final_output:   
        for i in range (0,len(g_vehicle_solution_list)):
            for j in range(0,len(g_vehicle_list)):
                # the '"1"' cannot be compared with '1', even the both is string
                if(eval(g_vehicle_solution_list[i])==g_internal_vehicle_list[j] and eval(g_path_solution_list[i])==g_Path_No_list[j]):
                    final_output.write("%s,%s,%s,%s,%s,%s,%s,%s,%s\n" % (g_internal_vehicle_list[j],g_vehicle_list[j],g_Path_No_list[j],g_Path_Node_seq_list[j],
                    g_Path_Node_time_seq_list[j],g_Path_Node_state_seq_list[j],g_Path_link_seq_list[j],g_Path_cost_list[j],
                    g_Passenger_served_list[j]))
                    
if __name__=='__main__':
    start_time = timeit.default_timer()

    iteration_number=4
    current_number = 2
    # obtain the initial feasble solution
    # generate the initial solution of sub-problems
    os.system(r'"C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\Initial_Sub-problems_VRP.exe"')
    g_output_initial_agent_VRP_solution()
    # generate the marginal values
    ws_INI = GamsWorkspace()
    master_problem_INI = ws_INI.add_job_from_file("C:\\Users\\jliu215\\Desktop\\DW-Interaction\\Presentation_DOT\\SF_DW_30vp\\Vehicle assignment_master.gms")
    master_problem_INI.run()

    # iteration by itetation to imporve the solution
    for i in range(2,iteration_number+1):
        g_output_ite_num_VRP(i,iteration_number)
        print("iteration_number:%d" % i)

       # generate the new solution of sub-problems
        os.system(r'"C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\Sub-problems_VRP.exe"')

        g_UpdateNetworkData()
        g_output_GAMS_inputs()
        g_output_agent_VRP_solution()

        # generate the marginal values of master problem
        ws = GamsWorkspace()
        master_problem = ws.add_job_from_file("C:\\Users\\jliu215\\Desktop\\DW-Interaction\\Presentation_DOT\\SF_DW_30vp\\Vehicle assignment_master.gms")
        master_problem.run()
        if(i==iteration_number):
            g_output_GAMS_solution()

    elapsed = timeit.default_timer() - start_time
    print(elapsed)
    print("well done!")