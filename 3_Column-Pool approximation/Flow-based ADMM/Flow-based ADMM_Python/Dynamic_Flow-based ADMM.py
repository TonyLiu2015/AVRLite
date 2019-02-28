'''
ADMM for flow-based dynamic assignment based on column pool
@ Jiangtao Liu, Arizona State University
'''
import csv
import numpy

g_agent_list=[]
g_number_of_agents = 0

g_pax_group_id_list =[]
g_pax_demand_dict ={}
g_group_demand_list= []

g_iteration_number=10
g_Obj_cost = [0.0]*(g_iteration_number+1)
g_optimal_solution=[0.0]*(g_iteration_number+1)

g_arc_capcity_dict={}
g_arc_list=[]
g_arc_cap_list=[]


class Agent:
    def __init__(self):
        self.agent_id = 0
        self.column_cost = 0.0
        self.column_flow = 0.0
        self.served_pax_group =''
        self.served_pax_group_list =[]
        self.column_node_seq =''
        self.column_node_time_seq =''
        self.column_node_list = []
        self.column_node_time_list = []
        self.column_node_arc_list = []
        self.column_node_time_arc_list = []

def g_ReadInputData():
    global g_number_of_agents 

    with open('input_agent.csv','r') as fa:
        lines = fa.readlines()   
        for l in lines[1:]:
            l = l.strip().split(',')
            agent = Agent()
            agent.agent_id = int(l[0])
            agent.column_cost =float(l[1])
            agent.served_pax_group =l[2]
            agent.column_node_seq = l[3]
            agent.column_node_time_seq = l[4]
           
            elements =agent.served_pax_group.split(';')
            for i in range(len(elements)):
                agent.served_pax_group_list.append(elements[i])
            if(agent.served_pax_group_list[-1]==''):
                del agent.served_pax_group_list[-1]

            node_elements =agent.column_node_seq.split(';')
            for i in range(0,len(node_elements)):
                agent.column_node_list.append(node_elements[i])
            if(agent.column_node_list[-1]==''):
                del agent.column_node_list[-1]

            node_time_elements=agent.column_node_time_seq.split(';')
            for i in range(0,len(node_time_elements)):
                agent.column_node_time_list.append(node_time_elements[i])
            if(agent.column_node_time_list[-1]==''):
                del agent.column_node_time_list[-1]

            for i in range(0,len(agent.column_node_list)-1):
                key_node_arc ='('+agent.column_node_list[i]+'_'+agent.column_node_list[i+1]+')'
                key_node_time_arc='('+agent.column_node_list[i]+'_'+agent.column_node_list[i+1]+'_'+agent.column_node_time_list[i]+'_'+agent.column_node_time_list[i+1]+')'
                agent.column_node_arc_list.append(key_node_arc)
                agent.column_node_time_arc_list.append(key_node_time_arc)

            g_agent_list.append(agent)
            g_number_of_agents = g_number_of_agents+1

    with open('input_passenger_demand.csv','r') as fpd:
        lines = fpd.readlines()   
        for l in lines[1:]:
            l = l.strip().split(',')
            key_pax_group_id=l[0]
            g_pax_demand_dict[key_pax_group_id]=float(l[1])
            g_pax_group_id_list.append(key_pax_group_id)
            g_group_demand_list.append(float(l[1]))

    with open('input_arc_capacity.csv','r') as fac:
        lines = fac.readlines()   
        for l in lines[1:]:
            l = l.strip().split(',')
            key_arc_cap='('+l[0]+'_'+l[1]+'_'+l[2]+'_'+l[3]+')'
            g_arc_capcity_dict[key_arc_cap]=float(l[2])
            g_arc_list.append(key_arc_cap)
            g_arc_cap_list.append(float(l[4]))

def ADMM_Newton():
    global g_optimal_solution

    roh_change_pax =[0.1]*(g_iteration_number+1)
    roh_change_arc =[0.1]*(g_iteration_number+1)

    Subgradient_pickup_pax=[0.1]*len(g_pax_group_id_list)
    Subgradient_visit_arc=[0.1]*len(g_arc_list)
        
    lambda_pax=[0.1]*len(g_pax_group_id_list)
    lambda_arc=[0.1]*len(g_arc_list)
    #delta_p=[[0 for m in range(len(g_agent_list)+1)] for k in range(len(g_pax_group_id_list)+1)]
    delta_p = numpy.zeros([len(g_agent_list),len(g_pax_group_id_list)])
    delta_arc = numpy.zeros([len(g_agent_list),len(g_arc_list)])

    roh_pax = 10
    roh_arc = 1
    
    for m in range(len(g_agent_list)):
        for k in range(len(g_pax_group_id_list)):  
            for n in range(len(g_agent_list[m].served_pax_group_list)):
                if(g_agent_list[m].served_pax_group_list[n]==g_pax_group_id_list[k]):
                    delta_p[m][k]=1

    for m in range(len(g_agent_list)):
        for r in range(len(g_arc_list)):
            for s in range(len(g_agent_list[m].column_node_time_arc_list)):
                if(g_agent_list[m].column_node_time_arc_list[s]==g_arc_list[r]):
                    delta_arc[m][r]=1

    for i in range(1,g_iteration_number+1):
        served_times_for_pax = [0]*len(g_pax_group_id_list)
        served_times_for_arc = [0]*len(g_arc_list)

        sub_cost_pax=[0.0]*len(g_agent_list)
        sub_cost_arc = [0.0]*len(g_agent_list)
        penalty_subgradient_agent_pax = [0.0]*len(g_agent_list)
        penalty_subgradient_agent_arc = [0.0]*len(g_agent_list)

        first_gradient_agent_pax = [0.0]*len(g_agent_list)
        s_coefficient_pax = [0.0]*len(g_agent_list)

        first_gradient_agent_arc = [0.0]*len(g_agent_list)
        s_coefficient_arc = [0.0]*len(g_agent_list)

        s_coefficient =[0.0]*len(g_agent_list)

        obj_1 = 0.0
        obj_2 = 0.0
        obj_3 = 0.0
        obj_4 = 0.0
        obj_5 = 0.0

        for k in range(len(g_pax_group_id_list)):
            for m in range(len(g_agent_list)):
                # x_a*delat(a,pax)
                served_times_for_pax[k]=served_times_for_pax[k]+g_agent_list[m].column_flow*delta_p[m][k]
            # sum(a,x_a*delat(a,pax))-D(pax)  
            Subgradient_pickup_pax[k]=served_times_for_pax[k]-g_group_demand_list[k]
            # update lambda for side constriants
            lambda_pax[k]=lambda_pax[k]+roh_pax*Subgradient_pickup_pax[k]

        for r in range(len(g_arc_list)):
            for m in range(len(g_agent_list)):
                # x_a*delat(a,arc(i,j,t,s))
                served_times_for_arc[r]=served_times_for_arc[r]+g_agent_list[m].column_flow*delta_arc[m][r]
            # sum(a,x_a*delat(a,arc))-cap(arc)  
            Subgradient_visit_arc[r]=served_times_for_arc[r]-g_arc_cap_list[k]
            # update lambda for side constriants
            lambda_arc[r]=max(0,lambda_arc[r]+roh_arc*Subgradient_visit_arc[r])

        for m in range(len(g_agent_list)):
            for k in range(len(g_pax_group_id_list)):
                # delta(a,pax)*[sum(a,x_a*delat(a,pax))-D(pax)]
                sub_cost_pax[m]=sub_cost_pax[m]+Subgradient_pickup_pax[k]*delta_p[m][k]
                penalty_subgradient_agent_pax[m]=penalty_subgradient_agent_pax[m]+lambda_pax[k]*delta_p[m][k]

            for r in range(len(g_arc_list)):
                # delta(a,arc)*[sum(a,x_a*delat(a,arc))-cap(arc)]
                sub_cost_arc[m]=sub_cost_arc[m]+Subgradient_visit_arc[r]*delta_arc[m][r]
                penalty_subgradient_agent_arc[m]=penalty_subgradient_agent_arc[m]+lambda_arc[r]*delta_arc[m][r]

            first_gradient_agent_pax[m]=roh_pax*sub_cost_pax[m]
            s_coefficient_pax[m]=roh_pax*len(g_agent_list[m].served_pax_group_list)

            first_gradient_agent_arc[m]=roh_arc*sub_cost_arc[m]
            s_coefficient_arc[m]=roh_arc*len(g_agent_list[m].column_node_time_arc_list)

            s_coefficient[m]=s_coefficient_pax[m]+s_coefficient_arc[m]

            g_agent_list[m].column_flow =max(0, g_agent_list[m].column_flow-(1.0/s_coefficient[m])*(g_agent_list[m].column_cost+penalty_subgradient_agent_pax[m]+first_gradient_agent_pax[m]+penalty_subgradient_agent_arc[m]+first_gradient_agent_arc[m]))

            served_times_for_pax=[0]*len(g_pax_group_id_list)
            for k in range(len(g_pax_group_id_list)):
                for m in range(len(g_agent_list)):            
                    served_times_for_pax[k]=served_times_for_pax[k]+g_agent_list[m].column_flow*delta_p[m][k]
                Subgradient_pickup_pax[k]=served_times_for_pax[k]-g_group_demand_list[k]

            served_times_for_arc=[0]*len(g_arc_list)
            for r in range(len(g_arc_list)):
                for m in range(len(g_agent_list)):            
                    served_times_for_arc[r]=served_times_for_arc[r]+g_agent_list[m].column_flow*delta_arc[m][r]
                Subgradient_visit_arc[r]=served_times_for_arc[r]-g_arc_cap_list[r]

        print('iteration:',i)
        for m in range(len(g_agent_list)):
            print('column flow:',g_agent_list[m].column_flow)

        for m in range(len(g_agent_list)):
            obj_1 = obj_1+g_agent_list[m].column_flow*g_agent_list[m].column_cost

        for k in range(len(g_pax_group_id_list)):
            obj_2 = obj_2 + lambda_pax[k]*Subgradient_pickup_pax[k]
            obj_3 = obj_3 +Subgradient_pickup_pax[k]*Subgradient_pickup_pax[k]

        for r in range(len(g_arc_list)):
            obj_4 = obj_4 + lambda_arc[r]*Subgradient_visit_arc[r]
            obj_5 = obj_5 +Subgradient_visit_arc[r]*Subgradient_visit_arc[r]

        roh_change_pax[i]=obj_3
        roh_change_arc[i]=obj_5

        if(roh_change_pax[i]>0.25*roh_change_pax[i-1]):
            roh_pax=roh_pax*1.0

        if(roh_change_arc[i]>0.25*roh_change_arc[i-1]):
            roh_arc=roh_arc*1.0

        g_optimal_solution[i]=obj_1+obj_2+obj_4
        #g_optimal_solution[i]=obj_1+obj_2+obj_3+obj_4+obj_5

        print('objective_value and iteration',g_optimal_solution[i],i)

def output_solution():
    with open("output_flow_solution.csv", "w") as output:
        output.write("column_id,column_flow\n")
        for i in range (len(g_agent_list)):
            output.write('{},{}\n'.format(i+1,g_agent_list[i].column_flow))

    with open("output_iterative_solution.csv", "w") as output:
        output.write("iteration_id,objective_value\n")
        for i in range (1,g_iteration_number+1):
            output.write('{},{}\n'.format(i,g_optimal_solution[i]))

if __name__=='__main__':
    print('Reading data......')
    g_ReadInputData()
    ADMM_Newton()
    output_solution()
    print('well done!')