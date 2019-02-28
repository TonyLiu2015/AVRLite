$ title Autonomous vehicle assignment

$include "C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\GAMS_input_set.txt"

$include "C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\GAMS_input_parameter.txt"

variable z_min;
binary variables
lamda(v,k)  selection of  path p by agent a;

equations
obj_min
pickup_constraint(p)
cap_constraint_new(i,j,t,s)
weights(v)
;

obj_min.. z_min =e= sum((v,k),c(v,k)*lamda(v,k));
pickup_constraint(p).. sum((v,k),lamda(v,k)*delta(v,k,p))=e=1;
cap_constraint_new(i,j,t,s)$(cap(i,j,t,s)>0.1) .. sum((v,k)$(c(v,k)>0.1),beta(v,k,i,j,t,s)*x(v,k)*lamda(v,k))=l=1000;
weights(v).. sum(k,lamda(v,k))=e=1;

Model AVs_assignment /all/;

solve AVs_assignment using MIP minimizing z_min;
display lamda.l;

FILE csv_capacity Report File /C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\GAMS_arc_cap_marg.csv/;
csv_capacity.pc = 5;
PUT csv_capacity;

set iteration/1*1/;

loop((iteration),put @5, 'from_node', @10, 'to_node', @20, 'from_time',  @30, 'to_time', @40,'marginal_value'/);
loop((i,j,t,s)$((cap(i,j,t,s) >0.1)),put @5, i.tl, @10, j.tl, @20, t.tl,  @30, s.tl, @40, cap_constraint_new.m(i,j,t,s)/);

FILE csv_passenger Report File /C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\GAMS_passenger_pickup_marg.csv/;
csv_passenger.pc = 5;
PUT csv_passenger;

loop((iteration),put @5, 'passenger', @10, 'pickup marginal_value'/);
loop((p),put @5, p.tl, @10, pickup_constraint.m(p)/);

FILE csv_vehicle Report File /C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\GAMS_vehicle_weight_marg.csv/;
csv_vehicle.pc = 5;
PUT csv_vehicle;

loop((iteration),put @5, 'vehicle', @10, 'vehicle_weight marginal_value'/);
loop((v),put @5, v.tl, @10, weights.m(v)/);

FILE csv_solution Report File /C:\Users\jliu215\Desktop\DW-Interaction\Presentation_DOT\SF_DW_30vp\GAMS_vehicle_path_solution.csv/;
csv_solution.pc = 5;
PUT csv_solution;

loop((iteration),put @5, 'vehicle', @10, 'path', @20, 'value'/);
loop((v,k),put @5, v.tl, @10, k.tl, @20,lamda.l(v,k)/);



