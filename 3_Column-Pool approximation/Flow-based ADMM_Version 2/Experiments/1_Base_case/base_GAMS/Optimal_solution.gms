$ title Measurement estimation

OPTIONS  LP = CPLEX;

set i nodes /1*1146/;
set t time stamp /0*60/;
set p path /0*1790/;
set a passenger_group/1*7858/;
alias (i, j);
alias (t, s);

$include "D:\Part_C-TEST\Test_comparison\1_Base_case\base_GAMS\output_GAMS_input.txt"

variable z_obj;
positive variables
x(p) ;

equations
obj_min_p1
pax_demand(a)
cap_constraint(i,j,t,s)
;

obj_min_p1.. z_obj =e= sum(p,path_cost(p)*x(p));
pax_demand(a).. sum(p,x(p)*path_pax_ind(p,a))=e=pax_group(a);
cap_constraint(i,j,t,s).. sum(p,path_link_ind(p,i,j,t,s)*x(p)) =l=arc_cap(i,j,t,s);

Model vehilcel_assignment /all/;

solve vehilcel_assignment using LP minimizing z_obj;
display z_obj.l;
display x.l;

