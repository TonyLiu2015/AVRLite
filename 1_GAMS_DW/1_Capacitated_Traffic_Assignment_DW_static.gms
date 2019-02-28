$ontext
Multi-commodity flow problem: Dantzig-Wolfe Decomposition
Erwin Kalvelagen and Jiangtao Liu
Reference:
      http://www.gams.com/~erwin/dw/dw.pdf
$offtext

set i nodes /1*4/;
set a OD pair /1*1/;
alias (i, j);

parameter c(i,j) link travel cost /
1.2 3
1.3 1
3.2 1
2.4 1
3.4 4
1.4 7
/;

parameter cap(i,j) link capacity /
1.2 1
1.3 1
3.2 1
2.4 1
3.4 1
1.4 1
/;

parameter origin(a,i) /
1.1 2
/;

parameter destination(a,i) /
1.4 2
/;

parameter intermediate(a,i);
intermediate(a,i) = (1- origin(a,i))* (1- destination(a,i));

*-----------------------------------------------------------------------
* direct LP formulation
*-----------------------------------------------------------------------

variable z;
positive variables
x(a,i,j)  selection of  comm a between i and j;

equations
obj_t
comm_flow_on_node_origin(a,i)         origin node flow
comm_flow_on_node_intermediate(a,i)   intermediate node flow
comm_flow_on_node_destination(a,i)      destination node flow
cap_constraint(i,j)
;

obj_t..  z =e= sum((a,i,j),c(i,j)*x(a,i,j));
comm_flow_on_node_origin(a,i)$(origin(a,i)>0.1).. sum(j$(c(i,j)>0.1), x(a,i,j)) =e= origin(a,i);
comm_flow_on_node_intermediate(a,i)$(intermediate(a,i)>0.1).. sum(j$(c(i,j)>0.1), x(a,i,j))-sum(j$(c(j,i)>0.1), x(a,j,i))=e= 0;
comm_flow_on_node_destination(a,i)$(destination(a,i)>0.1) ..  sum(j$(c(j,i)>0.1), x(a,j,i))=e= destination(a,i);
cap_constraint(i,j)$(c(i,j)>0.1).. Cap(i,j)-sum(a,x(a,i,j)) =g=0;

model primal_model/obj_t,comm_flow_on_node_origin,comm_flow_on_node_intermediate,comm_flow_on_node_destination,cap_constraint/;
solve primal_model minimizing z using lp;
display z.l;

*-----------------------------------------------------------------------
* subproblems
*-----------------------------------------------------------------------

positive variables xsub(i,j);
variables zsub;

parameters
   origin_sub(i)
   destination_sub(i)
   intermediate_sub(i)
   pi1(i,j) 'dual of limit'
   pi2(a)   'dual of convexity constraint'
   pi2a
;

equations
comm_flow_on_node_origin_sub(i)         origin node flow
comm_flow_on_node_intermediate_sub(i)   intermediate node flow
comm_flow_on_node_destination_sub(i)      destination node flow
rc1_sub          'phase 1 objective'
rc2_sub          'phase 2 objective'
;

comm_flow_on_node_origin_sub(i)$(origin_sub(i)>0.1).. sum(j$(c(i,j)>0.1),xsub(i,j)) =e= origin_sub(i);
comm_flow_on_node_intermediate_sub(i)$(intermediate_sub(i)>0.1).. sum(j$(c(i,j)>0.1), xsub(i,j))-sum(j$(c(j,i)>0.1), xsub(j,i))=e= 0;
comm_flow_on_node_destination_sub(i)$(destination_sub(i)>0.1) ..  sum(j$(c(j,i)>0.1), xsub(j,i))=e= destination_sub(i);
rc1_sub..       zsub =e= sum((i,j), -pi1(i,j)*xsub(i,j)) - pi2a;
rc2_sub..       zsub =e= sum((i,j), (c(i,j)-pi1(i,j))*xsub(i,j)) - pi2a;

model sub1 'phase 1 subproblem' /comm_flow_on_node_origin_sub, comm_flow_on_node_intermediate_sub,comm_flow_on_node_destination_sub, rc1_sub/;
model sub2 'phase 2 subproblem' /comm_flow_on_node_origin_sub, comm_flow_on_node_intermediate_sub,comm_flow_on_node_destination_sub, rc2_sub/;

*-----------------------------------------------------------------------
* master problem
*-----------------------------------------------------------------------

set k 'proposal count' /proposal1*proposal10/;
set ak(a,k);
ak(a,k) = no;

parameter proposal(i,j,a,k);
parameter proposalcost(a,k);
proposal(i,j,a,k) = 0;
proposalcost(a,k) = 0;


positive variables
   lambda(a,k)
   excess   'artificial variable'
;
variable zmaster;

equations
    obj1_master    'phase 1 objective'
    obj2_master    'phase 2 objective'
    capacity_limit_master(i,j)
    convex_master
;

obj1_master..  zmaster =e= excess;
obj2_master..  zmaster =e= sum(ak, proposalcost(ak)*lambda(ak));

capacity_limit_master(i,j)..
   sum(ak, proposal(i,j,ak)*lambda(ak)) =l= cap(i,j) + excess;

convex_master(a).. sum(ak(a,k), lambda(a,k)) =e= 1;

model master1 'phase 1 master' /obj1_master, capacity_limit_master, convex_master/;
model master2 'phase 2 master' /obj2_master, capacity_limit_master, convex_master/;

*-----------------------------------------------------------------------
* options to reduce solver output
*-----------------------------------------------------------------------

option limrow=0;
option limcol=0;

master1.solprint = 2;
master2.solprint = 2;

sub1.solprint = 2;
sub2.solprint = 2;

*-----------------------------------------------------------------------
* options to speed up solver execution
*-----------------------------------------------------------------------

master1.solvelink = 2;
master2.solvelink = 2;
sub1.solvelink = 2;
sub2.solvelink = 2;

*-----------------------------------------------------------------------
* DANTZIG-WOLFE INITIALIZATION PHASE
*    test subproblems for feasibility
*    create initial set of proposals
*-----------------------------------------------------------------------

display "-----------------------------------------------------------------",
        "INITIALIZATION PHASE",
        "-----------------------------------------------------------------";

set kk(k) 'current proposal';
kk('proposal1') = yes;

loop(a,

*
* solve subproblem, check feasibility
*
    c(i,j) = c(i,j);
    origin_sub(i) = origin(a,i);
    destination_sub(i) = destination(a,i);
    intermediate_sub(i) = (1- origin_sub(i))* (1- destination_sub(i));
    pi1(i,j) = 0;
    pi2a = 0;
    solve sub2 using lp minimizing zsub;
    abort$(sub2.modelstat = 4) "SUBPROBLEM IS INFEASIBLE: ORIGINAL MODEL IS INFEASIBLE";
    abort$(sub2.modelstat <> 1) "SUBPROBLEM NOT SOLVED TO OPTIMALITY";

*
* proposal generation
*
    proposal(i,j,a,kk) = xsub.l(i,j);
    proposalcost(a,kk) = sum((i,j), c(i,j)*xsub.l(i,j));
    ak(a,kk) = yes;
    kk(k) = kk(k-1);

);

option proposal:2:2:2;
display proposal;

*-----------------------------------------------------------------------
* DANTZIG-WOLFE ALGORITHM
*   while (true) do
*      solve restricted master
*      solve subproblems
*   until no more proposals
*-----------------------------------------------------------------------

set iter 'maximum iterations' /iter1*iter15/;
scalar done /0/;
scalar count /0/;
scalar phase /1/;
scalar iteration;

loop(iter$(not done),

    iteration = ord(iter);
    display "-----------------------------------------------------------------",
            iteration,
            "-----------------------------------------------------------------";

*
* solve master problem to get duals
*
    if (phase=1,
        solve master1 minimizing zmaster using lp;
        display lambda.l;

        abort$(master1.modelstat <> 1) "MASTERPROBLEM NOT SOLVED TO OPTIMALITY";
        if (excess.l < 0.0001,
           display "Switching to phase 2";
           phase = 2;
           excess.fx = 0;
        );

    );

    if (phase=2,
        solve master2 minimizing zmaster using lp;
        display lambda.l;
        abort$(master2.modelstat <> 1) "MASTERPROBLEM NOT SOLVED TO OPTIMALITY";
    );

    pi1(i,j) = capacity_limit_master.m(i,j);
    pi2(a) = convex_master.m(a);
    display pi1,pi2;

    count = 0;
    loop(a$(not done),

*
* solve each subproblem
*

        c(i,j) = c(i,j);
        origin_sub(i) = origin(a,i);
        destination_sub(i) = destination(a,i);
        intermediate_sub(i) = (1- origin_sub(i))* (1- destination_sub(i));
        pi2a = pi2(a);

        if (phase=1,
           solve sub1 using lp minimizing zsub;
           abort$(sub1.modelstat = 4) "SUBPROBLEM IS INFEASIBLE: ORIGINAL MODEL IS INFEASIBLE";
           abort$(sub1.modelstat <> 1) "SUBPROBLEM NOT SOLVED TO OPTIMALITY";
        else
           solve sub2 using lp minimizing zsub;
           abort$(sub2.modelstat = 4) "SUBPROBLEM IS INFEASIBLE: ORIGINAL MODEL IS INFEASIBLE";
           abort$(sub2.modelstat <> 1) "SUBPROBLEM NOT SOLVED TO OPTIMALITY";
        );


*
* proposal
*
        if (zsub.l < -0.0001,
           count = count + 1;
           display "new proposal", count,xsub.l;
           proposal(i,j,a,kk) = xsub.l(i,j);
           proposalcost(a,kk) = sum((i,j), c(i,j)*xsub.l(i,j));
           ak(a,kk) = yes;
           kk(k) = kk(k-1);
       );

    );

*
* no new proposals?
*
   abort$(count = 0 and phase = 1) "PROBLEM IS INFEASIBLE";
   done$(count = 0 and phase = 2) = 1;
);

abort$(not done) "Out of iterations";

*-----------------------------------------------------------------------
* recover solution
*-----------------------------------------------------------------------

parameter xsol(i,j,a);
xsol(i,j,a) = sum(ak(a,k), proposal(i,j,ak)*lambda.l(ak));
display xsol;

parameter totalcost;
totalcost = sum((i,j,a), c(i,j)*xsol(i,j,a));
display totalcost;
