% Ejecutar: mdp-problog solve -m mdp_left.pl dummy.pl
% Sí funciona.
%%%%%%%%%%%%%%%%%%%%%%%%%% 
% State variables 
%%%%%%%%%%%%%%%%%%%%%%%%%%

state_fluent(free_N).
state_fluent(free_NE).
state_fluent(free_E).
state_fluent(sE1).
state_fluent(sE2).

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Actions 
%%%%%%%%%%%%%%%%%%%%%%%%%%

action(cruise).
action(keep_distance).
action(change_lane).

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Utilities 
%%%%%%%%%%%%%%%%%%%%%%%%%%

utility(free_N(1), 1).
%utility(free_NE(1), 2).
utility(return_right, 2).
utility(rearEnd_crash, -3.0).  
utility(sideSwipe_crash, -2.0).  
utility(keep_distance, -1.0). 
utility(change_lane, -2.0). 

0.99::rearEnd_crash :- \+ free_N(0), (cruise; \+ keep_distance).
0.99::rearEnd_crash :- \+ free_NE(0), change_lane.
0.99::rearEnd_crash :- \+ sE1(0), \+ sE2(0), change_lane.
0.95::sideSwipe_crash :- \+ free_E(0), change_lane.

0.95::return_right :- free_NE(0), free_E(0), sE1(0), \+ sE2(0), change_lane.
0.95::return_right :- free_NE(0), free_E(0), sE2(0), change_lane.

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Transitions 
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Action: cruise
0.96::free_N(1) :- free_N(0), cruise.
0.05::free_N(1) :- \+ free_N(0), cruise.

% Action: change_lane
0.99::free_N(1) :- free_NE(0), free_E(0), change_lane.
0.05::free_N(1) :- \+ free_NE(0), change_lane.
0.01::free_N(1) :- \+ free_E(0); (\+ sE1(0), \+ sE2(0)), change_lane.

0.99::free_NE(1) :- free_E(0), sE1(0), \+ sE2(0), change_lane.
0.99::free_NE(1) :- free_E(0), sE2(0), change_lane.

0.99::sE1(1) :- \+ sE1(0), \+ sE2(0), change_lane.
0.99::sE2(1) :- \+ sE1(0), \+ sE2(0), change_lane.

% Action: keep_distance
0.5::free_N(1) :- \+ free_N(0), (\+ free_E(0); \+ free_NE(0)), keep_distance.

%Code:
%1  1  No sE car
%1  0  sE car has slower speed
%0  1  sE car has equal speed
%0  0  sE car has faster speed
