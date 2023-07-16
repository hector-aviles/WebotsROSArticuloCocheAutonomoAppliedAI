% Ejecutar: mdp-problog solve -m mdp_left.pl dummy.pl
% Funciona pero hay que optimizar las reglas free_NE(1) con change_lane 
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
utility(free_NE(1), 2).
utility(rearEnd_crash(1), -3.0).  
utility(sideSwipe_crash(1), -2.0).  
utility(keep_distance, -1.0). 
utility(change_lane, -2.0). 

0.99::rearEnd_crash(1) :- \+ free_N(0), (cruise; \+ keep_distance).
0.99::rearEnd_crash(1) :- \+ free_NE(0), change_lane.
0.99::rearEnd_crash(1) :- \+ sE1(0), \+ sE2(0), change_lane.
0.95::sideSwipe_crash(1) :- \+ free_E(0), change_lane.

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

0.99::free_NE(1) :- free_E(0), \+ free_N(0), sE1(0), \+ sE2(0), change_lane.
0.99::free_NE(1) :- free_E(0), free_N(0), sE1(0), \+ sE2(0), change_lane.
0.99::free_NE(1) :- free_E(0), \+ free_N(0), \+ sE1(0), sE2(0), change_lane.
0.99::free_NE(1) :- free_E(0), free_N(0), \+ sE1(0), sE2(0), change_lane.
0.99::free_NE(1) :- free_E(0), \+free_N(0), sE1(0), sE2(0), change_lane.
0.99::free_NE(1) :- free_E(0), free_N(0), sE1(0), sE2(0), change_lane.

% Moving from right to left does not leave the sE position free.
0.01::sE1(1) :- \+ sE1(0), \+ sE2(0), change_lane.
0.01::sE2(1) :- \+ sE1(0), \+ sE2(0), change_lane.

% Action: keep_distance
0.5::free_N(1) :- \+ free_N(0), (\+ free_E(0); \+ free_NE(0)), keep_distance.

%Code:
%1  1  No SE car
%1  0  SE car has slower speed
%0  1  SE car has equal speed
%0  0  SE car has faster speed
