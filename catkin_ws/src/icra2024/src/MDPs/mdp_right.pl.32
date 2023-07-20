% Ejecutar: mdp-problog solve -m mdp_right.pl dummy.pl
% Es la versión buena para la derecha
%%%%%%%%%%%%%%%%%%%%%%%%%% 
% State variables 
%%%%%%%%%%%%%%%%%%%%%%%%%%

state_fluent(free_N).
state_fluent(free_NW).
state_fluent(free_W).
state_fluent(sW1).
state_fluent(sW2).

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
utility(rearEnd_crash(1), -3.0).  
utility(sideSwipe_crash(1), -2.0).  
utility(keep_distance, -1.0). 
utility(change_lane, -2.0). 

0.99::rearEnd_crash(1) :- \+ free_N(0), cruise, \+ keep_distance.
0.99::rearEnd_crash(1) :- \+ free_NW(0), change_lane.
0.99::rearEnd_crash(1) :- \+ sW1(0), \+ sW2(0), change_lane.
0.95::sideSwipe_crash(1) :- \+ free_W(0), change_lane.

%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Transitions 
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Action: cruise
0.96::free_N(1) :- free_N(0), cruise.
0.05::free_N(1) :- \+ free_N(0), cruise.

% Action: change_lane
0.99::free_N(1) :- free_NW(0), free_W(0), change_lane.
0.05::free_N(1) :- \+ free_NW(0), change_lane.
0.01::free_N(1) :- \+ free_W(0); (\+ sW1(0), \+ sW2(0)), change_lane.

% Moving from right to left does not leave the sW position free.
0.01::sW1(1) :- \+ sW1(0), \+ sW2(0), change_lane.
0.01::sW2(1) :- \+ sW1(0), \+ sW2(0), change_lane.

% Action: keep_distance
0.5::free_N(1) :- \+ free_N(0), (\+ free_W(0); \+ free_NW(0)), keep_distance.

%Code:
%1  1  No SW car
%1  0  SW car has slower speed
%0  1  SW car has equal speed
%0  0  SW car has faster speed
