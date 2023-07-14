% Version 2 Eliminando stop & acción de success
%%%%%%%%%%%%%%%% 
% State variables 

state_fluent(f_N).
state_fluent(f_NW).
state_fluent(f_W).
state_fluent(sW1).
state_fluent(sW2).

%%%%%%%%%%%%%%%% 
% Actions 

action(cruise).
action(keep_distance).
action(change_lane).

%%%%%%%%%%%%%%%% 
% Utilities 

utility(f_N(0), 0.2). 
utility(keep_distance, -0.3). 
utility(change_lane, -0.5). 

%%%%%%%%%%%%%%%% 
% Reward model


%%%%%%%%%%%%%%%% 
% Actions

% change_lane

0.50::f_N(1) :- f_N(0), change_lane.
0.9::f_N(1) :- not(f_N(0)), change_lane.
0.5::f_NW(1) :- f_NW(0), change_lane.
0.1::f_NW(1) :- not(f_NW(0)), change_lane.
0.5::f_W(1) :- f_W(0), change_lane.
0.1::f_W(1) :- not(f_W(0)), change_lane.
0.5::sW1(1) :- sW1(0), change_lane.
0.5::sW2(1) :- not(sW1(0)), change_lane.

% Keep distance

0.5::f_N(1) :- f_N(0), keep_distance.
0.5::f_N(1) :- not(f_N(0)), keep_distance.

1.0::sW1(1) :- sW1(0), keep_distance.
0.0::sW1(1) :- not(sW1(0)), keep_distance.
1.0::sW2(1) :- sW2(0), keep_distance.
0.0::sW2(1) :- not(sW2(0)), keep_distance.

% Steady motion

0.9::f_N(1) :- f_N(0), cruise.
0.0::f_N(1) :- not(f_N(0)), cruise.
0.5::f_NW(1) :- cruise.
0.5::f_W(1) :- cruise.

1.0::sW1(1) :- (sW1(0); sW2(0)), cruise.
0.5::sW1(1) :- (not(sW1(0)); not(sW2(0))), cruise.




