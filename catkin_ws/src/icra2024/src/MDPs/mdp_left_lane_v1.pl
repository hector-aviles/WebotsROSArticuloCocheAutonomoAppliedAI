% Version 1
%%%%%%%%%%%%%%%% 
% State variables 

state_fluent(success).
state_fluent(f_N).
state_fluent(f_NE).
state_fluent(f_E).
state_fluent(sE1).
state_fluent(sE2).

%%%%%%%%%%%%%%%% 
% Actions 

action(cruise).
action(keep_distance).
action(change_lane).
action(stop).

%%%%%%%%%%%%%%%% 
% Utilities 

utility(success(0), 1).
#utility(stop_car, 1).
utility(f_N(0), 0.2). 
utility(keep_distance, -0.3). 
utility(change_lane, -0.5). 

%%%%%%%%%%%%%%%% 
% Reward model

success(0) :- not(rearEnd_crash(0)), not(sideSwipe_crash(0)).
rearEnd_crash(0) :- (not(f_N(0)), cruise, not(keep_distance), not(change_lane));
                    (not(f_NW(0)), change_lane).
sideSwipe_crash(0) :- (not(sW1(0)), not(sW1(0)), change_lane); 
                      (not(f_W(0)), change_lane).
#stop_car :- not(success(0)), stop.

%%%%%%%%%%%%%%%% 
% Actions

% Stop

1.0::success(1) :- not(success(0)), stop.
0.0::success(1) :- success(0), stop.

1.0::f_N(1) :- f_N(0), stop.
1.0::f_NW(1) :- f_NW(0), stop.
1.0::f_W(1) :- f_W(0), stop.

1.0::sW1(1) :- sW1(0), stop.
0.0::sW1(1) :- not(sW1(0)), stop.
1.0::sW2(1) :- sW2(0), stop.
0.0::sW2(1) :- not(sW2(0)), stop.

% change_lane

0.99::success(1) :- success(0), f_NW(0), f_W(0), (sW1(0); (not(sW1(0)), sW2(0))), change_lane.
0.5::success(1) :- success(0), f_NW(0), f_W(0), not(sW1(0)), not(sW2(0)), change_lane.
0.05::success(1) :- success(0), not(f_NW(0)), change_lane.
0.01::success(1) :- success(0), not(f_N(0)), not(f_NW(0)), not(f_W(0)), change_lane.
0.0::success(1) :- not(success(0)), change_lane.

0.50::f_N(1) :- f_N(0), change_lane.
0.9::f_N(1) :- not(f_N(0)), change_lane.
0.5::f_NW(1) :- f_NW(0), change_lane.
0.1::f_NW(1) :- not(f_NW(0)), change_lane.
0.5::f_W(1) :- f_W(0), change_lane.
0.1::f_W(1) :- not(f_W(0)), change_lane.
0.5::sW1(1) :- sW1(0), change_lane.
0.5::sW2(1) :- not(sW1(0)), change_lane.

% Keep distance

0.99::success(1) :- success(0),  keep_distance.
0.0::success(1) :- not(success(0)), keep_distance.

0.5::f_N(1) :- f_N(0), keep_distance.
0.5::f_N(1) :- not(f_N(0)), keep_distance.

1.0::sW1(1) :- sW1(0), keep_distance.
0.0::sW1(1) :- not(sW1(0)), keep_distance.
1.0::sW2(1) :- sW2(0), keep_distance.
0.0::sW2(1) :- not(sW2(0)), keep_distance.

% Steady motion

0.999::success(1) :- success(0), f_N(0), cruise.
0.3::success(1) :- success(0), not(f_N(0)),  cruise.
0.0::success(1) :- not(success(0)), cruise.

0.9::f_N(1) :- f_N(0), cruise.
0.0::f_N(1) :- not(f_N(0)), cruise.
0.5::f_NW(1) :- cruise.
0.5::f_W(1) :- cruise.

1.0::sW1(1) :- (sW1(0); sW2(0)), cruise.
0.5::sW1(1) :- (not(sW1(0)); not(sW2(0))), cruise.




