% Mahyar Abdeetedal
% Download RoboticToolBox from http://www.petercorke.com/robot
% >> addpath ROBOTDIR/simulink
% To bring up the block library
% >> roblocks

L1 = link([0 1 0 0],'standard')
L2 = link([0 1 0 0],'standard')
r = robot({L1,L2})
r.name = 'SCARA';
plot(r,[1,1]);

t = [0:1:99]';
q1 =sV(1,:)'; %actual state
q2 =sV(2,:)'; 
actual.time = t;
actual.signals.values = [q1,q2];
actual.signals.dimensions =2;

q1e = xV(1,:)'; %estimate
q2e = xV(2,:)';
estimate.time = t;
estimate.signals.values = [q1e,q2e];
estimate.signals.dimensions =2;

