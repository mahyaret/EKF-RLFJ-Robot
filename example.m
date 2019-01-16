close all
clear all
clc

n=4;      %number of state
q=0.1;    %std of process 
r=0.1;    %std of measurement
Q=q^2*eye(n); % covariance of process
R=r^2*eye(4);        % covariance of measurement  

T=[1;1];

f=@(x)[x(3);x(4);(-0.44*T(1) + 0.44*T(2) + 0.42*T(2)*cos(x(2))-0.36*sin(x(2))*x(4)*x(3)-0.18*sin(x(2))*x(3)^2-0.17*sin(x(2))*x(3)^2*cos(x(2))-0.18*sin(x(2))*x(4)^2)/(-1.26 + 0.18 * cos(x(2)) ^ 2);(0.44*T(1)+0.42*T(1)*cos(x(2))-0.32*T(2)-0.85*T(2)*cos(x(2))+0.36*sin(x(2))*x(4)*x(3)+0.35*x(3)*sin(x(2))*x(4)*cos(x(2))+1.37*sin(x(2))*x(3)^2+0.35*sin(x(2))*x(3)^2*cos(x(2))+0.18*sin(x(2))*x(4)^2+ 0.17*sin(x(2))*x(4)^2*cos(x(2)))/(-1.26 + 0.18 * cos(x(2)) ^ 2)];  % nonlinear state equations
h=@(x)[x(1);x(2);x(3);x(4)];                               % measurement equation
s=[1;1;0;0];                                % initial state
x=s+q*randn(4,1); %initial state          % initial state with noise
P = eye(n);                               % initial state covraiance
N=100;                                     % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual
zV = zeros(4,N);
for k=1:N
  z = h(s) + r*randn(4,1);                     % measurments
  sV(:,k)= s;                             % save actual state
  zV(:,k)  = z;                             % save measurment
  [x, P] = ekf(f,x,P,h,z,Q,R);            % ekf 
  xV(:,k) = x;                            % save estimate
  s = f(s) + q*randn(4,1);                % update process 
end
                                % plot results
figure
plot(1:N, sV(1,:), '-', 1:N, xV(1,:), '--')
title('q_1');
xlabel('Time');
ylabel('q_1');
legend('Actual State','Estimated State');

figure
plot(1:N, sV(2,:), '-', 1:N, xV(2,:), '--')
title('q_2');
xlabel('Time');
ylabel('q_2');
legend('Actual State','Estimated State');

figure
plot(1:N, sV(3,:), '-', 1:N, xV(3,:), '--')
title('qdot_1');
xlabel('Time');
ylabel('qdot_1');
legend('Actual State','Estimated State');

figure
plot(1:N, sV(4,:), '-', 1:N, xV(4,:), '--')
title('qdot_2');
xlabel('Time');
ylabel('qdot_2');
legend('Actual State','Estimated State');


figure
plot(1:N, 10*log10(sV(1,:)-xV(1,:)))
title('Error q_1');
xlabel('Time');
ylabel('dB');

figure
plot(1:N, 10*log10(sV(2,:)-xV(2,:)))
title('Error q_2');
xlabel('Time');
ylabel('dB');

figure
plot(1:N, 10*log10(sV(3,:)-xV(3,:)))
title('Error qdot_1');
xlabel('Time');
ylabel('dB');


figure
plot(1:N, 10*log10(sV(4,:)-xV(4,:)))
title('Error qdot_2');
xlabel('Time');
ylabel('dB');

