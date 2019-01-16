n=4;      %number of state
q=0.1;    %std of process 
r=0.1;    %std of measurement
Q=q^2*eye(n); % covariance of process
R=r^2*eye(2);        % covariance of measurement  

T=[1;1];

f=@(x)[x(3);x(4);(-0.44*T(1) + 0.44*T(2) + 0.42*T(2)*cos(x(2))-0.36*sin(x(2))*x(4)*x(3)-0.18*sin(x(2))*x(3)^2-0.17*sin(x(2))*x(3)^2*cos(x(2))-0.18*sin(x(2))*x(4)^2)/(-1.26 + 0.18 * cos(x(2)) ^ 2);(0.44*T(1)+0.42*T(1)*cos(x(2))-0.32*T(2)-0.85*T(2)*cos(x(2))+0.36*sin(x(2))*x(4)*x(3)+0.35*x(3)*sin(x(2))*x(4)*cos(x(2))+1.37*sin(x(2))*x(3)^2+0.35*sin(x(2))*x(3)^2*cos(x(2))+0.18*sin(x(2))*x(4)^2+ 0.17*sin(x(2))*x(4)^2*cos(x(2)))/(-1.26 + 0.18 * cos(x(2)) ^ 2)];  % nonlinear state equations
h=@(x)[x(1);x(2)];                               % measurement equation
s=[0;0;1;1];                                % initial state
x=s+q*randn(4,1); %initial state          % initial state with noise
P = eye(n);                               % initial state covraiance
N=20;                                     % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual
zV = zeros(2,N);
for k=1:N
  z = h(s) + r*randn(2,1);                     % measurments
  sV(:,k)= s;                             % save actual state
  zV(:,k)  = z;                             % save measurment
  [x, P] = ekf(f,x,P,h,z,Q,R);            % ekf 
  xV(:,k) = x;                            % save estimate
  s = f(s) + q*randn(4,1);                % update process 
end
for k=1:4                                 % plot results
  subplot(4,1,k)
  plot(1:N, sV(k,:), '-', 1:N, xV(k,:), '--')
end