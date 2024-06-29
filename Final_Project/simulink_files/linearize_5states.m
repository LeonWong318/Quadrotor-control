clc;
clear;

load FlightData.mat;
ctrl=sym("ctrl",[4,1]); %Thrust:left

pos = sym("p",[3,1]);   %Position:right
pos_dot= sym("p",[3,1]); 
v   = sym("v",[3,1]);   %Velocity:right
v_dot   = sym("v",[3,1]);
a   = sym("a",[3,1]); %Acceleration:right

syms phi theta psi phi_dot theta_dot psi_dot phi_dot_dot theta_dot_dot psi_dot_dot d_phi d_theta d_psi

angle = [phi; theta; psi];   %Angle:right
d_angle=[d_phi; d_theta; d_psi];
w     = [phi_dot theta_dot psi_dot]; %Angular velocity:right
w_dot = [phi_dot_dot theta_dot_dot psi_dot_dot]; % Angular acceleration


m = 0.027;       %Mass
g = 9.81;     %Gravity
d = 0.046;        %Arm length
lift = 1.9796e-9; %Lift constant
drag = 2.5383e-11;%Drag constant
F = 0.25;      %Air friction coefficient



%Moment of inertia matrix
Jx = 1.1463e-5;
Jy = 1.6993e-5;
Jz = 2.9944e-5;
J = [Jx,0,0;0,Jy,0;0,0,Jz];

w_b= sym('wb',[3,1]);
w_b_dot= sym('wb_dot',[3,1]);

% Add time invariant parameters here 


%%


%sum all forces:

xrot=[1   0          0    ;
       0 cos(phi) -sin(phi);
       0 sin(phi) cos(phi)];

yrot=[cos(theta)  0 sin(theta) ;
        0           1       0   ;
       -sin(theta) 0 cos(theta)];

zrot=[cos(psi) -sin(psi) 0;
        sin(psi) cos(psi) 0;
           0       0      1];

B2W=zrot*yrot*xrot;
 

rotor_speed=sqrt(abs(ctrl)/lift);


%torque expression


Tx=d*cos(pi/4)*(ctrl(3)+ctrl(4)-ctrl(1)-ctrl(2));
Ty=d*cos(pi/4)*(-ctrl(1)-ctrl(4)+ctrl(3)+ctrl(2));
Tz=drag*(-rotor_speed(1)^2-rotor_speed(3)^2+rotor_speed(2)^2+rotor_speed(4)^2);
T_b=[Tx;Ty;Tz];


w_angle2w=[1  0                sin(theta);
            0 cos(phi) -sin(phi)*cos(theta);
            0 sin(phi) cos(phi)*cos(theta)];
%%
eq2= m*v_dot == [0;0;0];
eq1=pos_dot==v;
eq4=J*w_b_dot==-cross(w_b, J*w_b)+T_b;
eq3= d_angle==w_angle2w\w_b;
          
%%    
equations=[eq1;
           eq2;
           eq3;
           eq4];

%%
x    = [pos;   v;       angle;w_b;];

xdot = [pos_dot; v_dot;  d_angle;w_b_dot];

u    = ctrl;

eqs = lhs(equations) - rhs(equations);
A = -jacobian(eqs, xdot) \ jacobian(eqs, x);
B = -jacobian(eqs, xdot) \ jacobian(eqs, u);

x_lin = zeros(12,1);

% inputs = [ ctrl_1, ctrl_2, ctrl_3, ctrl_4].';
u_lin = repmat(0.027*g/4, [4 1]);
x0= [0 0 0  0 0 0  0 0 0 0 0 0]';

An = [0,0,1,0,0;
    0,0,0,1,0;
    zeros(3,5)];
Bn = double(subs(B, [x;ctrl] , [x_lin; u_lin] ));
Bn = Bn(8:12,:);
Cn = eye(5);
Dn = zeros(5,4);

%% LQR gain
Q = diag([1000,1000,100,100,10]);

R = eye(4)*0.01;

k = lqr(An,Bn,Q,R);


% Sampling time
%Ts = 0.01; % Example sampling time
%sysc = ss(An, Bn, Cn, Dn);
% Convert to discrete-time using zero-order hold method
%sysd = c2d(sysc, Ts);

%k = dlqr(sysd.A,sysd.B,Q,R);

% H = [1,0,0,0,0;
%     0,1,0,0,0;
%     0,0,0,0,1];%dimension selection;
% kr = -(H* inv(An-Bn*k) *Bn)';

    