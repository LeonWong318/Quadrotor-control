clc;
clear;
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
 eq2= m*v_dot == B2W*[0;0; ctrl(1)+ctrl(2)+ctrl(3)+ctrl(4)]-[0; 0; m*g]-F*v;
 eq1=pos_dot==v;
 eq4=J*w_b_dot==-cross(w_b, J*w_b)+T_b;
 eq3= d_angle==w_angle2w\w_b;
              
%%    
    equations=[eq3;
               eq4];
 
%%
%%
    x    = [ angle;w_b;];

    xdot = [d_angle;w_b_dot];

u    = ctrl;
    
eqs = lhs(equations) - rhs(equations);
A = -jacobian(eqs, xdot) \ jacobian(eqs, x);
B = -jacobian(eqs, xdot) \ jacobian(eqs, u);

x_lin = zeros(6,1);

% inputs = [ ctrl_1, ctrl_2, ctrl_3, ctrl_4].';
u_lin = repmat(0.027*g/4, [4 1]);
x0= [0 0 0  0 0]';

An = double(subs(A, [x;ctrl] , [x_lin; u_lin] ));
Bn = double(subs(B, [x;ctrl] , [x_lin; u_lin] ));

An(3,:)=[];
An(:,3)=[];
Bn(3,:)=[];


Cn = [1 0 0 0 0;
      0 1 0 0 0;
      0 0 1 0 0;
      0 0 0 1 0;
      0 0 0 0 1]
Dn = zeros(5,4)
% Q=[10 0 0 0 0;
%     0 10 0 0 0;
%     0 0 100000 0 0
%     0 0 0 100000 0;
%     0 0 0 0 1000];

  % Q=[100 0 0 0 0;
  %     0 100 0 0 0;
  %     0 0 250 0 0
  %    0 0 0 250 0;
  %     0 0 0 0 100];
 Q=[1000000 0 0 0 0;
     0 1000000 0 0 0;
     0 0 100 0 0;
     0 0 0 100 0;
     0 0 0 0 10];
R=eye(4,4)*0.1;

%[K,S,CLP]=lqr(An,Bn,Q,R,[]);

Data=readmatrix("OurData.csv");


Acc_x=struct('time', linspace(0,0.01*length(Data),length(Data)), 'signals', struct('values',Data(:,2)));
Acc_y=struct('time', linspace(0,0.01*length(Data),length(Data)), 'signals', struct('values',Data(:,3)));
Acc_z=struct('time', linspace(0,0.01*length(Data),length(Data)), 'signals', struct('values',Data(:,4)));
Gyro_x=struct('time', linspace(0,0.01*length(Data),length(Data)), 'signals', struct('values',Data(:,5)));
Gyro_y=struct('time', linspace(0,0.01*length(Data),length(Data)), 'signals', struct('values',Data(:,6)));
Gyro_z=struct('time', linspace(0,0.01*length(Data),length(Data)), 'signals', struct('values',Data(:,7)));

% t=linspace(0,0.01*length(Data),length(Data));
% figure()
% plot(linspace(0,0.01*length(Data),length(Data)),Data(:,2));
% hold on 
% figure()
% plot(linspace(0,0.01*length(Data),length(Data)),Data(:,5));

%% test
% test=load("FlightData.mat");
% m=test.Acc_x.time(1:846)
% figure()
% plot(test.Acc_x.time,test.Acc_x.signals.values)
% figure()
% plot(test.Gyro_x.time,test.Gyro_x.signals.values)

%% discrete system
sysc = ss(An, Bn, Cn, Dn);

% Sampling time
Ts = 0.01; % Example sampling time

% Convert to discrete-time using zero-order hold method
sysd = c2d(sysc, Ts, 'zoh');

[K,S,CLP] = dlqr(sysd.A,sysd.B*0.06/4*g/65536/180/pi,Q,R,[])

%load("FlightData.mat")


%%    