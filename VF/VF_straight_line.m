clear,clc,close all

% Initialize the UAV
x_usv = 200;
y_usv = 0;
v_usv = 4;
h_usv = pi/2;
delta_t = 1;

% Define vessel shape
x_shape = [0,0.3,0.7,1,0.5,0,-0.5,-1,-0.7,-0.3,0];
y_shape = [6,4.6,2,0,-1.2,-2,-1.2,0,2,4.6,6];
scale = 8;
[m_shape, n_shape] = size(x_shape);

% Initialize the straight line path
startpoint_x = 0;
startpoint_y = 0;
destination_x = 1000;
destination_y = 1000;
plot([startpoint_x destination_x], [startpoint_y destination_y],'b')
hold on

% Set the guidance algorithm parameter
Xi_e = pi/2;
tau = 35;
k = 1;
alpha = 20;

% Initialize the plot
xlim([-200 1200])
ylim([-200 1200])
hold on
xlabel('x')
ylabel('y')
hold on
grid on
title('VF Straight Line Following Demo')

% The mission loop
for i = 1:1000
    x_usv = x_usv + v_usv * cos(h_usv) * delta_t;
    y_usv = y_usv + v_usv * sin(h_usv) * delta_t;
    
    %% VF strait line
    
    theta = atan2(destination_y - startpoint_y, destination_x - startpoint_x);


    p = [x_usv y_usv];
    Wpt1 = [startpoint_x startpoint_y];
    Wpt2 = [destination_x destination_y];


    s = (p - Wpt1)*(Wpt2 - Wpt1)'/norm(Wpt2 - Wpt1)^2;

    epsilon = norm(p - (s*(Wpt2 - Wpt1) + Wpt1));

    v1 = Wpt2-Wpt1;
    v1 = [v1 1];
    v2 = p - Wpt1;
    v2 = [v2 1];
    v3 = cross(v1,v2);
    rho = sign(v3(3));

    epsilon = rho*epsilon;

    if abs(epsilon) > tau
            psi_d = theta - rho*Xi_e;
            psi_c = psi_d;    
    else
            psi_d = theta - Xi_e*(epsilon/tau)^k;
            psi_c = psi_d - (k*Xi_e*v_usv/(alpha*tau^k))*epsilon^(k-1)*sin(h_usv);
    end
    
    h_usv = psi_c;
    
    % Plot the usv
    [x_usv_plot, y_usv_plot, H, G] = draw_kayaka(x_shape,y_shape,x_usv,y_usv,scale, h_usv);
    pause(0.03)
    delete(H)
    delete(G)
end
