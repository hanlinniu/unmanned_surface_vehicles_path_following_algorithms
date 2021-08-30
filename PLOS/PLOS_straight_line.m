clear,clc,close all

% Initialize the UAV
x_usv = 10;
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
k1 = 1;
k2 = 0.1;


% Initialize the plot
xlim([-200 1200])
ylim([-200 1200])
hold on
xlabel('x')
ylabel('y')
hold on
grid on
title('PLOS Straight Line Following Demo')

% The mission loop
for i = 1:1000
    x_usv = x_usv + v_usv * cos(h_usv) * delta_t;
    y_usv = y_usv + v_usv * sin(h_usv) * delta_t;
    
    %% PLOS straight line algorithm
    theta_d = atan2(destination_y - y_usv, destination_x - x_usv)
    
    if theta_d < 0 
        theta_d = theta_d + 2*pi;
    end
    
    d = -distance(x_usv,y_usv, startpoint_x,startpoint_y,destination_x, destination_y);
    
    h_usv = k1*(theta_d-h_usv) + k2*d + h_usv;
    
    % Plot the usv
    [x_usv_plot, y_usv_plot, H, G] = draw_kayaka(x_shape,y_shape,x_usv,y_usv,scale, h_usv);
    pause(0.01)
    delete(H)
    delete(G)
end
