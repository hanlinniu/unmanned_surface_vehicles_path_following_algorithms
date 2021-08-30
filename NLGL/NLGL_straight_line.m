clear,clc,close all

% Initialize the USV
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
R = 15;

% Initialize the plot
xlim([-200 1200])
ylim([-200 1200])
hold on
xlabel('x')
ylabel('y')
hold on
grid on
title('NLGL Straight Line Following Demo')

% The mission loop
for i = 1:1000
    x_usv = x_usv + v_usv * cos(h_usv) * delta_t;
    y_usv = y_usv + v_usv * sin(h_usv) * delta_t;
    
    %% NLGL straight line guidance
    theta_s_f = (destination_y - startpoint_y)/(destination_x - startpoint_x);
    intercept = (destination_y + startpoint_y - theta_s_f*destination_x - theta_s_f*startpoint_x)/2;
    [xq,yq] = linecirc(theta_s_f, intercept, x_usv, y_usv, R);
    q1 = [xq(1), yq(1)];
    q2 = [xq(2), yq(2)];
    
    % Select the VTP
    Selectvector = [norm([destination_x - xq(1) destination_y - yq(1)]) norm([destination_x - xq(2) destination_y - yq(2)])];
    [dist, I] = min(Selectvector);
    
    if I == 1
        VTP = q1;
    elseif I == 2
        VTP = q2;
    end
    
    theta = atan2(VTP(2) - y_usv, VTP(1) - x_usv);
    if theta < 0
        theta = theta + 2*pi;
    end
    h_usv = theta;
    
    
    % Plot the usv
    [x_usv_plot, y_usv_plot, H, G] = draw_kayaka(x_shape,y_shape,x_usv,y_usv,scale, h_usv);
    pause(0.1)
    delete(H)
    delete(G)
    i = i+1;
    
end
