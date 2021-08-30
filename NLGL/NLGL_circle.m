clear,clc,close all

% Initialize the UAV
x_usv = -10;
y_usv = -30;
v_usv = 4;
h_usv = pi/2;
delta_t = 1;

% Define vessel shape
x_shape = [0,0.3,0.7,1,0.5,0,-0.5,-1,-0.7,-0.3,0];
y_shape = [6,4.6,2,0,-1.2,-2,-1.2,0,2,4.6,6];
scale = 1;
[m_shape, n_shape] = size(x_shape);

% Initialize the straight line path
center_x = 0;
center_y = 0;
radius = 30;

theta = 0:0.01:2*pi;
circle_x = center_x + radius*cos(theta);
circle_y = center_y + radius*sin(theta);
plot(circle_x,circle_y,'.')
hold on

% Set the guidance algorithm parameter
radius_NLGL = 5;
circle_direction = 1; % -1 is clockwise, 1 is anticlock

% Initialize the plot
xlim([-50 50])
ylim([-50 50])
hold on
xlabel('x')
ylabel('y')
hold on
grid on
title('NLGL Circle Following Demo')

% The mission loop
for i = 1:1000
    x_usv = x_usv + v_usv * cos(h_usv) * delta_t;
    y_usv = y_usv + v_usv * sin(h_usv) * delta_t;
    
    %% NLGL circle guidance algorithm
    
    % Calculate the potential VTPs
    [xq, yq] = circcirc(center_x,center_y,radius, x_usv, y_usv,radius_NLGL);
    q1 = [xq(1), yq(1)];
    q2 = [xq(2), yq(2)];
    
    % Selecting the VTP
    alpha1 = atan2(q1(2)-center_y,q1(1)-center_x);
    alpha2 = atan2(q2(2)-center_y,q2(1)-center_x);
    
    if alpha1 < 0
        alpha1 = alpha1+2*pi;
    end
    
    if alpha2 < 0
        alpha2 = alpha2 + 2*pi;
    end
    
    beta = alpha2 - alpha1;
    
    if abs(beta)>pi
        beta = -beta;
    end
    
    if sign(beta)== sign(circle_direction)
        VTP = q2;
    elseif sign(beta)==-sign(circle_direction)
        VTP = q1;
    end
    
    h_desired_usv = atan2(VTP(2) - y_usv, VTP(1) - x_usv);
    if h_desired_usv < 0
        h_desired_usv = h_desired_usv + 2*pi;
    end
    
    h_usv = h_desired_usv;
    
    % Plot the usv
    [x_usv_plot, y_usv_plot, H, G] = draw_kayaka(x_shape,y_shape,x_usv,y_usv,scale, h_usv);
    pause(0.1)
    delete(H)
    delete(G)
end
