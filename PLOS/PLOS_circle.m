clear,clc,close all

% Initialize the UAV
x_usv = 45;
y_usv = 0;
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
k1 = 1;
k2 = 0.1;
circle_direction = -1;

% Initialize the plot
xlim([-50 50])
ylim([-50 50])
hold on
xlabel('x')
ylabel('y')
title('PLOS Circle Following Demo')

hold on
grid on


% The mission loop
for i = 1:1000
    x_usv = x_usv + v_usv * cos(h_usv) * delta_t;
    y_usv = y_usv + v_usv * sin(h_usv) * delta_t;
    
    %% PLOS circle guidance algorithm
    p = [x_usv, y_usv];
    center = [center_x, center_y];
    d = norm(p-center) - radius;
    
    theta = atan2(y_usv - center_y, x_usv - center_x);
    if theta<0
        theta = theta+2*pi;
    end
    if circle_direction==1
        theta_p = theta + pi/2;
        h_desired_usv = k1*(theta_p-h_usv) + k2*d + h_usv;
    elseif circle_direction==-1
        theta_p = theta - pi/2;
        h_desired_usv = k1*(theta_p-h_usv) - k2*d + h_usv;
    end

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
