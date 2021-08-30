% integrate CC and IAC
clear,clc,close all
xlim([-100 500])
ylim([-300 300])
hold on

% initialize the path
path_x = [0 100 300 400 300 100 0];
path_y = [0 100*sqrt(3) 100*sqrt(3) 0 -100*sqrt(3) -100*sqrt(3) 0];
plot(path_x, path_y,'r');
hold on

% initialize the USV states
XM0 = 0;
YM0 = 0;
VM = 2;
psi = pi/2;
XM = XM0;
YM = YM0;

% initialize the CC parameter
delta = 10;
alpha = 0.3;
delta_t = 1; 

% initialize the guidance following law; 1 is the CC following straight
% line, 2 is the CC following circle
guidance_index = 1;

%% Draw the mission trajectory
% modify the path
mission_x = zeros(1,12);
mission_y = zeros(1,12);

[p,n]=size(path_x);

for i = 1:(n-1)
    [mission_x(2*i-1), mission_y(2*i-1)] = extend_segment_start(path_x(i), path_y(i), path_x(i+1), path_y(i+1));
    [mission_x(2*i), mission_y(2*i)] = extend_segment_end(path_x(i), path_y(i), path_x(i+1), path_y(i+1));
   % plot([mission_x(2*i-1),mission_x(2*i)],[mission_y(2*i-1),mission_y(2*i)],'b')
    %hold on
    
end
mission_x = [mission_x, mission_x(1),mission_x(2)];
mission_y = [mission_y, mission_y(1),mission_y(2)];


% r is the radius of the turning circle
r = 20;

% initialize the first waypoint
m = 2;
XT0 = mission_x(m);
YT0 = mission_y(m);

%% Drawing the path

path_i = 2;

for i = 2:7
    path_i = i;
    mission_i = 2*path_i-3;
    w1_x = mission_x(mission_i);
    w1_y = mission_y(mission_i);
    w2_x = mission_x(mission_i+1);
    w2_y = mission_y(mission_i+1);
    w3_x = mission_x(mission_i+2);
    w3_y = mission_y(mission_i+2);
    w4_x = mission_x(mission_i+3);
    w4_y = mission_y(mission_i+3);

    %%
    theta_s = atan2(w2_y-w1_y, w2_x-w1_x);
    theta_f = atan2(w3_y-w4_y, w3_x-w4_x);
    theta_m = (theta_s+theta_f)/2;
    t_x = path_x(path_i);
    t_y = path_y(path_i);

    delta_theta = theta_m-theta_s;
    R_r = r/cos(delta_theta);

    % here is the center of the turning circle
    circle_center_x = t_x + R_r*cos(theta_m);
    circle_center_y = t_y + R_r*sin(theta_m);

    plot(t_x, t_y, '*')

    theta_turn_s = atan2(w2_y-circle_center_y, w2_x-circle_center_x);
    theta_turn_f = atan2(w3_y-circle_center_y, w3_x-circle_center_x);
    if theta_turn_s < 0
        theta_turn_s = theta_turn_s + 2* pi;
    end
    if theta_turn_f < 0
        theta_turn_f = theta_turn_f + 2* pi;
    end

    if theta_turn_s>theta_turn_f
        theta_turn_s = theta_turn_s - 2*pi;
    end
    
    if theta_turn_f - theta_turn_s < pi
       theta_turn_s = theta_turn_s + 2*pi;
       circle_theta = theta_turn_f:0.01:theta_turn_s;
    else circle_theta = theta_turn_s:0.01:theta_turn_f;
    end
     
    circle_theta = theta_turn_s:0.01:theta_turn_f;

    circle_x = circle_center_x + r*tan(delta_theta)*cos(circle_theta);
    circle_y = circle_center_y + r*tan(delta_theta)*sin(circle_theta);

    distance = sqrt((circle_center_x-w2_x)^2+(circle_center_y-w2_y)^2);
    plot(circle_x, circle_y, 'r')
    hold on
    
    plot([mission_x(mission_i),mission_x(mission_i+1)],[mission_y(mission_i), mission_y(mission_i+1)],'r')
    hold on
    plot([mission_x(mission_i+2),mission_x(mission_i+3)],[mission_y(mission_i+2), mission_y(mission_i+3)],'r')
    hold on
end


%% MISSION!!!
% mission loop
for i = 1:3000
    
    while guidance_index == 1
        R_u = sqrt((XM0 - XM)^2+(YM0 - YM)^2);
        theta = atan2(YT0 - YM0, XT0 - XM0);
        theta_u = atan2(YM - YM0, XM - XM0);
        beta = theta - theta_u;
        R = sqrt(R_u^2 - (R_u * sin(beta))^2);
        % target position
        x_t = XM0 + (R+delta)*cos(theta);
        y_t = YM0 + (R+delta)*sin(theta);

        psi_d = atan2(y_t - YM, x_t - XM);
        % convert psi_d into [0 2pi];
        if psi_d < 0
            psi_d = psi_d + 2*pi;
        end
        if psi_d - psi > pi
           psi_d = psi_d - 2*pi;
        end
        if psi_d - psi < -pi
            psi_d = psi_d + 2*pi;
        end


        if abs(psi_d - psi) > 10/180*pi
            delta_psi = abs(psi_d - psi)/(psi_d - psi)*10/180*pi;
        else delta_psi = psi_d - psi;
        end

        psi = psi + delta_psi + (rand(1)*5-5)*pi/180;

        XM = XM+VM*cos(psi)*delta_t;
        YM = YM+VM*sin(psi)*delta_t;

        plot(XM, YM,'.','MarkerEdgeColor','b');
        drawnow;
        pause(0.1)

        if sqrt((XM-XT0)^2+(YM-YT0)^2)<5
            guidance_index = 2;
         
            
%             XM0 = mission_x(m);
%             YM0 = mission_y(m);
%             m = m+1;
%             if m>7
%                 break
%             end
%             XT0 = mission_x(m);
%             YT0 = mission_y(m);
        end
    end
    
    while guidance_index == 2
        % get the turning circle center first
        
        w1_x = mission_x(2*m-3);
        w1_y = mission_y(2*m-3);

        w2_x = mission_x(2*m-2);
        w2_y = mission_y(2*m-2);

        w3_x = mission_x(2*m-1);
        w3_y = mission_y(2*m-1);

        w4_x = mission_x(2*m);
        w4_y = mission_y(2*m);
        
        theta_s = atan2(w2_y-w1_y, w2_x-w1_x);
        theta_f = atan2(w3_y-w4_y, w3_x-w4_x);
        theta_m = (theta_s+theta_f)/2;
        

        delta_theta = theta_m-theta_s;
        R_r = r/cos(delta_theta);
        
        center_x = path_x(m) + R_r*cos(theta_m);
        center_y = path_y(m) + R_r*sin(theta_m);
        
        
        theta = atan2(YM-center_y, XM-center_x);
        if theta<0
            theta = theta + 2*pi;
        end
        
        % analysis if beta is position or negative
        %alpha = 0.3;
        
        x_t = center_x + r*tan(delta_theta)*cos(theta+alpha);
        y_t = center_y + r*tan(delta_theta)*sin(theta+alpha);

        psi_d = atan2(y_t-YM, x_t-XM);
        if psi_d < 0
            psi_d = psi_d + 2*pi;
        end
        while psi<0
            psi = psi+2*pi;
        end
        while psi>2*pi
            psi = psi-2*pi;
        end

        if psi_d - psi > pi
           psi_d = psi_d - 2*pi;
        end
        if psi_d - psi < -pi
            psi_d = psi_d + 2*pi;
        end


        if abs(psi_d - psi) > 10/180*pi
            delta_psi = abs(psi_d - psi)/(psi_d - psi)*10/180*pi;
        else delta_psi = psi_d - psi;
        end

        psi = psi + delta_psi + (rand(1)*5-5)*pi/180;


        XM = XM + VM*delta_t*cos(psi_d);
        YM = YM + VM*delta_t*sin(psi_d);

        plot(XM, YM,'.','MarkerEdgeColor','b');
        drawnow;
        pause(0.1)
        % calculate the final switching point % switch to guidance_index = 1

        if sqrt((XM-mission_x(2*m-1))^2+(YM-mission_y(2*m-1))^2)<10
            m = m+1;
            guidance_index = 1;
            XM0 = mission_x(2*m-3);
            YM0 = mission_y(2*m-3);
            XT0 = mission_x(2*m-2);
            YT0 = mission_y(2*m-2);
            
        end
 
    
    end
    
    
   
    
end


         
    


% make the path planner