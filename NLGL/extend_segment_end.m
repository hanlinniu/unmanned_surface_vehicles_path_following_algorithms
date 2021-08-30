function [ x, y ] = extend_segment_end( XM0, YM0, XT0, YT0 )

theta = atan2(YT0-YM0, XT0-XM0);
d = sqrt((XM0-XT0)^2+(YM0-YT0)^2);

x = XM0 + (d+20)*cos(theta);
y = YM0 + (d+20)*sin(theta);


