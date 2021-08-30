function d = distance(x,y, initialwaypoint_x,initialwaypoint_y,finalwaypoint_x, finalwaypoint_y)

w_vector = [ initialwaypoint_x - finalwaypoint_x,initialwaypoint_y -  finalwaypoint_y ];
d = ((initialwaypoint_y - finalwaypoint_y)*x + (finalwaypoint_x - initialwaypoint_x)*y + (initialwaypoint_x*finalwaypoint_y -finalwaypoint_x * initialwaypoint_y) )/norm(w_vector);

