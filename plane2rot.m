function [rot] = plane2rot(plane)
    [~,i] = min(abs(plane));
    dt = plane(i);
    v2 = zeros(3,1);
    v2(i) = 1;
    v2 = v2-dt*plane;
    v3 = cross(plane,v2);
    rot = [v2 plane v3 zeros(3,1);zeros(1,3) 1];
end