function [plane] = fitplane(pts,k,t)
    inliers = zeros(k,1);
    s = size(pts,1);
    norms = zeros(3,k);
    for i = 1:k
        p = randperm(s,3);
        normal = cross(pts(:,p(2))-pts(:,p(1)),pts(:,p(3))-pts(:,p(1)));
        norms(:,i) = normal;
        ys = (-pts(1,:)*normal(1) - pts(3,:)*normal(3) + normal'*pts(:,p(1)))/normal(2);
        inliers(i) = sum(abs(ys-pts(2,:)) < t);
    end
    [~, i] = max(inliers);
    plane = norms(:,i);
end