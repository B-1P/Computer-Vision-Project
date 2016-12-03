function [plane,pt,pts] = fitplane(pts,k,t)
    inliers = zeros(k,1);
    s = size(pts,2);
    ps = zeros(3,k);
    norms = zeros(3,k);
    for i = 1:k
        p = randperm(s,3);
        normal = cross(pts(:,p(2))-pts(:,p(1)),pts(:,p(3))-pts(:,p(1)));
        norms(:,i) = normal;
        ys = (-pts(1,:)*normal(1) - pts(3,:)*normal(3) + normal'*pts(:,p(1)))/normal(2);
        inliers(i) = sum(abs(ys-pts(2,:)) < t);
        ps(:,i) = pts(:,p(1));
    end
    [~, i] = max(inliers);
    plane = norms(:,i);
    % Robustly fit a plane to the inliers
    % using the algorithm outlined in
    % http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points%
    ys = (-pts(1,:)*plane(1) - pts(3,:)*plane(3) + plane'*ps(:,i))/plane(2);
    is = find(abs(ys-pts(2,:)) < t);
    pts = pts(:,is);
    pt = mean(pts,2);
    pts = pts - repmat(pt,1,size(pts,2));
    xx = pts(1,:)*pts(1,:)';
    xy = pts(1,:)*pts(2,:)';
    yy = pts(2,:)*pts(2,:)';
    xz = pts(1,:)*pts(3,:)';
    yz = pts(2,:)*pts(3,:)';
    zz = pts(3,:)*pts(3,:)';
    dets = [yy*zz - yz*yz,xx*zz - xz*xz,xx*yy - xy*xy];
    [~,i] = max(dets);
    if i == 1
        plane = [1;(xz*yz - xy*zz) / dets(1); (xy*yz - xz*yy) / dets(1)];
    elseif i == 2
        plane = [(xz*yz - xy*zz) / dets(2); 1; (xy*xz - yz*xx) / dets(2)];
    else
        plane = [(yz*xy - xz*yy) / dets(3); (xy*xz - yz*xx) / dets(2); 1];
    end
    plane = plane/norm(plane);
end