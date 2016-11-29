%Simulate the inputs from our detectors
o = zeros(12,1);
o(2) = 1;
final = zeros(566,724,3,50,'uint8');
angle = (find(o==1)-1)*pi/6;
points = [1 1 1 1 -1 -1 -1 -1; 1 1 -1 -1 1 1 -1 -1; -1 1 -1 1 -1 1 -1 1; ones(1,8)];
cube = getcube(points);
x = -20 +40*rand(1,2000);
z = 30+20*rand(1,2000);
y = 0.4*z-16;
pts = [x;y;z];
[plane,pt] = fitplane(pts,2,1);
rot = plane2rot(plane);
rot(2,:) = -1*rot(2,:);
X = pt(1);
Y = pt(2);
Z = pt(3);
plane_pts = [pts(1,:); -pts(2,:); pts(3,:);ones(1,2000)];
% rot = [cos(angle) 0 sin(angle) 0;
%        0 1 0 0;
%        -sin(angle) 0 cos(angle) 0;
%        0 0 0 1;];
s = [0.5 0 0 0;
     0 0.5 0 0;
     0 0 0.5 0;
     0 0 0 1];
t = [eye(3) [X; -Y; Z]; [0 0 0 1]];
cube = t*s*rot*cube;
points = t*s*rot*points;
%Plot the cube in 3D
figure();
plot3(cube(1,:),cube(3,:),cube(2,:));

%Simulate camera parameters
f = 1000;
p = 0;
ang = 0;
T = [eye(3) [0;0;0];[0 0 0 1]];
R = [cos(ang) 0 sin(ang) 0;
     0 1 0 0;
     -sin(ang) 0 cos(ang) 0;
     0 0 0 1;];
K = [f 0 320;0 f 240; 0 0 1];
P = [eye(3) [0;0;0;]];

%Project our lines from 3D to 2D
points2D = K*P*R*T*points;
pts2D = K*P*R*T*plane_pts;
points2D = [points2D(1,:)./points2D(3,:);points2D(2,:)./points2D(3,:)];
[xs,ys] = getcube2D(points2D);
% xs = points2D(1,:);
% ys = points2D(2,:);
img = zeros(480,640);
f = figure;
figure(f);
imshow(img);
hold on;
plot(xs,ys);
hold on;
plot(pts2D(1,:)./pts2D(3,:),pts2D(2,:)./pts2D(3,:),'r.');
% frame = getframe(gcf);
% final(:,:,:,i) = frame.cdata;
% close;
% mov = immovie(final);
% implay(mov);