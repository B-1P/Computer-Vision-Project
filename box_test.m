files{1} = 0:99;
files{2} = 0:95;
files{3} = 0:93;
prefixes = ['uu_ '; 'um_ '; 'umm_'];
prefixes = cellstr(prefixes);
for pre = 1:3
    files2 = files{pre};
    for file = files2
        imname = [prefixes{pre} sprintf('%06d',file)];
        data = getData(imname,'testing','calib');
        mask = cast(imread(['MASKS_RESIZED/' imname '.png'])/255,'double');
        disp = cast(imread(['DISPS/' imname '.png'])/255,'double');
        C = imread(['data_road/testing/image_2/' imname '.png']);
        %% Organize variables for further computation
        % Compute X,Y,Z world co-ordinates from disparity
        Z = data.f*data.baseline./disp;
        X = (repmat(1:size(Z,2),size(Z,1),1)-data.K(1,3))./data.f.*Z;
        Y = (repmat((1:size(Z,1))',1,size(Z,2))-data.K(2,3))./data.f.*Z;
        % Compute point cloud of locations and colours
        valid = disp~=0 & mask==1;
        R = C(:,:,1);
        G = C(:,:,2);
        B = C(:,:,3);
        C = [R(:)';G(:)';B(:)'];
        pts = [X(valid)'; Y(valid)'; Z(valid)'];
        [plane,pt] = fitplane(pts,500,2);
        Y2 = (-pts(1,:)*plane(1) - pts(3,:)*plane(3) + plane'*pt)/plane(2);
        is = find(abs(Y2-pts(2,:)) < 2);
        pts = pts(:,is);
        %% Set up variables for drawing the cube
        points = [1 1 1 1 -1 -1 -1 -1; 1 1 -1 -1 1 1 -1 -1; -1 1 -1 1 -1 1 -1 1; ones(1,8)];
        arrow_pts = [0 0 0 0; 0 0 0.2 -0.2; -1 1 0.8 0.8; ones(1,4)];
        cube = getcube(points);
        rot = plane2rot(plane);
        cube_X = pt(1);
        cube_Y = pt(2);
        cube_Z = pt(3);
        pts = [X(:)';Y(:)';Z(:)'];
        plane_pts = [pts(1,:); pts(2,:); pts(3,:);ones(1,size(pts,2))];
        boxes = [1];
        cube_xs = zeros(5,-1);
        cube_ys = zeros(5,-1);
        for box = boxes
            o = zeros(12,1);
            o(3) = 1;
            angle = (find(o==1)-1)*pi/6;
            rot2 = [cos(angle) 0 sin(angle) 0;
                   0 1 0 0;
                   -sin(angle) 0 cos(angle) 0;
                   0 0 0 1;];
            const = 0.9;
            s = [1 0 0 0;
                 0 const 0 0;
                 0 0 0.5 0;
                 0 0 0 1];
            offset = plane;
            offset(2) = -offset(2);
            t = [eye(3) [cube_X; cube_Y; cube_Z]+offset*const; [0 0 0 1]];
            arrow_pts = t*rot*rot2*s*arrow_pts;
            cube = t*rot*rot2*s*cube;
            points = t*rot*rot2*s*points;
            % Plot the cube in 3D
            % figure();
            % plot3(cube1(1,:),cube1(3,:),cube1(2,:),'r',cube2(1,:),cube2(3,:),cube2(2,:),'b',cube3(1,:),cube3(3,:),cube3(2,:),'g');
            P = data.P_left;

            %Project our lines and point cloud from 3D to 2D
            pts2D = P*plane_pts;
            pts2D = [pts2D(1,:)./pts2D(3,:);pts2D(2,:)./pts2D(3,:)];
            arrow_pts2D = P*arrow_pts;
            arrow_pts2D = [arrow_pts2D(1,:)./arrow_pts2D(3,:);arrow_pts2D(2,:)./arrow_pts2D(3,:)];
            points2D = P*points;
            points2D = [points2D(1,:)./points2D(3,:);points2D(2,:)./points2D(3,:)];
            [xs,ys] = getcube2D(points2D);
            cube_xs = [cube_xs xs];
            cube_ys  = [cube_ys ys];
            [axs,ays] = getarrow2D(arrow_pts2D);
        end
        img = mask;
        f = figure;
        figure(f);
        imshow(img);
        hold on;
        scatter(pts2D(1,:),pts2D(2,:),[],cast(C,'double')'./255,'.');
        hold on;
        plot(axs, ays,'c');
        hold on;
        plot(cube_xs,cube_ys,'r');
        pause(3);
        close all;
    end
end
% center = P*t(:,4);
% center = center./center(3);
% err = const*(max(ys2(:))-center(2))/(max(ys(:))-center(2))-(max(ys3(:))-center(2))/(max(ys(:))-center(2));
% frame = getframe(gcf);
% final(:,:,:,i) = frame.cdata;
% close;
% mov = immovie(final);
% implay(mov);