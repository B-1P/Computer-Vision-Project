files{1} = 0:99;
files{2} = 0:95;
files{3} = 0:93;
prefixes = ['uu_ '; 'um_ '; 'umm_'];
prefixes = cellstr(prefixes);
i = 1;
v = VideoWriter('results2','MPEG-4');
set(v,'FrameRate',2.5);
open(v);
for pre = 1:3
    files2 = files{pre};
    for file = files2
        %% Load in images for this example
        imname = [prefixes{pre} sprintf('%06d',file)];
        data = getData(imname,'testing','calib');
        mask = cast(imread(['MASKS_RESIZED/' imname '.png'])/255,'double');
        disp = cast(imread(['DISPS/' imname '.png'])/255,'double');
        C = cast(imread(['data_road/testing/image_2/' imname '.png']),'double');
        % Mark our road segmentation on the point cloud
        C(:,:,1) = C(:,:,1)+ mask*140;
        C(:,:,2) = C(:,:,2)+ mask*20;
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
        %Fit a plane to the points marked as "road"
        [plane,pt] = fitplane(pts,500,0.3);
        %% Set up variables for drawing the cubes arrows
        points = [1 1 1 1 -1 -1 -1 -1; 1 1 -1 -1 1 1 -1 -1; -1 1 -1 1 -1 1 -1 1; ones(1,8)];
        arrow_pts = [0 0 0 0; 0 0 0.2 -0.2; -1 1 0.8 0.8; ones(1,4)];
        cube = getcube(points);
        %Our rotation matrix to make the cubes lie on the plane
        rot = plane2rot(plane);
        pts = [X(:)';Y(:)';Z(:)'];
        plane_pts = [pts(1,:); pts(2,:); pts(3,:);ones(1,size(pts,2))];
        cube_xs = zeros(5,-1);
        cube_ys = zeros(5,-1);
        % Try statement for the case where there are no detections
        try
            detections = csvread(['RECOGNITIONS/' imname '.csv']);
            boxes = detections(:,1:4);
            box_xs = zeros(5,size(boxes,1));
            box_ys = zeros(5,size(boxes,1));
            scores = detections(:,6);
            angles = detections(:,7);
            [~,~,origin] = KRt_from_P(data.P_left);
            P = data.P_left;
            offset = plane;
            offset(2) = -offset(2);
            ra_pts = [0 0 0 0; 1 -1 -0.8 -0.8; 0 0 0.2 -0.2; ones(1,4)];
            ra_X = pt(1);
            ra_Y = pt(2);
            ra_Z = pt(3);
            t = [eye(3) [ra_X; ra_Y; ra_Z]+offset*1; [0 0 0 1]];
            ra_pts = t*rot*ra_pts;
            ra_pts2D = P*ra_pts;
            ra_pts2D = [ra_pts2D(1,:)./ra_pts2D(3,:);ra_pts2D(2,:)./ra_pts2D(3,:)];
            [axs,ays] = getarrow2D(ra_pts2D);
            for j = 1:size(boxes,1);
                box = boxes(j,:);
                box(1) = max(min(box(1),size(mask,2)),1);
                box(2) = max(min(box(2),size(mask,1)),1);
                box(3) = max(min(box(3),size(mask,2)),1);
                box(4) = max(min(box(4),size(mask,1)),1);
                box_xs(:, j) = [box(1); box(3); box(3); box(1); box(1)];
                box_ys(:, j) = [box(2); box(2); box(4); box(4); box(2)];
                % Here I tried a multitude of methods to place the center
                % of our 3D boxes,the median was the best I could come
                % up with.
                cube_X = X(round(box(2)):round(box(4)),round(box(1)):round(box(3)));
                cube_X = median(cube_X(:));
                cube_Z = Z(round(box(2)):round(box(4)),round(box(1)):round(box(3)));
                cube_Z = median(cube_Z(:));
                % Fit our Y coordinate to be on the plane
                cube_Y = (-cube_X*plane(1) - cube_Z*plane(3) + plane'*pt)/plane(2);
                angle = angles(j)*pi/180;
                rot2 = [cos(angle) 0 sin(angle) 0;
                       0 1 0 0;
                       -sin(angle) 0 cos(angle) 0;
                       0 0 0 1;];
                camera_offset = ([cube_X;cube_Y;cube_Z]-origin);
                camera_offset = [1;1.3;2].*camera_offset./norm(camera_offset);
                % I tried moving the boxes away from the camera to get them
                % more towards the true centers of the cars, it didn't work
                % very well
                cube_X = cube_X + camera_offset(1);
                cube_Z = cube_Z + camera_offset(3);
                s = [1 0 0 0;
                     0 1.3 0 0;
                     0 0 2 0;
                     0 0 0 1];
                new_Y = (-cube_X*plane(1) - cube_Z*plane(3) + plane'*pt)/plane(2);
                t = [eye(3) [cube_X; new_Y; cube_Z]+offset*s(2,2)/2; + [0 0 0 1]];
                apts = t*rot*rot2*arrow_pts;
                points2 = t*rot*rot2*s*points;
                P = data.P_left;

                %Project our lines and point cloud from 3D to 2D
                pts2D = P*plane_pts;
                pts2D = [pts2D(1,:)./pts2D(3,:);pts2D(2,:)./pts2D(3,:)];
                arrow_pts2D = P*apts;
                arrow_pts2D = [arrow_pts2D(1,:)./arrow_pts2D(3,:);arrow_pts2D(2,:)./arrow_pts2D(3,:)];
                points2D = P*points2;
                points2D = [points2D(1,:)./points2D(3,:);points2D(2,:)./points2D(3,:)];
                [xs,ys] = getcube2D(points2D);
                cube_xs = [cube_xs xs];
                cube_ys  = [cube_ys ys];
                [ax,ay] = getarrow2D(arrow_pts2D);
                axs = [axs ax];
                ays = [ays ay];
            end
            %Create our plots for the video
            img = zeros(size(mask));
            f = figure;
            figure(f);
            imshow(img);
            hold on;
            scatter(pts2D(1,:),pts2D(2,:),[],cast(C,'double')'./255,'.');
            hold on;
            plot(box_xs,box_ys,'b');
            hold on;
            plot(axs, ays,'c');
            hold off;
            pause(0.5);
            set(gcf, 'Position', get(0, 'Screensize'));
            frame = getframe(gcf);
            writeVideo(v,frame);
            close;
            pause(0.5);
            figure();
            imshow(img);
            hold on;
            scatter(pts2D(1,:),pts2D(2,:),[],cast(C,'double')'./255,'.');
            hold on;
            plot(axs, ays,'c');
            hold on;
            plot(cube_xs,cube_ys,'r');
            hold off;
            pause(0.5);
            set(gcf, 'Position', get(0, 'Screensize'));
            frame = getframe(gcf);
            writeVideo(v,frame);
            close;
            pause(0.5);
            i = i+1;
        catch
            %When there's no detections we just want to plot the point
            %cloud and the road normal arrow
            P = data.P_left;
            ra_pts = [0 0 0 0; 1 -1 -0.8 -0.8; 0 0 0.2 -0.2; ones(1,4)];
            ra_X = pt(1);
            ra_Y = pt(2);
            ra_Z = pt(3);
            t = [eye(3) [ra_X; ra_Y; ra_Z]+offset*1; [0 0 0 1]];
            ra_pts = t*rot*ra_pts;
			%Project our road arrow and point cloud from 3D to 2D
            ra_pts2D = P*ra_pts;
            ra_pts2D = [ra_pts2D(1,:)./ra_pts2D(3,:);ra_pts2D(2,:)./ra_pts2D(3,:)];
            [axs,ays] = getarrow2D(ra_pts2D);
			pts2D = P*plane_pts;
			pts2D = [pts2D(1,:)./pts2D(3,:);pts2D(2,:)./pts2D(3,:)];
            img = zeros(size(mask));
            f = figure;
            figure(f);
            imshow(img);
            hold on;
            scatter(pts2D(1,:),pts2D(2,:),[],cast(C,'double')'./255,'.');
            hold on;
            plot(axs, ays,'c');
            hold off;
            pause(0.5);
            set(gcf, 'Position', get(0, 'Screensize'));
            frame = getframe(gcf);
            writeVideo(v,frame);
            close;
            i = i+1;
        end
    end
end
close(v);