clear;
load('data/extract_data/lineTracks.mat');
load('data/extract_data/data_camPoses.mat');
dir = 'data/extract_data/';
linedata = lineset;
K = [458.6540, 0, 0;
0, 457.2960, 0;
367.2150, 248.3750, 1]';

line3D_data6 = [];
for i=24:-1:2
    id = 0;
    figure(1);
    img = imread(strcat(dir, Images(i, :)));
    imshow(img);
    hold on;
    for j=1:length(linedata)
        if linedata(j).ViewIds(length(linedata(j).ViewIds))==i
            id = id+1;
            ids = linedata(j).ViewIds;
            sp_l(id, :) = linedata(j).StartPoints(size(linedata(j).StartPoints, 1)-1, :);
            ep_l(id, :) = linedata(j).EndPoints(size(linedata(j).StartPoints, 1)-1, :);
            sp_r(id, :) = linedata(j).StartPoints(size(linedata(j).StartPoints, 1), :);
            ep_r(id, :) = linedata(j).EndPoints(size(linedata(j).EndPoints, 1), :);
        end
    end
    [num, Line_3D] = Cal_3D_Line(Orientation(:,:,i-1), Orientation(:,:,i), Location(i-1,:)', Location(i,:)', sp_l, ep_l, sp_r, ep_r, K);
    if(num~=0)
        line3D_data6(size(line3D_data6, 1)+1:size(line3D_data6, 1)+size(Line_3D,1),:) = Line_3D;
    end
    
	figure(1);
    for k = 1:size(sp_r, 1)
        plot([sp_r(k, 1), ep_r(k, 1)]', [sp_r(k, 2), ep_r(k, 2)]', 'LineWidth', 1, 'Color', [1, 0, 0]);
    end
    if(num~=0)
        figure(2);
        for n=1:num
             plot3([Line_3D(n,1), Line_3D(n,4)], [Line_3D(n,2), Line_3D(n,5)], [Line_3D(n,3), Line_3D(n,6)], 'LineWidth', 0.5, 'Color', [0, 0, 1]);
             hold on
        end
    end

    sp_r = [0, 0];
    ep_r = [0, 0];
    sp_l = [0, 0];
    ep_l = [0, 0];    
end
