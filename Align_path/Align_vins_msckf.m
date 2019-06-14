% P = [R1' -R1'*t1 ;
%     0' 1] * PW_R
%   = T1 * PW_R
% P = [R2' -R2'*t2 ;
%     0' 1] * PW_E
%   = T2 * PW_E
% PW_E = T2^(-1) * T1 * PW_R
% 
% P - point in Camera Coordinate
% PW_R - point in World Coordinate of traj_Real
% PW_E - point in World Coordinate of estimate

clear;
load('data\groundtruth\V1_01_easy\data_V101_easy.mat');

filePath='data\msckf_V101_estimate.bag';
filePath1='data\vins_V101_estimate.bag';
bag=rosbag(filePath);
bag1=rosbag(filePath1);
odom_message = select(bag,'MessageType','nav_msgs/Odometry');
odom_message1 = select(bag1,'MessageType','nav_msgs/Odometry');
data = readMessages(odom_message);
data1 = readMessages(odom_message1);
msckf_length = length(data);
vins_length = length(data1);

traj_Estimate_msckf = zeros(3, msckf_length);
% so3_Estimate_msckf = zeros(3, msckf_length);
traj_Real_msckf = zeros(3, msckf_length);
ErrorR_msckf = zeros(msckf_length, 1);

traj_Estimate_vins = zeros(3, vins_length);
% so3_Estimate_vins = zeros(3, vins_length);
traj_Real_vins = zeros(3, vins_length);
ErrorR_vins = zeros(vins_length, 1);

t_Real_msckf = zeros(msckf_length, 1);
t_Real_vins = zeros(vins_length, 1);

init = 0;
SKIP = 20;
idx = 1;
for i=1:msckf_length
    t_nsec = toNsec(data{i, 1}.Header.Stamp.Sec, data{i, 1}.Header.Stamp.Nsec);
    while(tReal(idx, 1)<t_nsec && idx < length(tReal))
        idx = idx+1;
    end
    
    if(init < SKIP)
        init = init+1;
        R = quat2rotm([data{i, 1}.Pose.Pose.Orientation.W, data{i, 1}.Pose.Pose.Orientation.X, data{i, 1}.Pose.Pose.Orientation.Y, data{i, 1}.Pose.Pose.Orientation.Z])*( quat2rotm(trajReal.quat(:, idx)')' );
        t = [data{i, 1}.Pose.Pose.Position.X; 
             data{i, 1}.Pose.Pose.Position.Y; 
             data{i, 1}.Pose.Pose.Position.Z] - R*trajReal.x(:, idx);
         continue;
    end
    if idx == length(tReal)
        msckf_end = i-1;
        break;
    end
    t_Real_msckf(i,1) = t_nsec*10^(-9);
    R_rc = R'*quat2rotm([data{i, 1}.Pose.Pose.Orientation.W, data{i, 1}.Pose.Pose.Orientation.X, data{i, 1}.Pose.Pose.Orientation.Y, data{i, 1}.Pose.Pose.Orientation.Z]);
%     so3_Estimate_msckf(:, i) = (logSO3(R_rc))';
    traj_Estimate_msckf(:, i) = R'*([data{i, 1}.Pose.Pose.Position.X; data{i, 1}.Pose.Pose.Position.Y; data{i, 1}.Pose.Pose.Position.Z]-t);
    traj_Real_msckf(:, i) = trajReal.x(:, idx);
    ErrorR_msckf(i) = norm( logSO3(quat2rotm(trajReal.quat(:, idx)')*R_rc') )/3;
end

init = 0;
idx = 1;
for i=1:vins_length
    t_nsec = toNsec(data1{i, 1}.Header.Stamp.Sec, data1{i, 1}.Header.Stamp.Nsec);
    while(tReal(idx, 1)<t_nsec && idx < length(tReal))
        idx = idx+1;
    end
    
    if(init < SKIP)
        init = init+1;
        R = quat2rotm([data1{i, 1}.Pose.Pose.Orientation.W, data1{i, 1}.Pose.Pose.Orientation.X, data1{i, 1}.Pose.Pose.Orientation.Y, data1{i, 1}.Pose.Pose.Orientation.Z])*( quat2rotm(trajReal.quat(:, idx)')' );
        t = [data1{i, 1}.Pose.Pose.Position.X; 
             data1{i, 1}.Pose.Pose.Position.Y; 
             data1{i, 1}.Pose.Pose.Position.Z] - R*trajReal.x(:, idx);
         continue;
    end
    if idx == length(tReal)
        vins_end = i-1;
        break;
    end
    
    t_Real_vins(i,1) = t_nsec*10^(-9);
    R_rc = R'*quat2rotm([data1{i, 1}.Pose.Pose.Orientation.W, data1{i, 1}.Pose.Pose.Orientation.X, data1{i, 1}.Pose.Pose.Orientation.Y, data1{i, 1}.Pose.Pose.Orientation.Z]);
%     so3_Estimate_vins(:, i) = (logSO3(R_rc))';
    traj_Estimate_vins(:, i) = R'*([data1{i, 1}.Pose.Pose.Position.X; data1{i, 1}.Pose.Pose.Position.Y; data1{i, 1}.Pose.Pose.Position.Z]-t);
    traj_Real_vins(:, i) = trajReal.x(:, idx);
    ErrorR_vins(i) = norm( logSO3(quat2rotm(trajReal.quat(:, idx)')*R_rc') )/3;
end

ErrorX_msckf = computeError(traj_Estimate_msckf, traj_Real_msckf, 1/3);
t_plot_msckf = t_Real_msckf(SKIP+1:msckf_end)-t_Real_msckf(SKIP+1);
ErrorR_plot_msckf = ErrorR_msckf(SKIP+1:msckf_end);
ErrorX_plot_msckf = ErrorX_msckf(SKIP+1:msckf_end);
disp('MSCKF(Rot(rad)  Trans(m)): ');
disp(sqrt(mean([ErrorR_plot_msckf  ErrorX_plot_msckf].^2)));

ErrorX_vins = computeError(traj_Estimate_vins, traj_Real_vins, 1/3);
t_plot_vins = t_Real_vins(SKIP+1:vins_end)-t_Real_vins(SKIP+1);
ErrorR_plot_vins = ErrorR_vins(SKIP+1:vins_end);
ErrorX_plot_vins = ErrorX_vins(SKIP+1:vins_end);
disp('VINS(Rot(rad)  Trans(m)): ');
disp(sqrt(mean([ErrorR_plot_vins  ErrorX_plot_vins].^2)));

figure(1);hold on
plot(t_plot_msckf, ErrorR_plot_msckf);
plot(t_plot_vins, ErrorR_plot_vins);
legend('MSCKF','VINS')
xlabel('t (s)')
ylabel('RMSE attitdude (rad)')
title('RMSE on attitude as function of time')
figure(2);hold on
plot(t_plot_msckf, ErrorX_plot_msckf);
plot(t_plot_vins, ErrorX_plot_vins);
legend('MSCKF','VINS')
xlabel('t (s)')
ylabel('RMSE postion (m)')
title('RMSE on postion as function of time')

figure(3);
plot3(traj_Estimate_msckf(1,SKIP+1:msckf_end),traj_Estimate_msckf(2,SKIP+1:msckf_end),traj_Estimate_msckf(3,SKIP+1:msckf_end),'r');
hold on
plot3(traj_Estimate_vins(1,SKIP+1:vins_end),traj_Estimate_vins(2,SKIP+1:vins_end),traj_Estimate_vins(3,SKIP+1:vins_end),'b');
hold on
plot3(traj_Real_msckf(1,SKIP+1:msckf_end),traj_Real_msckf(2,SKIP+1:msckf_end),traj_Real_msckf(3,SKIP+1:msckf_end),'k');
% hold on
% plot3(traj_Real_vins(1,:),traj_Real_vins(2,:),traj_Real_vins(3,:),'g');
figure(4);
color_traj(traj_Estimate_vins(:,SKIP+1:vins_end), traj_Real_vins(:,SKIP+1:vins_end), ErrorX_vins(SKIP+1:vins_end));
title('VINS');
figure(5);
color_traj(traj_Estimate_msckf(:,SKIP+1:msckf_end), traj_Real_msckf(:,SKIP+1:msckf_end), ErrorX_msckf(SKIP+1:msckf_end));
title('MSCKF');

function sec = toNsec(sec, nsec)
    sec = sec*10^9 + nsec;
end
function error = computeError(Estimate, Real, K)
    error = zeros(length(Estimate), 1);
    for i=1:length(Estimate)
        error(i) = norm(Estimate(:, i) - Real(:, i));
    end
    error = error*K;
end