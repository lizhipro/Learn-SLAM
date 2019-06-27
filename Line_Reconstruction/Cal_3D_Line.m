function [num, Line_3D]= Cal_3D_Line(R1_, R2_, t1_, t2_, line1_sp, line1_ep, line2_sp, line2_ep, K)
R1 = R1_;
R2 = R2_;
t1 = -R1*(t1_);
t2 = -R2*(t2_);

fu = K(1,1);
fv = K(2,2);
cu = K(1,3);
cv = K(2,3);
% Matrix to project line
K_l = [fv, 0, 0;
    0, fu, 0;
    -fv*cu, -fu*cv, fu*fv];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    origin1 = -(R1)'*t1;
    origin2 = -(R2)'*t2;
    num = 0;
    Line_3D = [0 0 0 0 0 0];
    for i=1:length(line1_sp)
        % X1, X2
        line1_sp_cam = R1'*(pixel2cam(line1_sp(i, 1:2), K)-t1);
        line1_ep_cam = R1'*(pixel2cam(line1_ep(i, 1:2), K)-t1);
        line2_sp_cam = R2'*(pixel2cam(line2_sp(i, 1:2), K)-t2);
        line2_ep_cam = R2'*(pixel2cam(line2_ep(i, 1:2), K)-t2);

        % Three points(X1, X2, X3) define a plane
        PI1 = plane_By_points([line1_sp_cam; 1], [line1_ep_cam; 1], [origin1; 1]);
        PI2 = plane_By_points([line2_sp_cam; 1], [line2_ep_cam; 1], [origin2; 1]);
        % Get plucker
        L_PI = line_by_planes(PI1, PI2);

%         e1 = ReprojectionErr_of_line(line1_sp(i, :)', line1_ep(i, :)', L_PI, R1, t1, K_l)
%         e2 = ReprojectionErr_of_line(line2_sp(i, :)', line2_ep(i, :)', L_PI, R2, t2, K_l)
        l1_1 = [origin1;1]*[line1_sp_cam;1]'-[line1_sp_cam;1]*[origin1;1]';
        l1_2 = [origin1;1]*[line1_ep_cam;1]'-[line1_ep_cam;1]*[origin1;1]';
        x1 = point_By_LinePlane2(l1_1, PI2);
        x2 = point_By_LinePlane2(l1_2, PI2);

        proj1 = K*(R2*x1(1:3)+t2);
        proj1 = proj1./proj1(3);
        e1 = norm(line2_sp(i,1:2)-proj1(1:2)');
        proj2 = K*(R2*x2(1:3)+t2);
        proj2 = proj2./proj2(3);
        e2 = norm(line2_ep(i,1:2)-proj2(1:2)');
        x=(x2-x1)
        
%        if (norm(x2-x1)>0.4 && (norm(e1)<3 || norm(e2)<3) && x1(3)<4 && x2(3)<4 && x1(3)>-3 && x2(3)>-3 && norm(x2-x1)<6)
        if (norm(x2-x1)>0.1 && (norm(e1)<5 || norm(e2)<5) && x1(3)<4 && x2(3)<4 && x1(3)>-3 && x2(3)>-3 && norm(x2-x1)<6.5 && ((x(1)<0.06&&x(1)>-0.06) || (x(2)<0.06&&x(2)>-0.06) || (x(3)<0.06&&x(3)>-0.06)) )
%        if (norm(x2-x1)>0.1 && norm(x2-x1)<6.5 && (norm(e1)<5 || norm(e2)<5) && x1(3)<4 && x2(3)<4 && x1(3)>-3 && x2(3)>-3 && ((x(1)<0.1&&x(1)>-0.1) || (x(2)<0.2&&x(2)>-0.2) || (x(3)<0.2&&x(3)>-0.2)) )
            num = num+1;
            Line_3D(num, :) = [x1(1:3)', x2(1:3)'];
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function m = vector2matrix(v)
m = [0, -v(3), v(2);
    v(3), 0, -v(1);
    -v(2), v(1), 0];
end
function v = matrix2vector(m)
v = [m(3,2); m(1,3); m(2,1)];
end
function p_cam = pixel2cam(p_pixel, K)
cam_u = (p_pixel(1)-K(1,3))/K(1,1);
cam_v = (p_pixel(2)-K(2,3))/K(2,2);
p_cam = [cam_u; cam_v; 1];
end

function x = point_By_LinePlane2(L, PI)
x = L*PI;
x = x./x(4);
end

function PI = plane_By_points(X1_, X2_, X3_)
X1 = X1_(1:3);
X2 = X2_(1:3);
X3 = X3_(1:3);
PI = [cross((X1-X3), (X2-X3)); -X3'*cross(X1, X2)];
end
function L = line_by_planes(PI1, PI2)
L_dual = PI1*PI2'-PI2*PI1';
L = [L_dual(1:3,4); matrix2vector(L_dual(1:3,1:3))];
end

function e = ReprojectionErr_of_line(sp, ep, L, R, t, K_l)
L_c = [R,vector2matrix(t)*R;
    zeros(3),R]*L;
line = K_l*L_c(1:3);
line = line./line(3);
e = [(sp(1)*line(1)+sp(2)*line(2)+1)/sqrt(line(1)*line(1) + line(2)*line(2)), (ep(1)*line(1)+ep(2)*line(2)+1)/sqrt(line(1)*line(1) + line(2)*line(2))];
end