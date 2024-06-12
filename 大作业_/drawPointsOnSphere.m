% 目标点获取思路：以机械手的基座点A与球心P相连,计算出AP的单位向量tao，然后以点P为起点，在tao方向上延伸0.75R的长度得到点P_1，然后计算与向量AP_1垂直的平面的法向量normal，计算球和平面的交线(圆)的半径circle_radius，最后在圆上生成八个点，这八个点就是我们要找的目标点。将八个目标点分配给每个手指即可。
function [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points] = drawPointsOnSphere(P, R, A,pos_now_0)
    % 计算单位向量tao
    AP = P - A;
    tao = AP / norm(AP);
    % 计算圆心P_1
    % P_1 = P + 0.01* R * tao;
    P_1 = P + 0.01* tao;
    % 计算与向量AP_1垂直的平面的法向量
    normal = tao;
    % 计算球和平面的交线的半径
    circle_radius = sqrt(R^2 - norm(P - P_1)^2);
    % 在圆上生成八个点
    theta = linspace(0, 2*pi, 9);
    theta = theta(1:end-1);  % 移除最后一个点，因为它与第一个点重合
    % 计算圆上的点
    %重新给点：首先，假设有三个已知点：pos_now_0，P为球心坐标，P_2 = P+5Rtao，以pos_now_0、P、P_2围成一个三角形Triangle_p，在原本代码中的那个生成八个点的圆上，找出这样一个点，它是该圆与三角形Triangle_p三条边以及三角形Triangle_p内部的唯一一个交点，以该点作为p_pos_0，然后再按等角度去给出圆上的剩余7个点。

    P_2 = P + 5 * R * tao;
    Triangle_p = [pos_now_0, P, P_2];
    points = zeros(3, length(theta));
    R_TCP = 5;
    for i = 1:length(theta)
        % 计算在圆上的点
        rotMatrix = axang2rotm([normal', theta(i)]);
        point = P_1 + circle_radius * rotMatrix * [1; 0; 0];
        % 检查点是否在三角形内部
        if isPointInTriangle(point, Triangle_p)
            p_pos_0 = point;
            break;
        end

    end
    % 在圆上生成剩余的点
    for i = 1:8
        theta = 2 * pi * i / 8;
        rotMatrix = axang2rotm([normal', theta]);
        points(:, i) = P_1 + (circle_radius+R_TCP) * rotMatrix * [1; 0; 0];
%         points(:, i) = P_1 + circle_radius * rotMatrix * [1; 0; 0];
    end
    % 在球上绘制点
%     hold on;
%     h_points = plot3(points(1, :), points(2, :), points(3, :), 'k*');
    h_points = 0;
    % 将点的坐标赋给向量p_pos_0到p_pos_7
    p_pos_0 = points(:, 1);
    p_pos_1 = points(:, 2);
    p_pos_2 = points(:, 3);
    p_pos_3 = points(:, 4);
    p_pos_4 = points(:, 5);
    p_pos_5 = points(:, 6);
    p_pos_6 = points(:, 7);
    p_pos_7 = points(:, 8);
    % 打印点的坐标
%     disp([p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7]);
end

function isIn = isPointInTriangle(point, triangle)
    % 这个函数检查一个点是否在一个三角形内部
    % 使用barycentric coordinates（重心坐标）来判断

    % 三角形的顶点
    A = triangle(:, 1);
    B = triangle(:, 2);
    C = triangle(:, 3);

    % 计算向量
    v0 = C - A;
    v1 = B - A;
    v2 = point - A;

    % 计算重心坐标
    dot00 = dot(v0, v0);
    dot01 = dot(v0, v1);
    dot02 = dot(v0, v2);
    dot11 = dot(v1, v1);
    dot12 = dot(v1, v2);

    % 计算分母
    invDenom = 1 / (dot00 * dot11 - dot01 * dot01);

    % 计算重心坐标
    u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    % 检查点是否在三角形内部
    isIn = (u >= 0) & (v >= 0) & (u + v < 1);

    return
end