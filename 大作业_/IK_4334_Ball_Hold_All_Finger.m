close all;
clear;
clc;
figure; 
%初始角度
global Link_finger0
% global Link_finger1
% global Link_finger2
% global Link_finger3
% global Link_finger4
% global Link_finger5
% global Link_finger6
% global Link_finger7
h_sphere = [];
% h_points = [];
while(1)
    th_finger0_1=0;
    th_finger0_2=0;
    th_finger0_3=0;
    th_finger0_4=0;
    th_finger0_5=0;
    th_finger1_1=0;
    th_finger1_2=0;
    th_finger1_3=0;
    th_finger1_4=0;
    th_finger1_5=0;
    th_finger2_1=0;
    th_finger2_2=0;
    th_finger2_3=0;
    th_finger2_4=0;
    th_finger2_5=0;
    th_finger3_1=0;
    th_finger3_2=0;
    th_finger3_3=0;
    th_finger3_4=0;
    th_finger3_5=0;
    th_finger4_1=0;
    th_finger4_2=0;
    th_finger4_3=0;
    th_finger4_4=0;
    th_finger4_5=0;
    th_finger5_1=0;
    th_finger5_2=0;
    th_finger5_3=0;
    th_finger5_4=0;
    th_finger5_5=0;
    th_finger6_1=0;
    th_finger6_2=0;
    th_finger6_3=0;
    th_finger6_4=0;
    th_finger6_5=0;
    th_finger7_1=0;
    th_finger7_2=0;
    th_finger7_3=0;
    th_finger7_4=0;
    th_finger7_5=0;
    stp=10;
    dtime=0.02;
    % DHfk_finger0_Lnya(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5,0); 
    % DHfk_finger1_Lnya(th_finger1_1,th_finger1_2,th_finger1_3,th_finger1_4,th_finger1_5,0); 
    % DHfk_finger2_Lnya(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5,0); 
    % DHfk_finger3_Lnya(th_finger3_1,th_finger3_2,th_finger3_3,th_finger3_4,th_finger3_5,0); 
    % DHfk_finger4_Lnya(th_finger4_1,th_finger4_2,th_finger4_3,th_finger4_4,th_finger4_5,0); 
    % DHfk_finger5_Lnya(th_finger5_1,th_finger5_2,th_finger5_3,th_finger5_4,th_finger5_5,0); 
    % DHfk_finger6_Lnya(th_finger6_1,th_finger6_2,th_finger6_3,th_finger6_4,th_finger6_5,0); 
    % DHfk_finger7_Lnya(th_finger7_1,th_finger7_2,th_finger7_3,th_finger7_4,th_finger7_5,0); 
    DHfk_finger0_Lnya_calculate(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5); 
    DHfk_finger1_Lnya_calculate(th_finger1_1,th_finger1_2,th_finger1_3,th_finger1_4,th_finger1_5); 
    DHfk_finger2_Lnya_calculate(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5); 
    DHfk_finger3_Lnya_calculate(th_finger3_1,th_finger3_2,th_finger3_3,th_finger3_4,th_finger3_5); 
    DHfk_finger4_Lnya_calculate(th_finger4_1,th_finger4_2,th_finger4_3,th_finger4_4,th_finger4_5); 
    DHfk_finger5_Lnya_calculate(th_finger5_1,th_finger5_2,th_finger5_3,th_finger5_4,th_finger5_5); 
    DHfk_finger6_Lnya_calculate(th_finger6_1,th_finger6_2,th_finger6_3,th_finger6_4,th_finger6_5); 
    DHfk_finger7_Lnya_calculate(th_finger7_1,th_finger7_2,th_finger7_3,th_finger7_4,th_finger7_5); 
    if ishandle(h_sphere)
        delete(h_sphere);
    end
%     if ishandle(h_points)
%         delete(h_points);
%     end
%% 球体绘制
    %画球体
    %这里的球体中心必须与Link_finger0(2).p(1:3)-Link_finger0(1).p(1:3)这个向量在同一直线上，否则就需要分别去计算每个关节的轨迹
    x = 0;
    y = 0;
    z = 70+rand*10;
    R_r = 15+10*rand;
    % 画球
%     h_sphere = drawSphere(x, y, z, R_r);
%% 手指目标位置及姿态设置
    P = [x;y;z];
%     A = [0;0;0];
    A = Link_finger0(1).p(1:3);
    pos_now_0 = Link_finger0(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points] = drawPointsOnSphere(P, R_r, A,pos_now_0);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_0 = Link_finger0(6).p(1:3);
    min_distance = inf;  % 初始化最小距离为无穷大
    min_index = 0;  % 初始化最小距离对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距离
        distance = norm(pos_now_0 - p_pos{i});
        % 如果这个距离小于当前的最小距离
        if distance < min_distance
            % 更新最小距离和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    
    % 最小距离对应的 p_pos_
    min_p_pos = p_pos{min_index};
%     min_p_pos = p_pos_0;
    % P_1是p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6,p_pos_7所围成的圆的圆心
    vec_min_p_posP_1 = P_1 - min_p_pos;  % 计算向量P_1P和向量Pmin_p_pos
    vec_min_p_posP = P - min_p_pos;   % 计算向量P_1P和向量Pmin_p_pos
    % 计算这两个向量的叉积，得到的向量就是基于点min_p_pos的平面P_1Pmin_p_pos的法向量
    vec_normal = cross(vec_min_p_posP_1, vec_min_p_posP);
    % 计算向量min_p_posP与法向量的叉积，得到的向量就是切线方向
    vec_tangent = cross(vec_min_p_posP, vec_normal);
    % 将切线方向向量归一化，得到单位切线方向向量
    unit_vec_tangent = vec_tangent / norm(vec_tangent);
    % 计算旋转矩阵
    % 选择一个与 unit_vec_tangent 不平行的向量
    vec = [1; 0; 0];
    if abs(dot(unit_vec_tangent, vec)) > 0.99  % 如果它们几乎平行，选择一个不同的向量
        vec = [0; 1; 0];
    end
    % 计算 vec 和 unit_vec_tangent 的叉积
    vec_perp1 = cross(unit_vec_tangent, vec);
    vec_perp1 = vec_perp1 / norm(vec_perp1);  % 归一化向量
    % 计算 vec_perp1 和 unit_vec_tangent 的叉积
    vec_perp2 = cross(unit_vec_tangent, vec_perp1);
    vec_perp2 = vec_perp2 / norm(vec_perp2);  % 归一化向量
    % 现在 unit_vec_tangent、vec_perp1 和 vec_perp2 形成了一个正交基
    % 构造旋转矩阵
    R_33 = [unit_vec_tangent, vec_perp1, vec_perp2];
    %求逆解
    [th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final] = IK_finger0(min_p_pos,R_33);
    %需要做一些判断
    % 由初始和终点关节角，线性计算路径点角度矩阵
%     route_num = 10;
%     route_theta = zeros(route_num, 5);  % 初始化路径点矩阵
%     route_theta(:, 1) = linspace(th_finger0_1, th_finger0_1_final, route_num);
%     route_theta(:, 2) = linspace(th_finger0_2, th_finger0_2_final, route_num);
%     route_theta(:, 3) = linspace(th_finger0_3, th_finger0_3_final, route_num);
%     route_theta(:, 4) = linspace(th_finger0_4, th_finger0_4_final, route_num);
%     route_theta(:, 5) = linspace(th_finger0_5, th_finger0_5_final, route_num);
%% RRT路径规划
    J1 = [-45,45];
    J2 = [-90,0];
    J3 = [-90,0];
    J4 = [-90,0];
    J5 = [-90,0];
    q_Min = [J1(1), J2(1), J3(1), J4(1), J5(1)];
    q_Max = [J1(2), J2(2), J3(2), J4(2), J5(2)];
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger0_1_final, th_finger0_2_final, th_finger0_3_final, th_finger0_4_final, th_finger0_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num, route_theta] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    
    
    
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num <= 4
        new_route_theta = [];
        for i = 1:route_num-1
            % 对每两个路径点进行线性插值
            new_route_theta = [new_route_theta; route_theta(i, :); (route_theta(i, :) + route_theta(i+1, :))/2];
        end
        new_route_theta = [new_route_theta; route_theta(route_num, :)];
        
        % 更新路径点数量和路径点矩阵
        route_num = size(new_route_theta, 1);
        route_theta = new_route_theta;
    end
    fprintf('逆解得到的末端角度\n' );
    disp(q_Goal)
    fprintf('插值后的RRT_angle:\n');
    disp(route_theta)
%% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    
    for i = 1:route_num
        DHfk_finger0_Lnya_calculate(route_theta(i, 1), route_theta(i, 2), route_theta(i, 3), route_theta(i, 4), route_theta(i, 5));
        route_point(i,:) = Link_finger0(6).p;
    end

    for joint_num = 1:5
        [Trajectory_theta(:,joint_num), Trajectory_speed(:,joint_num), Trajectory_acceleration(:,joint_num), t] = Trajectory4334(route_theta, T, joint_num, time_gap);
    end

    for i = 1:(T/time_gap + 1)
        DHfk_finger0_Lnya_calculate(Trajectory_theta(i,1),Trajectory_theta(i,2),Trajectory_theta(i,3),Trajectory_theta(i,4),Trajectory_theta(i,5));

        final_route(i,:) = Link_finger0(6).p;
    end
%     h_sphere = drawSphere(x, y, z, R_r);
%% 正运动学计算
    for i = 1:(T/time_gap + 1)
%         p2 = plot3(final_route(:,1),final_route(:,2),final_route(:,3),'m-');
% %         hold on
%         p3 = plot3(route_point(:,1),route_point(:,2),route_point(:,3),'b*');
% %         hold on
        h_sphere = drawSphere(x, y, z, R_r);
        if i~=T/time_gap + 1
            DHfk_fingers_Lnya(Trajectory_theta(i,1),Trajectory_theta(i,2),Trajectory_theta(i,3),Trajectory_theta(i,4),Trajectory_theta(i,5),1); 
          
        % else
        %     DHfk_fingers_Lnya(Trajectory_theta(i,1),Trajectory_theta(i,2),Trajectory_theta(i,3),Trajectory_theta(i,4),Trajectory_theta(i,5),0); 
%             C = min_p_pos + 50*unit_vec_tangent;
%             start_point = min_p_pos;
%             end_point = C;
%             quiver3(start_point(1), start_point(2), start_point(3), end_point(1)-start_point(1), end_point(2)-start_point(2), end_point(3)-start_point(3), 'r');

        end
    end
%% 抓球运动
%     for i = 0:50
%         z = z + 1;
%         P = [x;y;z];
%         A = Link_finger0(1).p(1:3);
%         pos_now_0 = Link_finger0(6).p(1:3); 
%         [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points] = drawPointsOnSphere(P, R_r, A,pos_now_0);
%         p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
%         pos_now_0 = Link_finger0(6).p(1:3);
%         min_distance = inf;  % 初始化最小距离为无穷大
%         min_index = 0;  % 初始化最小距离对应的索引 
%         % 遍历每个点
%         for i = 1:length(p_pos)
%             % 计算距离
%             distance = norm(pos_now_0 - p_pos{i});
%             % 如果这个距离小于当前的最小距离
%             if distance < min_distance
%                 % 更新最小距离和对应的索引
%                 min_distance = distance;
%                 min_index = i;
%             end
%         end
% 
%         % 最小距离对应的 p_pos_
%         min_p_pos = p_pos{min_index};
%     %     min_p_pos = p_pos_0;
%         % P_1是p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6,p_pos_7所围成的圆的圆心
%         vec_min_p_posP_1 = P_1 - min_p_pos;  % 计算向量P_1P和向量Pmin_p_pos
%         vec_min_p_posP = P - min_p_pos;   % 计算向量P_1P和向量Pmin_p_pos
%         % 计算这两个向量的叉积，得到的向量就是基于点min_p_pos的平面P_1Pmin_p_pos的法向量
%         vec_normal = cross(vec_min_p_posP_1, vec_min_p_posP);
%         % 计算向量min_p_posP与法向量的叉积，得到的向量就是切线方向
%         vec_tangent = cross(vec_min_p_posP, vec_normal);
%         % 将切线方向向量归一化，得到单位切线方向向量
%         unit_vec_tangent = vec_tangent / norm(vec_tangent);
%         % 计算旋转矩阵
%         % 选择一个与 unit_vec_tangent 不平行的向量
%         vec = [1; 0; 0];
%         if abs(dot(unit_vec_tangent, vec)) > 0.99  % 如果它们几乎平行，选择一个不同的向量
%             vec = [0; 1; 0];
%         end
%         % 计算 vec 和 unit_vec_tangent 的叉积
%         vec_perp1 = cross(unit_vec_tangent, vec);
%         vec_perp1 = vec_perp1 / norm(vec_perp1);  % 归一化向量
%         % 计算 vec_perp1 和 unit_vec_tangent 的叉积
%         vec_perp2 = cross(unit_vec_tangent, vec_perp1);
%         vec_perp2 = vec_perp2 / norm(vec_perp2);  % 归一化向量
%         % 现在 unit_vec_tangent、vec_perp1 和 vec_perp2 形成了一个正交基
%         % 构造旋转矩阵
%         R_33 = [unit_vec_tangent, vec_perp1, vec_perp2];
%         %求逆解
%         [th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final] = IK_finger0(min_p_pos,R_33);
%         h_sphere = drawSphere(x, y, z, R_r);
%         DHfk_fingers_Lnya(th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,1)
%     end
%     for i = 0:50
%         z = z - 1;
%         P = [x;y;z];
%         A = Link_finger0(1).p(1:3);
%         pos_now_0 = Link_finger0(6).p(1:3); 
%         [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points] = drawPointsOnSphere(P, R_r, A,pos_now_0);
%         p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
%         pos_now_0 = Link_finger0(6).p(1:3);
%         min_distance = inf;  % 初始化最小距离为无穷大
%         min_index = 0;  % 初始化最小距离对应的索引 
%         % 遍历每个点
%         for i = 1:length(p_pos)
%             % 计算距离
%             distance = norm(pos_now_0 - p_pos{i});
%             % 如果这个距离小于当前的最小距离
%             if distance < min_distance
%                 % 更新最小距离和对应的索引
%                 min_distance = distance;
%                 min_index = i;
%             end
%         end
% 
%         % 最小距离对应的 p_pos_
%         min_p_pos = p_pos{min_index};
%     %     min_p_pos = p_pos_0;
%         % P_1是p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6,p_pos_7所围成的圆的圆心
%         vec_min_p_posP_1 = P_1 - min_p_pos;  % 计算向量P_1P和向量Pmin_p_pos
%         vec_min_p_posP = P - min_p_pos;   % 计算向量P_1P和向量Pmin_p_pos
%         % 计算这两个向量的叉积，得到的向量就是基于点min_p_pos的平面P_1Pmin_p_pos的法向量
%         vec_normal = cross(vec_min_p_posP_1, vec_min_p_posP);
%         % 计算向量min_p_posP与法向量的叉积，得到的向量就是切线方向
%         vec_tangent = cross(vec_min_p_posP, vec_normal);
%         % 将切线方向向量归一化，得到单位切线方向向量
%         unit_vec_tangent = vec_tangent / norm(vec_tangent);
%         % 计算旋转矩阵
%         % 选择一个与 unit_vec_tangent 不平行的向量
%         vec = [1; 0; 0];
%         if abs(dot(unit_vec_tangent, vec)) > 0.99  % 如果它们几乎平行，选择一个不同的向量
%             vec = [0; 1; 0];
%         end
%         % 计算 vec 和 unit_vec_tangent 的叉积
%         vec_perp1 = cross(unit_vec_tangent, vec);
%         vec_perp1 = vec_perp1 / norm(vec_perp1);  % 归一化向量
%         % 计算 vec_perp1 和 unit_vec_tangent 的叉积
%         vec_perp2 = cross(unit_vec_tangent, vec_perp1);
%         vec_perp2 = vec_perp2 / norm(vec_perp2);  % 归一化向量
%         % 现在 unit_vec_tangent、vec_perp1 和 vec_perp2 形成了一个正交基
%         % 构造旋转矩阵
%         R_33 = [unit_vec_tangent, vec_perp1, vec_perp2];
%         %求逆解
%         [th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final] = IK_finger0(min_p_pos,R_33);
%         h_sphere = drawSphere(x, y, z, R_r);
%         DHfk_fingers_Lnya(th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,1)
%         %   绘制4334轨迹规划后的位置速度、加速度
% %         draw_trajectory(Trajectory_theta, Trajectory_speed, Trajectory_acceleration, T, time_gap, t);
% %         pause;
%     end
    
    % cla
%     % 延时1秒
% %     pause(1);
end