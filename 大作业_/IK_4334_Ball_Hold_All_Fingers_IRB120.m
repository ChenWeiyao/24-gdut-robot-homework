clc;
close all;
clear;
figure;
global Link_IRB120
global Link_finger0
global Link_finger1
global Link_finger2
global Link_finger3
global Link_finger4
global Link_finger5
global Link_finger6
global Link_finger7
h_sphere = [];
% h_points = [];
%输入起始关节位置
th1_finger0=0;
th2_finger0=0;
th3_finger0=0;
th4_finger0=0;
th5_finger0=0;
th1_finger1=0;
th2_finger1=0;
th3_finger1=0;
th4_finger1=0;
th5_finger1=0;
th1_finger2=0;
th2_finger2=0;
th3_finger2=0;
th4_finger2=0;
th5_finger2=0;
th1_finger3=0;
th2_finger3=0;
th3_finger3=0;
th4_finger3=0;
th5_finger3=0;
th1_finger4=0;
th2_finger4=0;
th3_finger4=0;
th4_finger4=0;
th5_finger4=0;
th1_finger5=0;
th2_finger5=0;
th3_finger5=0;
th4_finger5=0;
th5_finger5=0;
th1_finger6=0;
th2_finger6=0;
th3_finger6=0;
th4_finger6=0;
th5_finger6=0;
th1_finger7=0;
th2_finger7=0;
th3_finger7=0;
th4_finger7=0;
th5_finger7=0;
th1=0;
th2=0;
th3=0;
th4=0;
th5=0;
th6=0;
stp=10;
dtime=0.02;
while(1)
    %% 计算机械臂与手的初始位姿及末端姿态
    % DHfk_IRB120_Lnya(th1,th2,th3,th4,th5,th6);
    DHfk_fingers_Lnya_Mov_calculate_every_finger(th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7); 
    %
    if ishandle(h_sphere)
        delete(h_sphere);
    end
    %%生成球，给定目标位置，求逆解
    %%随机生成的球心坐标为P,半径为R_r
    % x = 200+200*rand;
    % x = 150+50*rand;
    % y = 380+20*rand;
    % z = 450+50*rand;
    x = 200 + 200 * rand;
    y = 200 + 300 * rand;
    z =  300 + 300 * rand;
    R_r = 20 +  10 * rand;
    P = [x;y;z];
    
    P_Base_IRB120 = [Link_IRB120(4).p(1);Link_IRB120(4).p(2);Link_IRB120(4).p(3)];
    P_Base_IRB120P = P-P_Base_IRB120;
    unit_vec_P_Base_IRB120P = P_Base_IRB120P/norm(P_Base_IRB120P);
    Tpos_IRB120 = P - 350 * unit_vec_P_Base_IRB120P;
    vec_x_IRB120 = cross(unit_vec_P_Base_IRB120P,[1;0;0]);
    if norm(vec_x_IRB120) < 1e-6
        vec_x_IRB120 = cross(unit_vec_P_Base_IRB120P,[0;1;0]);
    end
    vec_x_IRB120 = vec_x_IRB120/norm(vec_x_IRB120);
    vec_y_IRB120 = cross(unit_vec_P_Base_IRB120P,vec_x_IRB120);
    vec_y_IRB120 = vec_y_IRB120/norm(vec_y_IRB120);
    
    rpos_IRB120_goal = [vec_x_IRB120,vec_y_IRB120,unit_vec_P_Base_IRB120P];
    [th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final] = IK_IRB120(Tpos_IRB120,rpos_IRB120_goal,th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0);
    DHfk_fingers_Lnya_Mov_calculate_every_finger(th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    
    P_Base_IRB120 = [Link_finger0(1).p(1);Link_finger0(1).p(2);Link_finger0(1).p(3)];
    P_Base_IRB120P = P-P_Base_IRB120;
    unit_vec_P_Base_IRB120P = P_Base_IRB120P/norm(P_Base_IRB120P);
    Tpos_IRB120 = P - 120 * unit_vec_P_Base_IRB120P;
    unit_position_vec = (P - Link_finger0(1).p(1:3))/norm(P - Link_finger0(1).p(1:3));
    vec_x_position = cross(unit_position_vec,[1;0;0]);
    if norm(vec_x_position) < 1e-6
        vec_x_position = cross(unit_position_vec,[0;1;0]);
    end
    vec_x_position = vec_x_position/norm(vec_x_position);
    vec_y_position = cross(unit_position_vec,vec_x_position);
    vec_y_position = vec_y_position/norm(vec_y_position);
    rpos_IRB120_position = [vec_x_position,vec_y_position,unit_position_vec];
    [th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final] = IK_IRB120_Rpos(Tpos_IRB120,rpos_IRB120_position,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0);
    
    %% IRB120的RRT规划相关输入
    J1_IRB120 = [-165,165];
    J2_IRB120 = [-110,110];
    J3_IRB120 = [-110,70];
    J4_IRB120 = [-180,180];
    J5_IRB120 = [-90,90];
    J6_IRB120 = [-360,360];
    q_Min_IRB120 = [J1_IRB120(1),J2_IRB120(1),J3_IRB120(1),J4_IRB120(1),J5_IRB120(1),J6_IRB120(1)];
    q_Max_IRB120 = [J1_IRB120(2),J2_IRB120(2),J3_IRB120(2),J4_IRB120(2),J5_IRB120(2),J6_IRB120(2)];
    q_Init_IRB120 = [th1,th2,th3,th4,th5,th6];
    q_Goal_IRB120 = [th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final];
    q_fingers = [th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0];
    r_link_IRB120 = 10;
    joint_nums_IRB120 = 6;
    maxIter_IRB120 = 10000;
    stepSize_IRB120 = 100;
    [route_num_IRB120,route_theta_IRB120] =  RRT_Route_Points_Get_IRB120(q_fingers,q_Init_IRB120,q_Goal_IRB120,q_Min_IRB120,q_Max_IRB120,P,R_r,r_link_IRB120,maxIter_IRB120,stepSize_IRB120,joint_nums_IRB120);
    %当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_IRB120 <= 4
        new_route_theta_IRB120 = [];
        for i = 1:route_num_IRB120-1
            % 对每两个路径点进行线性插值
            new_route_theta_IRB120 = [new_route_theta_IRB120; route_theta_IRB120(i, :); (route_theta_IRB120(i, :) + route_theta_IRB120(i+1, :))/2];
        end
        new_route_theta_IRB120 = [new_route_theta_IRB120; route_theta_IRB120(route_num_IRB120, :)];
        
        % 更新路径点数量和路径点矩阵
        route_num_IRB120 = size(new_route_theta_IRB120, 1);
        route_theta_IRB120 = new_route_theta_IRB120;
    end
    %%IRB120的4334轨迹规划
    T_IRB120_All = 5;
    time_gap_IRB120 = 0.1;
    for i = 1:route_num_IRB120
        DHfk_IRB120_Lnya(route_theta_IRB120(i,1),route_theta_IRB120(i,2),route_theta_IRB120(i,3),route_theta_IRB120(i,4),route_theta_IRB120(i,5),route_theta_IRB120(i,6));
        route_point_IRB120(i,:) = Link_IRB120(7).p;
    end
    for joint_num_IRB120 = 1:6
        [Trajectory_theta_IRB120(:,joint_num_IRB120),Trajectory_speed_IRB120(:,joint_num_IRB120),Trajectory_acceleration_IRB120(:,joint_num_IRB120),t_IRB120] = Trajectory4334(route_theta_IRB120,T_IRB120_All,joint_num_IRB120,time_gap_IRB120);
    end
        %   绘制4334轨迹规划后的位置速度、加速度
    %         draw_trajectory(Trajectory_theta_IRB120, Trajectory_speed_IRB120, Trajectory_acceleration_IRB120, T_IRB120_All, time_gap_IRB120, t_IRB120);
    %         pause;
    for i = 1:(T_IRB120_All/time_gap_IRB120+1)
        DHfk_IRB120_Lnya(Trajectory_theta_IRB120(i,1),Trajectory_theta_IRB120(i,2),Trajectory_theta_IRB120(i,3),Trajectory_theta_IRB120(i,4),Trajectory_theta_IRB120(i,5),Trajectory_theta_IRB120(i,6));
        % final_route_IRB120(i,:) = Link_IRB120(7).p;
    end
    for i = 1:(T_IRB120_All/time_gap_IRB120+1)
        h_sphere = drawSphere(x,y,z,R_r);
        if i~=T_IRB120_All/time_gap_IRB120+1
            DHfk_fingers_Lnya_Mov(Trajectory_theta_IRB120(i,1),Trajectory_theta_IRB120(i,2),Trajectory_theta_IRB120(i,3),Trajectory_theta_IRB120(i,4),Trajectory_theta_IRB120(i,5),Trajectory_theta_IRB120(i,6),th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,1);
        else
            DHfk_fingers_Lnya_Mov(Trajectory_theta_IRB120(i,1),Trajectory_theta_IRB120(i,2),Trajectory_theta_IRB120(i,3),Trajectory_theta_IRB120(i,4),Trajectory_theta_IRB120(i,5),Trajectory_theta_IRB120(i,6),th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,0);
        end
    end
    % % %     % 给定切线起始与末端位置，绘制切线
    %     C = Tpos_IRB120 + 200 * unit_vec_P_Base_IRB120P;
    %     start_point = Tpos_IRB120;
    %     end_point = C;
    %     quiver3(start_point(1), start_point(2), start_point(3), end_point(1)-start_point(1), end_point(2)-start_point(2), end_point(3)-start_point(3), 'r');
    DHfk_fingers_Lnya_Mov_calculate_every_finger(th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %%规划每根手指的路径
    % A = Link_finger0(2).p(1:3);
    % pos_now_0_finger0 = Link_finger0(6).p(1:3); 
    % pos_now_0_finger1 = Link_finger1(6).p(1:3);
    % pos_now_0_finger2 = Link_finger2(6).p(1:3);
    % pos_now_0_finger3 = Link_finger3(6).p(1:3);
    % pos_now_0_finger4 = Link_finger4(6).p(1:3);
    % pos_now_0_finger5 = Link_finger5(6).p(1:3);
    % pos_now_0_finger6 = Link_finger6(6).p(1:3);
    % pos_now_0_finger7 = Link_finger7(6).p(1:3);
    % 要求得Trajectory_theta_finger0,Trajectory_theta_finger1,Trajectory_theta_finger2,Trajectory_theta_finger3,Trajectory_theta_finger4,Trajectory_theta_finger5,Trajectory_theta_finger6,Trajectory_theta_finger7
    % [Trajectory_theta_finger0,Trajectory_theta_finger1,Trajectory_theta_finger2,Trajectory_theta_finger3,Trajectory_theta_finger4,Trajectory_theta_finger5,Trajectory_theta_finger6,Trajectory_theta_finger7] = Route_Fingers_RRT_4334(P,A,R_r,pos_now_0_finger0,pos_now_0_finger1,pos_now_0_finger2,pos_now_0_finger3,pos_now_0_finger4,pos_now_0_finger5,pos_now_0_finger6,pos_now_0_finger7,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5,th_finger1_1,th_finger1_2,th_finger1_3,th_finger1_4,th_finger1_5,th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5,th_finger3_1,th_finger3_2,th_finger3_3,th_finger3_4,th_finger3_5,th_finger4_1,th_finger4_2,th_finger4_3,th_finger4_4,th_finger4_5,th_finger5_1,th_finger5_2,th_finger5_3,th_finger5_4,th_finger5_5,th_finger6_1,th_finger6_2,th_finger6_3,th_finger6_4,th_finger6_5,th_finger7_1,th_finger7_2,th_finger7_3,th_finger7_4,th_finger7_5);
    
    % P_fingerssss = [x;y;z;1];
    % P = Link_IRB120(7).A\P_fingerssss;
    % P = P(1:3);
    %%  第一根手指 
    % 手指目标位置及姿态设置
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
    [th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final] = IK_finger0_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    J1 = [-45,0];
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
    
    
    %%第二根手指
    %% 手指目标位置及姿态设置
    %     A = [0;0;0];
    A_1 = Link_finger1(1).p(1:3);
    pos_now_1 = Link_finger1(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points_1] = drawPointsOnSphere(P, R_r, A_1,pos_now_1);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_1 = Link_finger1(6).p(1:3);
    min_distance = inf;  % 初始化最小距离为无穷大
    min_index = 0;  % 初始化最小距离对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距离
        distance = norm(pos_now_1 - p_pos{i});
        % 如果这个距离小于当前的最小距离
        if distance < min_distance
            % 更新最小距离和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    % 最小距离对应的 p_pos_
    min_p_pos = p_pos{min_index};
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
    [th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final] = IK_finger1_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger1_1_final, th_finger1_2_final, th_finger1_3_final, th_finger1_4_final, th_finger1_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_1, route_theta_1] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_1 <= 4
        new_route_theta_1 = [];
        for i = 1:route_num_1-1
            % 对每两个路径点进行线性插值
            new_route_theta_1 = [new_route_theta_1; route_theta_1(i, :); (route_theta_1(i, :) + route_theta_1(i+1, :))/2];
        end
        new_route_theta_1 = [new_route_theta_1; route_theta_1(route_num_1, :)];
        % 更新路径点数量和路径点矩阵
        route_num_1 = size(new_route_theta_1, 1);
        route_theta_1 = new_route_theta_1;
    end
    %% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    for i = 1:route_num_1
        DHfk_finger1_Lnya_calculate(route_theta_1(i, 1), route_theta_1(i, 2), route_theta_1(i, 3), route_theta_1(i, 4), route_theta_1(i, 5));
        route_point_1(i,:) = Link_finger1(6).p;
    end
    for joint_num = 1:5
        [Trajectory_theta_1(:,joint_num), Trajectory_speed_1(:,joint_num), Trajectory_acceleration_1(:,joint_num), t] = Trajectory4334(route_theta_1, T, joint_num, time_gap);
    end
    
    
    %%第三根手指
    %% 手指目标位置及姿态设置
    %     A = [0;0;0];
    A_2 = Link_finger2(1).p(1:3);
    pos_now_2 = Link_finger2(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points_2] = drawPointsOnSphere(P, R_r, A_2,pos_now_2);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_2 = Link_finger2(6).p(1:3);
    min_distance = inf;  % 初始化最小距离为无穷大
    min_index = 0;  % 初始化最小距离对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距离
        distance = norm(pos_now_2 - p_pos{i});
        % 如果这个距离小于当前的最小距离
        if distance < min_distance
            % 更新最小距离和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    % 最小距离对应的 p_pos_
    min_p_pos = p_pos{min_index};
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
    [th_finger2_1_final,th_finger2_2_final,th_finger2_3_final,th_finger2_4_final,th_finger2_5_final] = IK_finger2_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger2_1_final, th_finger2_2_final, th_finger2_3_final, th_finger2_4_final, th_finger2_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_2, route_theta_2] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_2 <= 4
        new_route_theta_2 = [];
        for i = 1:route_num_2-1
            % 对每两个路径点进行线性插值
            new_route_theta_2 = [new_route_theta_2; route_theta_2(i, :); (route_theta_2(i, :) + route_theta_2(i+1, :))/2];
        end
        new_route_theta_2 = [new_route_theta_2; route_theta_2(route_num_2, :)];
        % 更新路径点数量和路径点矩阵
        route_num_2 = size(new_route_theta_2, 1);
        route_theta_2 = new_route_theta_2;
    end
    %% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    for i = 1:route_num_2
        DHfk_finger2_Lnya_calculate(route_theta_2(i, 1), route_theta_2(i, 2), route_theta_2(i, 3), route_theta_2(i, 4), route_theta_2(i, 5));
        route_point_2(i,:) = Link_finger2(6).p;
    end
    for joint_num = 1:5
        [Trajectory_theta_2(:,joint_num), Trajectory_speed_2(:,joint_num), Trajectory_acceleration_2(:,joint_num), t] = Trajectory4334(route_theta_2, T, joint_num, time_gap);
    end
    
    
    %%第四根手指
    %% 手指目标位置及姿态设置
    %     A = [0;0;0];
    A_3 = Link_finger3(1).p(1:3);
    pos_now_3 = Link_finger3(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points_3] = drawPointsOnSphere(P, R_r, A_3,pos_now_3);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_3 = Link_finger3(6).p(1:3);
    min_distance = inf;  % 初始化最小距离为无穷大
    min_index = 0;  % 初始化最小距离对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距离
        distance = norm(pos_now_3 - p_pos{i});
        % 如果这个距离小于当前的最小距离
        if distance < min_distance
            % 更新最小距离和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    % 最小距离对应的 p_pos_
    min_p_pos = p_pos{min_index};
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
    [th_finger3_1_final,th_finger3_2_final,th_finger3_3_final,th_finger3_4_final,th_finger3_5_final] = IK_finger3_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger3_1_final, th_finger3_2_final, th_finger3_3_final, th_finger3_4_final, th_finger3_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_3, route_theta_3] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_3 <= 4
        new_route_theta_3 = [];
        for i = 1:route_num_3-1
            % 对每两个路径点进行线性插值
            new_route_theta_3 = [new_route_theta_3; route_theta_3(i, :); (route_theta_3(i, :) + route_theta_3(i+1, :))/2];
        end
        new_route_theta_3 = [new_route_theta_3; route_theta_3(route_num_3, :)];
        % 更新路径点数量和路径点矩阵
        route_num_3 = size(new_route_theta_3, 1);
        route_theta_3 = new_route_theta_3;
    end
    %% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    for i = 1:route_num_3
        DHfk_finger3_Lnya_calculate(route_theta_3(i, 1), route_theta_3(i, 2), route_theta_3(i, 3), route_theta_3(i, 4), route_theta_3(i, 5));
        route_point_3(i,:) = Link_finger3(6).p;
    end
    for joint_num = 1:5
        [Trajectory_theta_3(:,joint_num), Trajectory_speed_3(:,joint_num), Trajectory_acceleration_3(:,joint_num), t] = Trajectory4334(route_theta_3, T, joint_num, time_gap);
    end
    
    
    %% 第五根手指
    %% 手指目标位置及姿态设置
    %     A = [0;0;0];
    A_4 = Link_finger4(1).p(1:3);
    pos_now_4 = Link_finger4(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points_4] = drawPointsOnSphere(P, R_r, A_4,pos_now_4);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_4 = Link_finger4(6).p(1:3);
    min_distance = inf;  % 初始化最小距离为无穷大
    min_index = 0;  % 初始化最小距离对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距离
        distance = norm(pos_now_4 - p_pos{i});
        % 如果这个距离小于当前的最小距离
        if distance < min_distance
            % 更新最小距禷和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    % 最小距离对应的 p_pos_
    min_p_pos = p_pos{min_index};
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
    [th_finger4_1_final,th_finger4_2_final,th_finger4_3_final,th_finger4_4_final,th_finger4_5_final] = IK_finger4_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger4_1_final, th_finger4_2_final, th_finger4_3_final, th_finger4_4_final, th_finger4_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_4, route_theta_4] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_4 <= 4
        new_route_theta_4 = [];
        for i = 1:route_num_4-1
            % 对每两个路径点进行线性插值
            new_route_theta_4 = [new_route_theta_4; route_theta_4(i, :); (route_theta_4(i, :) + route_theta_4(i+1, :))/2];
        end
        new_route_theta_4 = [new_route_theta_4; route_theta_4(route_num_4, :)];
        % 更新路径点数量和路径点矩阵
        route_num_4 = size(new_route_theta_4, 1);
        route_theta_4 = new_route_theta_4;
    end
    %% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    for i = 1:route_num_4
        DHfk_finger4_Lnya_calculate(route_theta_4(i, 1), route_theta_4(i, 2), route_theta_4(i, 3), route_theta_4(i, 4), route_theta_4(i, 5));
        route_point_4(i,:) = Link_finger4(6).p;
    end
    for joint_num = 1:5
        [Trajectory_theta_4(:,joint_num), Trajectory_speed_4(:,joint_num), Trajectory_acceleration_4(:,joint_num), t] = Trajectory4334(route_theta_4, T, joint_num, time_gap);
    end
    
    
    
    %% 第六根手指
    %% 手指目标位置及姿态设置
    %     A = [0;0;0];
    A_5 = Link_finger5(1).p(1:3);
    pos_now_5 = Link_finger5(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points_5] = drawPointsOnSphere(P, R_r, A_5,pos_now_5);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_5 = Link_finger5(6).p(1:3);
    min_distance = inf;  % 初始化最小距离为无穷大
    min_index = 0;  % 初始化最小距禷对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距离
        distance = norm(pos_now_5 - p_pos{i});
        % 如果这个距禷小于当前的最小距禷
        if distance < min_distance
            % 更新最小距禷和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    % 最小距禷对应的 p_pos_
    min_p_pos = p_pos{min_index};
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
    [th_finger5_1_final,th_finger5_2_final,th_finger5_3_final,th_finger5_4_final,th_finger5_5_final] = IK_finger5_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger5_1_final, th_finger5_2_final, th_finger5_3_final, th_finger5_4_final, th_finger5_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_5, route_theta_5] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_5 <= 4
        new_route_theta_5 = [];
        for i = 1:route_num_5-1
            % 对每两个路径点进行线性插值
            new_route_theta_5 = [new_route_theta_5; route_theta_5(i, :); (route_theta_5(i, :) + route_theta_5(i+1, :))/2];
        end
        new_route_theta_5 = [new_route_theta_5; route_theta_5(route_num_5, :)];
        % 更新路径点数量和路径点矩阵
        route_num_5 = size(new_route_theta_5, 1);
        route_theta_5 = new_route_theta_5;
    end
    %% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    for i = 1:route_num_5
        DHfk_finger5_Lnya_calculate(route_theta_5(i, 1), route_theta_5(i, 2), route_theta_5(i, 3), route_theta_5(i, 4), route_theta_5(i, 5));
        route_point_5(i,:) = Link_finger5(6).p;
    end
    for joint_num = 1:5
        [Trajectory_theta_5(:,joint_num), Trajectory_speed_5(:,joint_num), Trajectory_acceleration_5(:,joint_num), t] = Trajectory4334(route_theta_5, T, joint_num, time_gap);
    end
    
    
    
    %% 第七根手指
    %% 手指目标位置及姿态设置
    %     A = [0;0;0];
    A_6 = Link_finger6(1).p(1:3);
    pos_now_6 = Link_finger6(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points_6] = drawPointsOnSphere(P, R_r, A_6,pos_now_6);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_6 = Link_finger6(6).p(1:3);
    min_distance = inf;  % 初始化最小距禷为无穷大
    min_index = 0;  % 初始化最小距禷对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距离
        distance = norm(pos_now_6 - p_pos{i});
        % 如果这个距禷小于当前的最小距禷
        if distance < min_distance
            % 更新最小距禷和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    % 最小距禷对应的 p_pos_
    min_p_pos = p_pos{min_index};
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
    [th_finger6_1_final,th_finger6_2_final,th_finger6_3_final,th_finger6_4_final,th_finger6_5_final] = IK_finger6_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger6_1_final, th_finger6_2_final, th_finger6_3_final, th_finger6_4_final, th_finger6_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_6, route_theta_6] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_6 <= 4
        new_route_theta_6 = [];
        for i = 1:route_num_6-1
            % 对每两个路径点进行线性插值
            new_route_theta_6 = [new_route_theta_6; route_theta_6(i, :); (route_theta_6(i, :) + route_theta_6(i+1, :))/2];
        end
        new_route_theta_6 = [new_route_theta_6; route_theta_6(route_num_6, :)];
        % 更新路径点数量和路径点矩阵
        route_num_6 = size(new_route_theta_6, 1);
        route_theta_6 = new_route_theta_6;
    end
    %% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    for i = 1:route_num_6
        DHfk_finger6_Lnya_calculate(route_theta_6(i, 1), route_theta_6(i, 2), route_theta_6(i, 3), route_theta_6(i, 4), route_theta_6(i, 5));
        route_point_6(i,:) = Link_finger6(6).p;
    end
    for joint_num = 1:5
        [Trajectory_theta_6(:,joint_num), Trajectory_speed_6(:,joint_num), Trajectory_acceleration_6(:,joint_num), t] = Trajectory4334(route_theta_6, T, joint_num, time_gap);
    end
    
    
    %% 第八根手指
    %% 手指目标位置及姿态设置
    %     A = [0;0;0];
    A_7 = Link_finger7(1).p(1:3);
    pos_now_7 = Link_finger7(6).p(1:3); 
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points_7] = drawPointsOnSphere(P, R_r, A_7,pos_now_7);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    pos_now_7 = Link_finger7(6).p(1:3);
    min_distance = inf;  % 初始化最小距禷为无穷大
    min_index = 0;  % 初始化最小距禷对应的索引 
    % 遍历每个点
    for i = 1:length(p_pos)
        % 计算距禷
        distance = norm(pos_now_7 - p_pos{i});
        % 如果这个距禷小于当前的最小距禷
        if distance < min_distance
            % 更新最小距禷和对应的索引
            min_distance = distance;
            min_index = i;
        end
    end
    % 最小距禷对应的 p_pos_
    min_p_pos = p_pos{min_index};
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
    [th_finger7_1_final,th_finger7_2_final,th_finger7_3_final,th_finger7_4_final,th_finger7_5_final] = IK_finger7_Irb(min_p_pos,R_33,th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);
    %% RRT路径规划
    J1 = [-20,20];
    J2 = [-90,0];
    J3 = [-90,0];
    J4 = [-90,0];
    J5 = [-90,0];
    q_Min = [J1(1), J2(1), J3(1), J4(1), J5(1)];
    q_Max = [J1(2), J2(2), J3(2), J4(2), J5(2)];
    q_Init = [0, 0, 0, 0, 0];
    q_Goal = [th_finger7_1_final, th_finger7_2_final, th_finger7_3_final, th_finger7_4_final, th_finger7_5_final];
    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_7, route_theta_7] = RRT_Route_Points_Get(q_Init, q_Goal, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    %% 线性插值，当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_7 <= 4
        new_route_theta_7 = [];
        for i = 1:route_num_7-1
            % 对每两个路径点进行线性插值
            new_route_theta_7 = [new_route_theta_7; route_theta_7(i, :); (route_theta_7(i, :) + route_theta_7(i+1, :))/2];
        end
        new_route_theta_7 = [new_route_theta_7; route_theta_7(route_num_7, :)];
        % 更新路径点数量和路径点矩阵
        route_num_7 = size(new_route_theta_7, 1);
        route_theta_7 = new_route_theta_7;
    end
    %% 43...34轨迹规划
    T = 5;
    time_gap = 0.1;
    for i = 1:route_num_7
        DHfk_finger7_Lnya_calculate(route_theta_7(i, 1), route_theta_7(i, 2), route_theta_7(i, 3), route_theta_7(i, 4), route_theta_7(i, 5));
        route_point_7(i,:) = Link_finger7(6).p;
    end
    for joint_num = 1:5
        [Trajectory_theta_7(:,joint_num), Trajectory_speed_7(:,joint_num), Trajectory_acceleration_7(:,joint_num), t] = Trajectory4334(route_theta_7, T, joint_num, time_gap);
    end
    
    
    %%  调整轨迹矩阵大小
    Trajectory_theta_num_0 = size(Trajectory_theta, 1);
    Trajectory_theta_num_1 = size(Trajectory_theta_1, 1);
    Trajectory_theta_num_2 = size(Trajectory_theta_2, 1);
    Trajectory_theta_num_3 = size(Trajectory_theta_3, 1);
    Trajectory_theta_num_4 = size(Trajectory_theta_4, 1);
    Trajectory_theta_num_5 = size(Trajectory_theta_5, 1);
    Trajectory_theta_num_6 = size(Trajectory_theta_6, 1);
    Trajectory_theta_num_7 = size(Trajectory_theta_7, 1);
    Trajectory_theta_0 = Trajectory_theta;
    Trajectory_theta_num_fingers = [Trajectory_theta_num_0, Trajectory_theta_num_1, Trajectory_theta_num_2, Trajectory_theta_num_3, Trajectory_theta_num_4, Trajectory_theta_num_5, Trajectory_theta_num_6, Trajectory_theta_num_7];
    % 找到最小行数
    min_rows = min(Trajectory_theta_num_fingers);
    % 对每个Trajectory_theta矩阵进行调整
    for i = 0:7
        eval(['current_matrix = Trajectory_theta_', num2str(i), ';']);
        if size(current_matrix, 1) > min_rows
            % 保留首尾两行，删除中间的行
            middle_rows = linspace(2, size(current_matrix, 1)-1, size(current_matrix, 1)-2);
            rows_to_keep = [1, round(middle_rows(1:(end-(size(current_matrix, 1)-min_rows)))), size(current_matrix, 1)];
            eval(['Trajectory_theta_', num2str(i), ' = current_matrix(rows_to_keep, :);']);
        end
    end
    Trajectory_theta_finger0 = Trajectory_theta_0; 
    Trajectory_theta_finger1 = Trajectory_theta_1;
    Trajectory_theta_finger2 = Trajectory_theta_2;
    Trajectory_theta_finger3 = Trajectory_theta_3;
    Trajectory_theta_finger4 = Trajectory_theta_4;
    Trajectory_theta_finger5 = Trajectory_theta_5;
    Trajectory_theta_finger6 = Trajectory_theta_6;
    Trajectory_theta_finger7 = Trajectory_theta_7;
    %正运动学计算
    for i = 1:(T/time_gap + 1)
        DHfk_fingers_Lnya_Mov_calculate_every_finger(th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,Trajectory_theta_finger0(i,1),Trajectory_theta_finger0(i,2),Trajectory_theta_finger0(i,3),Trajectory_theta_finger0(i,4),Trajectory_theta_finger0(i,5),Trajectory_theta_finger1(i,1),Trajectory_theta_finger1(i,2),Trajectory_theta_finger1(i,3),Trajectory_theta_finger1(i,4),Trajectory_theta_finger1(i,5),Trajectory_theta_finger2(i,1),Trajectory_theta_finger2(i,2),Trajectory_theta_finger2(i,3),Trajectory_theta_finger2(i,4),Trajectory_theta_finger2(i,5),Trajectory_theta_finger3(i,1),Trajectory_theta_finger3(i,2),Trajectory_theta_finger3(i,3),Trajectory_theta_finger3(i,4),Trajectory_theta_finger3(i,5),Trajectory_theta_finger4(i,1),Trajectory_theta_finger4(i,2),Trajectory_theta_finger4(i,3),Trajectory_theta_finger4(i,4),Trajectory_theta_finger4(i,5),Trajectory_theta_finger5(i,1),Trajectory_theta_finger5(i,2),Trajectory_theta_finger5(i,3),Trajectory_theta_finger5(i,4),Trajectory_theta_finger5(i,5),Trajectory_theta_finger6(i,1),Trajectory_theta_finger6(i,2),Trajectory_theta_finger6(i,3),Trajectory_theta_finger6(i,4),Trajectory_theta_finger6(i,5),Trajectory_theta_finger7(i,1),Trajectory_theta_finger7(i,2),Trajectory_theta_finger7(i,3),Trajectory_theta_finger7(i,4),Trajectory_theta_finger7(i,5));
    
        % final_route_finger0(i,:) = Link_finger0(6).p;
        % final_route_finger1(i,:) = Link_finger1(6).p;
        % final_route_finger2(i,:) = Link_finger2(6).p;
        % final_route_finger3(i,:) = Link_finger3(6).p;
        % final_route_finger4(i,:) = Link_finger4(6).p;
        % final_route_finger5(i,:) = Link_finger5(6).p;
        % final_route_finger6(i,:) = Link_finger6(6).p;
        % final_route_finger7(i,:) = Link_finger7(6).p;
    end
    %     h_sphere = drawSphere(x, y, z, R_r);
    for i = 1:(T/time_gap + 1)
    %         p2 = plot3(final_route(:,1),final_route(:,2),final_route(:,3),'m-');
    %         hold on
    %         p3 = plot3(route_point(:,1),route_point(:,2),route_point(:,3),'b*');
    %         hold on
        h_sphere = drawSphere(x, y, z, R_r);
        if i~=T/time_gap + 1
            DHfk_fingers_Lnya_Mov_every_finger(th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,Trajectory_theta_finger0(i,1),Trajectory_theta_finger0(i,2),Trajectory_theta_finger0(i,3),Trajectory_theta_finger0(i,4),Trajectory_theta_finger0(i,5),Trajectory_theta_finger1(i,1),Trajectory_theta_finger1(i,2),Trajectory_theta_finger1(i,3),Trajectory_theta_finger1(i,4),Trajectory_theta_finger1(i,5),Trajectory_theta_finger2(i,1),Trajectory_theta_finger2(i,2),Trajectory_theta_finger2(i,3),Trajectory_theta_finger2(i,4),Trajectory_theta_finger2(i,5),Trajectory_theta_finger3(i,1),Trajectory_theta_finger3(i,2),Trajectory_theta_finger3(i,3),Trajectory_theta_finger3(i,4),Trajectory_theta_finger3(i,5),Trajectory_theta_finger4(i,1),Trajectory_theta_finger4(i,2),Trajectory_theta_finger4(i,3),Trajectory_theta_finger4(i,4),Trajectory_theta_finger4(i,5),Trajectory_theta_finger5(i,1),Trajectory_theta_finger5(i,2),Trajectory_theta_finger5(i,3),Trajectory_theta_finger5(i,4),Trajectory_theta_finger5(i,5),Trajectory_theta_finger6(i,1),Trajectory_theta_finger6(i,2),Trajectory_theta_finger6(i,3),Trajectory_theta_finger6(i,4),Trajectory_theta_finger6(i,5),Trajectory_theta_finger7(i,1),Trajectory_theta_finger7(i,2),Trajectory_theta_finger7(i,3),Trajectory_theta_finger7(i,4),Trajectory_theta_finger7(i,5),1); 
        else
            DHfk_fingers_Lnya_Mov_every_finger(th1_Final,th2_Final,th3_Final,th4_Final,th5_Final,th6_Final,Trajectory_theta_finger0(i,1),Trajectory_theta_finger0(i,2),Trajectory_theta_finger0(i,3),Trajectory_theta_finger0(i,4),Trajectory_theta_finger0(i,5),Trajectory_theta_finger1(i,1),Trajectory_theta_finger1(i,2),Trajectory_theta_finger1(i,3),Trajectory_theta_finger1(i,4),Trajectory_theta_finger1(i,5),Trajectory_theta_finger2(i,1),Trajectory_theta_finger2(i,2),Trajectory_theta_finger2(i,3),Trajectory_theta_finger2(i,4),Trajectory_theta_finger2(i,5),Trajectory_theta_finger3(i,1),Trajectory_theta_finger3(i,2),Trajectory_theta_finger3(i,3),Trajectory_theta_finger3(i,4),Trajectory_theta_finger3(i,5),Trajectory_theta_finger4(i,1),Trajectory_theta_finger4(i,2),Trajectory_theta_finger4(i,3),Trajectory_theta_finger4(i,4),Trajectory_theta_finger4(i,5),Trajectory_theta_finger5(i,1),Trajectory_theta_finger5(i,2),Trajectory_theta_finger5(i,3),Trajectory_theta_finger5(i,4),Trajectory_theta_finger5(i,5),Trajectory_theta_finger6(i,1),Trajectory_theta_finger6(i,2),Trajectory_theta_finger6(i,3),Trajectory_theta_finger6(i,4),Trajectory_theta_finger6(i,5),Trajectory_theta_finger7(i,1),Trajectory_theta_finger7(i,2),Trajectory_theta_finger7(i,3),Trajectory_theta_finger7(i,4),Trajectory_theta_finger7(i,5),0);
    %             C = min_p_pos + 50*unit_vec_tangent;
    %             start_point = min_p_pos;
    %             end_point = C;
    %             quiver3(start_point(1), start_point(2), start_point(3), end_point(1)-start_point(1), end_point(2)-start_point(2), end_point(3)-start_point(3), 'r');
    
        end
    end
    
    th1 = th1_Final;
    th2 = th2_Final;
    th3 = th3_Final;
    th4 = th4_Final;
    th5 = th5_Final;
    th6 = th6_Final;
    th1_finger0 = Trajectory_theta_finger0(end,1);
    th2_finger0 = Trajectory_theta_finger0(end,2);
    th3_finger0 = Trajectory_theta_finger0(end,3);
    th4_finger0 = Trajectory_theta_finger0(end,4);
    th5_finger0 = Trajectory_theta_finger0(end,5);
    th1_finger1 = Trajectory_theta_finger1(end,1);
    th2_finger1 = Trajectory_theta_finger1(end,2);  
    th3_finger1 = Trajectory_theta_finger1(end,3);
    th4_finger1 = Trajectory_theta_finger1(end,4);
    th5_finger1 = Trajectory_theta_finger1(end,5);
    th1_finger2 = Trajectory_theta_finger2(end,1);
    th2_finger2 = Trajectory_theta_finger2(end,2);
    th3_finger2 = Trajectory_theta_finger2(end,3);
    th4_finger2 = Trajectory_theta_finger2(end,4);
    th5_finger2 = Trajectory_theta_finger2(end,5);
    th1_finger3 = Trajectory_theta_finger3(end,1);
    th2_finger3 = Trajectory_theta_finger3(end,2);
    th3_finger3 = Trajectory_theta_finger3(end,3);
    th4_finger3 = Trajectory_theta_finger3(end,4);
    th5_finger3 = Trajectory_theta_finger3(end,5);
    th1_finger4 = Trajectory_theta_finger4(end,1);
    th2_finger4 = Trajectory_theta_finger4(end,2);
    th3_finger4 = Trajectory_theta_finger4(end,3);
    th4_finger4 = Trajectory_theta_finger4(end,4);
    th5_finger4 = Trajectory_theta_finger4(end,5);
    th1_finger5 = Trajectory_theta_finger5(end,1);
    th2_finger5 = Trajectory_theta_finger5(end,2);
    th3_finger5 = Trajectory_theta_finger5(end,3);
    th4_finger5 = Trajectory_theta_finger5(end,4);
    th5_finger5 = Trajectory_theta_finger5(end,5);
    th1_finger6 = Trajectory_theta_finger6(end,1);
    th2_finger6 = Trajectory_theta_finger6(end,2);
    th3_finger6 = Trajectory_theta_finger6(end,3);
    th4_finger6 = Trajectory_theta_finger6(end,4);
    th5_finger6 = Trajectory_theta_finger6(end,5);
    th1_finger7 = Trajectory_theta_finger7(end,1);
    th2_finger7 = Trajectory_theta_finger7(end,2);
    th3_finger7 = Trajectory_theta_finger7(end,3);
    th4_finger7 = Trajectory_theta_finger7(end,4);
    th5_finger7 = Trajectory_theta_finger7(end,5);
    

    
    %%一轮抓取结束后，将手指复位
    % th_finger0_1 =0;
    % th_finger0_2 =0;
    % th_finger0_3 =0;
    % th_finger0_4 =0;
    % th_finger0_5 =0;
    % DHfk_fingers_Lnya_Mov_calculate(th1,th2,th3,th4,th5,th6,th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5);
end