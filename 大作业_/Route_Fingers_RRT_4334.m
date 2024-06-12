function [Trajectory_theta_finger0,Trajectory_theta_finger1,Trajectory_theta_finger2,Trajectory_theta_finger3,Trajectory_theta_finger4,Trajectory_theta_finger5,Trajectory_theta_finger6,Trajectory_theta_finger7] = Route_Fingers_RRT_4334(P,A,R_r,pos_now_0_finger0,pos_now_0_finger1,pos_now_0_finger2,pos_now_0_finger3,pos_now_0_finger4,pos_now_0_finger5,pos_now_0_finger6,pos_now_0_finger7,th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7)

    % 绘制球体上的点
    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger0);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger0 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    %各关节求逆解
    vec_p_posP_1_finger0 = P_1 - p_pos{1};  
    vec_p_posP_finger0 = P - p_pos{1};   
    vec_normal_finger0 = cross(vec_p_posP_1_finger0, vec_p_posP_finger0);
    vec_tangent_finger0 = cross(vec_p_posP_finger0, vec_normal_finger0);
    unit_vec_tangent_finger0 = vec_tangent_finger0 / norm(vec_tangent_finger0);
    vec_finger0 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger0, vec_finger0)) > 0.99  
        vec_finger0 = [0; 1; 0];
    end
    vec_perp1_finger0 = cross(unit_vec_tangent_finger0, vec_finger0);
    vec_perp1_finger0 = vec_perp1_finger0 / norm(vec_perp1_finger0); 
    vec_perp2_finger0 = cross(unit_vec_tangent_finger0, vec_perp1_finger0);
    vec_perp2_finger0 = vec_perp2_finger0 / norm(vec_perp2_finger0); 
    R_33_finger0 = [unit_vec_tangent_finger0, vec_perp1_finger0, vec_perp2_finger0];
    [th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final] = IK_finger0_Irb(p_pos{1},R_33_finger0,th1,th2,th3,th4,th5,th6,th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);


    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger1);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger1 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    vec_p_posP_1_finger1 = P_1 - p_pos{1};
    vec_p_posP_finger1 = P - p_pos{1};
    vec_normal_finger1 = cross(vec_p_posP_1_finger1, vec_p_posP_finger1);
    vec_tangent_finger1 = cross(vec_p_posP_finger1, vec_normal_finger1);
    unit_vec_tangent_finger1 = vec_tangent_finger1 / norm(vec_tangent_finger1);
    vec_finger1 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger1, vec_finger1)) > 0.99  
        vec_finger1 = [0; 1; 0];
    end
    vec_perp1_finger1 = cross(unit_vec_tangent_finger1, vec_finger1);
    vec_perp1_finger1 = vec_perp1_finger1 / norm(vec_perp1_finger1);
    vec_perp2_finger1 = cross(unit_vec_tangent_finger1, vec_perp1_finger1);
    vec_perp2_finger1 = vec_perp2_finger1 / norm(vec_perp2_finger1);
    R_33_finger1 = [unit_vec_tangent_finger1, vec_perp1_finger1, vec_perp2_finger1];
    [th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final] = IK_finger1_Irb(p_pos{1},R_33_finger1,th1,th2,th3,th4,th5,th6,th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);


    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger2);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger2 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    vec_p_posP_1_finger2 = P_1 - p_pos{1};
    vec_p_posP_finger2 = P - p_pos{1};
    vec_normal_finger2 = cross(vec_p_posP_1_finger2, vec_p_posP_finger2);
    vec_tangent_finger2 = cross(vec_p_posP_finger2, vec_normal_finger2);
    unit_vec_tangent_finger2 = vec_tangent_finger2 / norm(vec_tangent_finger2);
    vec_finger2 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger2, vec_finger2)) > 0.99  
        vec_finger2 = [0; 1; 0];
    end
    vec_perp1_finger2 = cross(unit_vec_tangent_finger2, vec_finger2);
    vec_perp1_finger2 = vec_perp1_finger2 / norm(vec_perp1_finger2);
    vec_perp2_finger2 = cross(unit_vec_tangent_finger2, vec_perp1_finger2);
    vec_perp2_finger2 = vec_perp2_finger2 / norm(vec_perp2_finger2);
    R_33_finger2 = [unit_vec_tangent_finger2, vec_perp1_finger2, vec_perp2_finger2];
    [th_finger2_1_final,th_finger2_2_final,th_finger2_3_final,th_finger2_4_final,th_finger2_5_final] = IK_finger2_Irb(p_pos{1},R_33_finger2,th1,th2,th3,th4,th5,th6,th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final,th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);


    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger3);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger3 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    vec_p_posP_1_finger3 = P_1 - p_pos{1};
    vec_p_posP_finger3 = P - p_pos{1};
    vec_normal_finger3 = cross(vec_p_posP_1_finger3, vec_p_posP_finger3);
    vec_tangent_finger3 = cross(vec_p_posP_finger3, vec_normal_finger3);
    unit_vec_tangent_finger3 = vec_tangent_finger3 / norm(vec_tangent_finger3);
    vec_finger3 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger3, vec_finger3)) > 0.99  
        vec_finger3 = [0; 1; 0];
    end
    vec_perp1_finger3 = cross(unit_vec_tangent_finger3, vec_finger3);
    vec_perp1_finger3 = vec_perp1_finger3 / norm(vec_perp1_finger3);
    vec_perp2_finger3 = cross(unit_vec_tangent_finger3, vec_perp1_finger3);
    vec_perp2_finger3 = vec_perp2_finger3 / norm(vec_perp2_finger3);
    R_33_finger3 = [unit_vec_tangent_finger3, vec_perp1_finger3, vec_perp2_finger3];
    [th_finger3_1_final,th_finger3_2_final,th_finger3_3_final,th_finger3_4_final,th_finger3_5_final] = IK_finger3_Irb(p_pos{1},R_33_finger3,th1,th2,th3,th4,th5,th6,th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final,th_finger2_1_final,th_finger2_2_final,th_finger2_3_final,th_finger2_4_final,th_finger2_5_final,th1_finger3,th2_finger3,th3_finger3,th4_finger3,th5_finger3,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);


    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger4);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger4 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    vec_p_posP_1_finger4 = P_1 - p_pos{1};
    vec_p_posP_finger4 = P - p_pos{1};
    vec_normal_finger4 = cross(vec_p_posP_1_finger4, vec_p_posP_finger4);
    vec_tangent_finger4 = cross(vec_p_posP_finger4, vec_normal_finger4);
    unit_vec_tangent_finger4 = vec_tangent_finger4 / norm(vec_tangent_finger4);
    vec_finger4 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger4, vec_finger4)) > 0.99  
        vec_finger4 = [0; 1; 0];
    end
    vec_perp1_finger4 = cross(unit_vec_tangent_finger4, vec_finger4);
    vec_perp1_finger4 = vec_perp1_finger4 / norm(vec_perp1_finger4);
    vec_perp2_finger4 = cross(unit_vec_tangent_finger4, vec_perp1_finger4);
    vec_perp2_finger4 = vec_perp2_finger4 / norm(vec_perp2_finger4);
    R_33_finger4 = [unit_vec_tangent_finger4, vec_perp1_finger4, vec_perp2_finger4];
    [th_finger4_1_final,th_finger4_2_final,th_finger4_3_final,th_finger4_4_final,th_finger4_5_final] = IK_finger4_Irb(p_pos{1},R_33_finger4,th1,th2,th3,th4,th5,th6,th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final,th_finger2_1_final,th_finger2_2_final,th_finger2_3_final,th_finger2_4_final,th_finger2_5_final,th_finger3_1_final,th_finger3_2_final,th_finger3_3_final,th_finger3_4_final,th_finger3_5_final,th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);


    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger5);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger5 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    vec_p_posP_1_finger5 = P_1 - p_pos{1};
    vec_p_posP_finger5 = P - p_pos{1};
    vec_normal_finger5 = cross(vec_p_posP_1_finger5, vec_p_posP_finger5);
    vec_tangent_finger5 = cross(vec_p_posP_finger5, vec_normal_finger5);
    unit_vec_tangent_finger5 = vec_tangent_finger5 / norm(vec_tangent_finger5);
    vec_finger5 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger5, vec_finger5)) > 0.99  
        vec_finger5 = [0; 1; 0];
    end
    vec_perp1_finger5 = cross(unit_vec_tangent_finger5, vec_finger5);
    vec_perp1_finger5 = vec_perp1_finger5 / norm(vec_perp1_finger5);
    vec_perp2_finger5 = cross(unit_vec_tangent_finger5, vec_perp1_finger5);
    vec_perp2_finger5 = vec_perp2_finger5 / norm(vec_perp2_finger5);
    R_33_finger5 = [unit_vec_tangent_finger5, vec_perp1_finger5, vec_perp2_finger5];
    [th_finger5_1_final,th_finger5_2_final,th_finger5_3_final,th_finger5_4_final,th_finger5_5_final] = IK_finger5_Irb(p_pos{1},R_33_finger5,th1,th2,th3,th4,th5,th6,th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final,th_finger2_1_final,th_finger2_2_final,th_finger2_3_final,th_finger2_4_final,th_finger2_5_final,th_finger3_1_final,th_finger3_2_final,th_finger3_3_final,th_finger3_4_final,th_finger3_5_final,th_finger4_1_final,th_finger4_2_final,th_finger4_3_final,th_finger4_4_final,th_finger4_5_final,th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);


    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger6);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger6 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    vec_p_posP_1_finger6 = P_1 - p_pos{1};
    vec_p_posP_finger6 = P - p_pos{1};
    vec_normal_finger6 = cross(vec_p_posP_1_finger6, vec_p_posP_finger6);
    vec_tangent_finger6 = cross(vec_p_posP_finger6, vec_normal_finger6);
    unit_vec_tangent_finger6 = vec_tangent_finger6 / norm(vec_tangent_finger6);
    vec_finger6 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger6, vec_finger6)) > 0.99  
        vec_finger6 = [0; 1; 0];
    end
    vec_perp1_finger6 = cross(unit_vec_tangent_finger6, vec_finger6);
    vec_perp1_finger6 = vec_perp1_finger6 / norm(vec_perp1_finger6);
    vec_perp2_finger6 = cross(unit_vec_tangent_finger6, vec_perp1_finger6);
    vec_perp2_finger6 = vec_perp2_finger6 / norm(vec_perp2_finger6);
    R_33_finger6 = [unit_vec_tangent_finger6, vec_perp1_finger6, vec_perp2_finger6];
    [th_finger6_1_final,th_finger6_2_final,th_finger6_3_final,th_finger6_4_final,th_finger6_5_final] = IK_finger6_Irb(p_pos{1},R_33_finger6,th1,th2,th3,th4,th5,th6,th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final,th_finger2_1_final,th_finger2_2_final,th_finger2_3_final,th_finger2_4_final,th_finger2_5_final,th_finger3_1_final,th_finger3_2_final,th_finger3_3_final,th_finger3_4_final,th_finger3_5_final,th_finger4_1_final,th_finger4_2_final,th_finger4_3_final,th_finger4_4_final,th_finger4_5_final,th_finger5_1_final,th_finger5_2_final,th_finger5_3_final,th_finger5_4_final,th_finger5_5_final,th1_finger6,th2_finger6,th3_finger6,th4_finger6,th5_finger6,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);


    [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7, P_1, h_points] = drawPointsOnSphere(P, R_r, A, pos_now_0_finger7);
    p_pos = {p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7};
    % 使用 cellfun 计算每个点到给定位置的距离
    distances = cellfun(@(p) norm(pos_now_0_finger7 - p), p_pos);
    min_distance = inf;
    % 找到最小距离及其索引
    [min_distance, min_index] = min(distances);
    % 修正：直接使用 min_index 作为索引
    if min_index > 1
        part1 = p_pos(min_index:end); % 从最近点到数组末尾
        part2 = p_pos(1:min_index-1); % 从数组开始到最近点之前
    else
        part1 = p_pos(min_index:end); % 如果 min_index 是 1，整个数组都是 part1
        part2 = []; % 没有 part2
    end
    % 重新组合数组
    p_pos = [part1, part2];
    vec_p_posP_1_finger7 = P_1 - p_pos{1};
    vec_p_posP_finger7 = P - p_pos{1};
    vec_normal_finger7 = cross(vec_p_posP_1_finger7, vec_p_posP_finger7);
    vec_tangent_finger7 = cross(vec_p_posP_finger7, vec_normal_finger7);
    unit_vec_tangent_finger7 = vec_tangent_finger7 / norm(vec_tangent_finger7);
    vec_finger7 = [1; 0; 0];
    if abs(dot(unit_vec_tangent_finger7, vec_finger7)) > 0.99  
        vec_finger7 = [0; 1; 0];
    end
    vec_perp1_finger7 = cross(unit_vec_tangent_finger7, vec_finger7);
    vec_perp1_finger7 = vec_perp1_finger7 / norm(vec_perp1_finger7);
    vec_perp2_finger7 = cross(unit_vec_tangent_finger7, vec_perp1_finger7);
    vec_perp2_finger7 = vec_perp2_finger7 / norm(vec_perp2_finger7);
    R_33_finger7 = [unit_vec_tangent_finger7, vec_perp1_finger7, vec_perp2_finger7];
    [th_finger7_1_final,th_finger7_2_final,th_finger7_3_final,th_finger7_4_final,th_finger7_5_final] = IK_finger7_Irb(p_pos{1},R_33_finger7,th1,th2,th3,th4,th5,th6,th_finger0_1_final,th_finger0_2_final,th_finger0_3_final,th_finger0_4_final,th_finger0_5_final,th_finger1_1_final,th_finger1_2_final,th_finger1_3_final,th_finger1_4_final,th_finger1_5_final,th_finger2_1_final,th_finger2_2_final,th_finger2_3_final,th_finger2_4_final,th_finger2_5_final,th_finger3_1_final,th_finger3_2_final,th_finger3_3_final,th_finger3_4_final,th_finger3_5_final,th_finger4_1_final,th_finger4_2_final,th_finger4_3_final,th_finger4_4_final,th_finger4_5_final,th_finger5_1_final,th_finger5_2_final,th_finger5_3_final,th_finger5_4_final,th_finger5_5_final,th_finger6_1_final,th_finger6_2_final,th_finger6_3_final,th_finger6_4_final,th_finger6_5_final,th1_finger7,th2_finger7,th3_finger7,th4_finger7,th5_finger7);

    %RRT规划路径点参数
    J1 = [0,45];
    J2 = [-90,90];
    J3 = [90,0];
    J4 = [-150,0];
    J5 = [-150,0];
    q_Min = [J1(1), J2(1), J3(1), J4(1), J5(1)];
    q_Max = [J1(2), J2(2), J3(2), J4(2), J5(2)];
    q_Init = [0, 0, 0, 0, 0];
    q_Goal_finger0 = [th_finger0_1_final, th_finger0_2_final, th_finger0_3_final, th_finger0_4_final, th_finger0_5_final];
    q_Goal_finger1 = [th_finger1_1_final, th_finger1_2_final, th_finger1_3_final, th_finger1_4_final, th_finger1_5_final];
    q_Goal_finger2 = [th_finger2_1_final, th_finger2_2_final, th_finger2_3_final, th_finger2_4_final, th_finger2_5_final];
    q_Goal_finger3 = [th_finger3_1_final, th_finger3_2_final, th_finger3_3_final, th_finger3_4_final, th_finger3_5_final];
    q_Goal_finger4 = [th_finger4_1_final, th_finger4_2_final, th_finger4_3_final, th_finger4_4_final, th_finger4_5_final];
    q_Goal_finger5 = [th_finger5_1_final, th_finger5_2_final, th_finger5_3_final, th_finger5_4_final, th_finger5_5_final];
    q_Goal_finger6 = [th_finger6_1_final, th_finger6_2_final, th_finger6_3_final, th_finger6_4_final, th_finger6_5_final];
    q_Goal_finger7 = [th_finger7_1_final, th_finger7_2_final, th_finger7_3_final, th_finger7_4_final, th_finger7_5_final];


    %球心坐标为P，半径为R_r,
    r_link = 5;  %连杆半径
    joint_nums = 5;  %关节数量
    maxIter = 10000;  %最大循环次数
    stepSize =40;  %步长
    [route_num_finger0, route_theta_finger0] = RRT_Route_Points_Get(q_Init, q_Goal_finger0, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    [route_num_finger1, route_theta_finger1] = RRT_Route_Points_Get(q_Init, q_Goal_finger1, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    [route_num_finger2, route_theta_finger2] = RRT_Route_Points_Get(q_Init, q_Goal_finger2, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    [route_num_finger3, route_theta_finger3] = RRT_Route_Points_Get(q_Init, q_Goal_finger3, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    [route_num_finger4, route_theta_finger4] = RRT_Route_Points_Get(q_Init, q_Goal_finger4, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    [route_num_finger5, route_theta_finger5] = RRT_Route_Points_Get(q_Init, q_Goal_finger5, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    [route_num_finger6, route_theta_finger6] = RRT_Route_Points_Get(q_Init, q_Goal_finger6, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);
    [route_num_finger7, route_theta_finger7] = RRT_Route_Points_Get(q_Init, q_Goal_finger7, q_Min, q_Max, P, R_r, r_link, maxIter, stepSize, joint_nums);

    %当规划出来的路径点无法用于4334轨迹规划时，对路径点进行插值
    while route_num_finger0 <= 4
        new_route_theta_finger0 = [];
        for i = 1:route_num_finger0-1
            new_route_theta_finger0 = [new_route_theta_finger0; route_theta_finger0(i, :); (route_theta_finger0(i, :) + route_theta_finger0(i+1, :))/2];
        end
        new_route_theta_finger0 = [new_route_theta_finger0; route_theta_finger0(route_num_finger0, :)];
        route_num_finger0 = size(new_route_theta_finger0, 1);
        route_theta_finger0 = new_route_theta_finger0;
    end

    while route_num_finger1 <= 4
        new_route_theta_finger1 = [];
        for i = 1:route_num_finger1-1
            new_route_theta_finger1 = [new_route_theta_finger1; route_theta_finger1(i, :); (route_theta_finger1(i, :) + route_theta_finger1(i+1, :))/2];
        end
        new_route_theta_finger1 = [new_route_theta_finger1; route_theta_finger1(route_num_finger1, :)];
        route_num_finger1 = size(new_route_theta_finger1, 1);
        route_theta_finger1 = new_route_theta_finger1;
    end

    while route_num_finger2 <= 4
        new_route_theta_finger2 = [];
        for i = 1:route_num_finger2-1
            new_route_theta_finger2 = [new_route_theta_finger2; route_theta_finger2(i, :); (route_theta_finger2(i, :) + route_theta_finger2(i+1, :))/2];
        end
        new_route_theta_finger2 = [new_route_theta_finger2; route_theta_finger2(route_num_finger2, :)];
        route_num_finger2 = size(new_route_theta_finger2, 1);
        route_theta_finger2 = new_route_theta_finger2;
    end

    while route_num_finger3 <= 4
        new_route_theta_finger3 = [];
        for i = 1:route_num_finger3-1
            new_route_theta_finger3 = [new_route_theta_finger3; route_theta_finger3(i, :); (route_theta_finger3(i, :) + route_theta_finger3(i+1, :))/2];
        end
        new_route_theta_finger3 = [new_route_theta_finger3; route_theta_finger3(route_num_finger3, :)];
        route_num_finger3 = size(new_route_theta_finger3, 1);
        route_theta_finger3 = new_route_theta_finger3;
    end

    while route_num_finger4 <= 4
        new_route_theta_finger4 = [];
        for i = 1:route_num_finger4-1
            new_route_theta_finger4 = [new_route_theta_finger4; route_theta_finger4(i, :); (route_theta_finger4(i, :) + route_theta_finger4(i+1, :))/2];
        end
        new_route_theta_finger4 = [new_route_theta_finger4; route_theta_finger4(route_num_finger4, :)];
        route_num_finger4 = size(new_route_theta_finger4, 1);
        route_theta_finger4 = new_route_theta_finger4;
    end

    while route_num_finger5 <= 4
        new_route_theta_finger5 = [];
        for i = 1:route_num_finger5-1
            new_route_theta_finger5 = [new_route_theta_finger5; route_theta_finger5(i, :); (route_theta_finger5(i, :) + route_theta_finger5(i+1, :))/2];
        end
        new_route_theta_finger5 = [new_route_theta_finger5; route_theta_finger5(route_num_finger5, :)];
        route_num_finger5 = size(new_route_theta_finger5, 1);
        route_theta_finger5 = new_route_theta_finger5;
    end

    while route_num_finger6 <= 4
        new_route_theta_finger6 = [];
        for i = 1:route_num_finger6-1
            new_route_theta_finger6 = [new_route_theta_finger6; route_theta_finger6(i, :); (route_theta_finger6(i, :) + route_theta_finger6(i+1, :))/2];
        end
        new_route_theta_finger6 = [new_route_theta_finger6; route_theta_finger6(route_num_finger6, :)];
        route_num_finger6 = size(new_route_theta_finger6, 1);
        route_theta_finger6 = new_route_theta_finger6;
    end

    while route_num_finger7 <= 4
        new_route_theta_finger7 = [];
        for i = 1:route_num_finger7-1
            new_route_theta_finger7 = [new_route_theta_finger7; route_theta_finger7(i, :); (route_theta_finger7(i, :) + route_theta_finger7(i+1, :))/2];
        end
        new_route_theta_finger7 = [new_route_theta_finger7; route_theta_finger7(route_num_finger7, :)];
        route_num_finger7 = size(new_route_theta_finger7, 1);
        route_theta_finger7 = new_route_theta_finger7;
    end

    %如果route_num_finger1-7不一样大，以最小的为准，将其他对应的route_theta在保留首尾两行的基础上，删去中间的行，使得route_num_finger1-7一样大
    route_num_fingers = [route_num_finger0, route_num_finger1, route_num_finger2, route_num_finger3, route_num_finger4, route_num_finger5, route_num_finger6, route_num_finger7];
    % 找到最小行数
    min_rows = min(route_num_fingers);
    % 对每个route_theta_finger矩阵进行调整
    for i = 0:7
        eval(['current_matrix = route_theta_finger', num2str(i), ';']);
        if size(current_matrix, 1) > min_rows
            % 保留首尾两行，删除中间的行
            middle_rows = linspace(2, size(current_matrix, 1)-1, size(current_matrix, 1)-2);
            rows_to_keep = [1, round(middle_rows(1:(end-(size(current_matrix, 1)-min_rows)))), size(current_matrix, 1)];
            eval(['route_theta_finger', num2str(i), ' = current_matrix(rows_to_keep, :);']);
        end
    end


    T = 5;
    time_gap = 0.1;

    for joint_num = 1:5
        [Trajectory_theta_finger0(:,joint_num), Trajectory_speed_finger0(:,joint_num), Trajectory_acceleration_finger0(:,joint_num), t] = Trajectory4334(route_theta_finger0, T, joint_num, time_gap);
    end

    for joint_num = 1:5
        [Trajectory_theta_finger1(:,joint_num), Trajectory_speed_finger1(:,joint_num), Trajectory_acceleration_finger1(:,joint_num), t] = Trajectory4334(route_theta_finger1, T, joint_num, time_gap);
    end

    for joint_num = 1:5
        [Trajectory_theta_finger2(:,joint_num), Trajectory_speed_finger2(:,joint_num), Trajectory_acceleration_finger2(:,joint_num), t] = Trajectory4334(route_theta_finger2, T, joint_num, time_gap);
    end

    for joint_num = 1:5
        [Trajectory_theta_finger3(:,joint_num), Trajectory_speed_finger3(:,joint_num), Trajectory_acceleration_finger3(:,joint_num), t] = Trajectory4334(route_theta_finger3, T, joint_num, time_gap);
    end

    for joint_num = 1:5
        [Trajectory_theta_finger4(:,joint_num), Trajectory_speed_finger4(:,joint_num), Trajectory_acceleration_finger4(:,joint_num), t] = Trajectory4334(route_theta_finger4, T, joint_num, time_gap);
    end

    for joint_num = 1:5
        [Trajectory_theta_finger5(:,joint_num), Trajectory_speed_finger5(:,joint_num), Trajectory_acceleration_finger5(:,joint_num), t] = Trajectory4334(route_theta_finger5, T, joint_num, time_gap);
    end 

    for joint_num = 1:5
        [Trajectory_theta_finger6(:,joint_num), Trajectory_speed_finger6(:,joint_num), Trajectory_acceleration_finger6(:,joint_num), t] = Trajectory4334(route_theta_finger6, T, joint_num, time_gap);
    end

    for joint_num = 1:5
        [Trajectory_theta_finger7(:,joint_num), Trajectory_speed_finger7(:,joint_num), Trajectory_acceleration_finger7(:,joint_num), t] = Trajectory4334(route_theta_finger7, T, joint_num, time_gap);
    end

    %正运动学放函数外做，要重写这部分的正运动学

    % for i = 1:(T/time_gap + 1)
    %     DHfk_finger0_Lnya_calculate(Trajectory_theta(i,1),Trajectory_theta(i,2),Trajectory_theta(i,3),Trajectory_theta(i,4),Trajectory_theta(i,5));

    %     final_route(i,:) = Link_finger0(6).p;
    % end