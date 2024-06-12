%RRT路径规划，输入：初始角度q_Init，目标角度q_Goal，关节角度范围q_Min,q_Max,球形障碍物数据:球体中心点center，球的半径R，圆柱体连杆的半径r，最大循环次数maxIter，步长stepSize，关节数量joint_nums
function [route_num,route_theta] = RRT_Route_Points_Get(q_Init,q_Goal,q_Min,q_Max,center,R,r,maxIter,stepSize,joint_nums)
    global Link_finger0;
    flag=0;         	% 碰撞标志
    % 构建RRT树  
    tree = [q_Init, -1]; % 初始节点，关节角为初始关节角，parent为-1
    for i = 1:maxIter
        % 随机采样
        probability=rand(1); 
        if probability>0.1
        q_Rand = q_Goal;    % 目标节点设置为样本点
        else
        q_Rand = q_Min + rand(1,joint_nums).*(q_Max - q_Min); % 随机生成的样本点
        end
        % 寻找距离样本点最近节点
        distances = sum((tree(:,1:joint_nums) - q_Rand).^2, 2);   
        [~, min_Index] = min(distances);
        q_Near = tree(min_Index, 1:joint_nums); 

        % 扩展节点
        q_New = q_Near + stepSize*(q_Rand - q_Near)/norm(q_Rand - q_Near);
        % 正运动学求解
        DHfk_finger0_Lnya_calculate(q_New(1),q_New(2),q_New(3),q_New(4),q_New(5)); %正运动学计算,有几个自由度就写到几
        for i=3:joint_nums    
            C(i) = Collision_detection (Link_finger0(i).p(1:3),Link_finger0(i+1).p(1:3), center,R+r);
        end
        flag=C(3)+C(4)+C(5);
        fprintf('在找了在找了')
        if flag==0
            % 添加新节点到树中
            newNode = [q_New, min_Index];
            tree = [tree; newNode];
            % 如果新节点接近目标状态，则结束规划
            if norm(q_New - q_Goal) < stepSize
                fprintf("结束规划\n");
                break;
            end
        end
    end

    if norm(q_New - q_Goal) > stepSize
        disp('未搜索到RRT路径!');
    else
            %% 从目标状态反向搜索路径
            % 记录一条路径
            tree_num=size(tree,1);
            path_size=1;
            path(path_size,1:joint_nums+1)=tree(tree_num,:);
            mark=path(1,joint_nums+1);
            
            while mark~=-1
                path_size=path_size+1;
                path(path_size,:)=tree(mark,:);
                mark=path(path_size,joint_nums+1);
            end
            size_path=size(path,1);
            %% 反向储存该路径
            for i=1:size_path
            RRT_angle(i,1:joint_nums)=path(size_path-i+1,1:joint_nums);
            end
           
            for i=1:size_path
                % 正运动学求解
                    DHfk_finger0_Lnya_calculate(RRT_angle(i,1),RRT_angle(i,2),RRT_angle(i,3),RRT_angle(i,4),RRT_angle(i,5)); %正运动学计算
%                     RRT_point(i,1:3)=Link_finger0(6).p(1:3);
            end
            %% 显示路径
            % %% 创建一个不可见的图窗
            % f = figure('visible', 'off');
            % 
            % % 绘制初始点和目标点
            % plot3(q_Init(3), q_Init(4), q_Init(5), 'bo', 'MarkerSize', 5); 
            % hold on;
            % plot3(q_Goal(3), q_Goal(4), q_Goal(5), 'b*', 'MarkerSize', 5); 
            % hold on;
            % 
            % % 使用红色线将tree中的点连起来
            % plot3(tree(:,3), tree(:,4), tree(:,5), 'r-', 'LineWidth', 1.5);
            % hold on;
            % % 使用红色线将RRT_angle中的点连起来
            % plot3(RRT_angle(:,3), RRT_angle(:,4), RRT_angle(:,5), 'r-', 'LineWidth', 1.5);
            % 
            % J1 = [-45,45];
            % J2 = [-90,0];
            % J3 = [-90,0];
            % J4 = [-90,0];
            % J5 = [-90,0];
            % grid on;
            % axis([J3(1),J3(2),J4(1),J4(2),J5(1),J5(2)]);
            % xlabel('q3');
            % ylabel('q4');
            % zlabel('q5');
            % 
            % % 保存图像到文件，这里假设保存为PNG格式，您可以根据需要选择其他格式
            % saveas(f, 'RRT_Path.png');
            % % 或者使用print函数
            % % print(f, 'RRT_Path.png', '-dpng');
            % 
            % % 关闭图窗
            % close(f);
            
    end
    %输出：route_num是路径点的数量，RRT_angle是一个矩阵，每一行是一个路径点的关节角度

    RRT_angle = [RRT_angle; q_Goal];
    route_num = size(RRT_angle,1);
    route_theta = RRT_angle;