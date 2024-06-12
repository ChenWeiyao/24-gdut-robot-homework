%RRT路径规划，输入：初始角度q_Init，目标角度q_Goal，关节角度范围q_Min,q_Max,球形障碍物数据:球体中心点center，球的半径R，圆柱体连杆的半径r，最大循环次数maxIter，步长stepSize，关节数量joint_nums
function [route_num,route_theta] = RRT_Route_Points_Get_IRB120(q_fingers,q_Init,q_Goal,q_Min,q_Max,center,R,r,maxIter,stepSize,joint_nums)
    global Link_IRB120;
    global Link_finger0;
    global Link_finger1;
    global Link_finger2;
    global Link_finger3;
    global Link_finger4;
    global Link_finger5;
    global Link_finger6;
    global Link_finger7;
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
        DHfk_fingers_Lnya_Mov_calculate(q_New(1),q_New(2),q_New(3),q_New(4),q_New(5),q_New(6),q_fingers(1),q_fingers(2),q_fingers(3),q_fingers(4),q_fingers(5)); %正运动学计算
        %% 碰撞检测
        for i=1:joint_nums    
            C(i) = Collision_detection (Link_IRB120(i).p(1:3),Link_IRB120(i+1).p(1:3), center,R+r);
        end
        for i=joint_nums+1:joint_nums+5
            C(i) = Collision_detection (Link_finger0(i-joint_nums).p(1:3),Link_finger0(i-joint_nums+1).p(1:3), center,R+r);
        end
        for i=joint_nums+6:joint_nums+10
            C(i) = Collision_detection (Link_finger1(i-joint_nums-5).p(1:3),Link_finger1(i-joint_nums-5+1).p(1:3), center,R+r);
        end
        for i=joint_nums+11:joint_nums+15
            C(i) = Collision_detection (Link_finger2(i-joint_nums-10).p(1:3),Link_finger2(i-joint_nums-10+1).p(1:3), center,R+r);
        end
        for i=joint_nums+16:joint_nums+20
            C(i) = Collision_detection (Link_finger3(i-joint_nums-15).p(1:3),Link_finger3(i-joint_nums-15+1).p(1:3), center,R+r);
        end
        for i=joint_nums+21:joint_nums+25
            C(i) = Collision_detection (Link_finger4(i-joint_nums-20).p(1:3),Link_finger4(i-joint_nums-20+1).p(1:3), center,R+r);
        end
        for i=joint_nums+26:joint_nums+30
            C(i) = Collision_detection (Link_finger5(i-joint_nums-25).p(1:3),Link_finger5(i-joint_nums-25+1).p(1:3), center,R+r);
        end
        for i=joint_nums+31:joint_nums+35
            C(i) = Collision_detection (Link_finger6(i-joint_nums-30).p(1:3),Link_finger6(i-joint_nums-30+1).p(1:3), center,R+r);
        end
        for i=joint_nums+36:joint_nums+40
            C(i) = Collision_detection (Link_finger7(i-joint_nums-35).p(1:3),Link_finger7(i-joint_nums-35+1).p(1:3), center,R+r);
        end
        % flag = C(1)+C(2)+C(3)+C(4)+C(5)+C(6)+C(7)+C(8)+C(9)+C(10)+C(11)+C(12)+C(13)+C(14)+C(15)+C(16)+C(17)+C(18)+C(19)+C(20)+C(21)+C(22)+C(23)+C(24)+C(25)+C(26)+C(27)+C(28)+C(29)+C(30)+C(31)+C(32)+C(33)+C(34)+C(35)+C(36)+C(37)+C(38)+C(39)+C(40);
        
        flag = C(1)+C(2)+C(3)+C(4)+C(7)+C(9)+C(10)+C(11)+C(12)+C(14)+C(15)+C(16)+C(17)+C(19)+C(20)+C(21)+C(22)+C(25)+C(26)+C(27)+C(29)+C(30)+C(31)+C(32)+C(34)+C(35)+C(36)+C(37)+C(39)+C(40);
        fprintf('flag=%d\n',flag);
        % flag = 0;
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
                    DHfk_IRB120_Lnya(RRT_angle(i,1),RRT_angle(i,2),RRT_angle(i,3),RRT_angle(i,4),RRT_angle(i,5),RRT_angle(i,6)); %正运动学计算
            end
            

    end
    %输出：route_num是路径点的数量，RRT_angle是一个矩阵，每一行是一个路径点的关节角度

    RRT_angle = [RRT_angle; q_Goal];
    route_num = size(RRT_angle,1);
    route_theta = RRT_angle;