function [th1,th2,th3,th4,th5,th6] = IK_IRB120(Tpos,rpos,th1,th2,th3,th4,th5,th6,th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5)
    global Link_IRB120
    global Link_finger0
    num=1;
    ToDeg = 180/pi;
    ToRad = pi/180;
    %% 计算机械臂初始位姿及末端姿态
    DHfk_fingers_Lnya_Mov_calculate(th1,th2,th3,th4,th5,th6,th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5);
    % %%机器人各关节运动范围
    th1_min = -165;
    th1_max = 165;
    th2_min = -110;
    th2_max = 110;
    th3_min = -110;
    th3_max = 70;
    th4_min = -180;
    th4_max = 180;
    th5_min = -90;
    th5_max = 90;
    th6_min = -180;
    th6_max = 180;
    tic
    while (1)
        if toc > 2
            th1=0;
            th2=0;
            th3=0;
            th4=0;
            th5=20;
            break;
        end
        %% FK计算并绘制机器人，及目标点
        DHfk_fingers_Lnya_Mov_calculate(th1,th2,th3,th4,th5,th6,th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5);
        %获取机械臂末端当前位置
        ex=Link_IRB120(7).p(1);
        ey=Link_IRB120(7).p(2);
        ez=Link_IRB120(7).p(3);
        % ex=Link_finger0(1).p(1);
        % ey=Link_finger0(1).p(2);
        % ez=Link_finger0(1).p(3);
        %% 计算误差
        R_now=Link_IRB120(7).R;
        % R_now=Link_finger0(1).R;
        p_err =[Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%计算位置误差
        R_err=R_now'*rpos;%R_err 
        if R_err==eye(3)
            w_err=[0,0,0]'; 
             Loss = norm(p_err) + norm(w_err) ; %误差评价 
             Loss_pos = norm(p_err);
        else
            theta_R_err=acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
            w_err=(theta_R_err/(2*sin(theta_R_err)))*[R_err(3,2)-R_err(2,3),R_err(1,3)-R_err(3,1),R_err(2,1)-R_err(1,2)]';
            Loss = norm(p_err) + norm(w_err) ; %误差评价 
            Loss_pos = norm(p_err);
        end
        %% 小于期望误差则结束迭代
        if Loss_pos < 1 && Loss < 5
            break;
        end
        %否则计算雅可比矩阵并计算角度修正量
        J=Jacobian6DoF_Ln(th1,th2,th3,th4,th5,th6);    %计算雅可比矩阵
            %%判断奇异
            D = det(J);
            if D == 0
                fprintf('D= %2.4f ',D); 
                fprintf('\n');
                fprintf('Pass the singilarity !'); 
                fprintf('\n');
                pause;
            end
            learning_rate = min(Loss, 0.99); %自适应学习率，根据误差大小调整，最大不超过0.66
%             learning_rate =0.5;
            dth = learning_rate * pinv(J) * [p_err; w_err];  %计算修正量，此处单位为弧度
        th1=th1+dth(1)*ToDeg;
        th1 = max(th1_min, min(th1_max, th1));
        th2=th2+dth(2)*ToDeg;
        th2 = max(th2_min, min(th2_max, th2));
        th3=th3+dth(3)*ToDeg;
        th3 = max(th3_min, min(th3_max, th3));
        th4=th4+dth(4)*ToDeg;
        th4 = max(th4_min, min(th4_max, th4));
        th5=th5+dth(5)*ToDeg;
        th5 = max(th5_min, min(th5_max, th5));
        th6=th6+dth(6)*ToDeg;
        th6 = max(th6_min, min(th6_max, th6));
        x(num)=ex;
        y(num)=ey;
        z(num)=ez;
        num=num+1;
        for i = 1:length(x)
             fprintf('Step %d: Loss_pos=%f\n', i, Loss_pos);  
        end
    end
    fprintf('收敛成功！\n');
    DHfk_fingers_Lnya_Mov_calculate(th1,th2,th3,th4,th5,th6,th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5);

    
    
    
    
    

