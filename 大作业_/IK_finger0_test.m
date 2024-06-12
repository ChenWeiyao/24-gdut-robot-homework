clc;
clear;
close all;
global Link_finger0
num=1;
ToDeg = 180/pi;
ToRad = pi/180;
%输入起始六关节位置
th_finger0_1=0;
th_finger0_2=0;
th_finger0_3=0;
th_finger0_4=0;
th_finger0_5=0;
%% 输入目标末端位置及旋转矩阵
% 通过计算球上的点位来给定期望位置
Tpos=[8.0008, -0.9767 ,  317.5991]';     %期望位置

rpos= [  -0.7522         0   -0.6589;
   -0.0020    1.0000    0.0023;
    0.6589    0.0031   -0.7522];
%% 绘制机械臂初始位姿及末端姿态
% 通过计算球上的点位来给定期望位置
DHfk_finger0_Lnya(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5,0); %绘制机械臂
plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); 
hold on;
view(-21,12); 
pause; 
cla;
% 
% %%机器人各关节运动范围
th1_min = 0;
th1_max = 40;
th2_min = -90;
th2_max = 90;
th3_min = -90;
th3_max = 0;
th4_min = -150;
th4_max = 0;
th5_min = -150;
th5_max = 0;
%%
tic
while (1)
    %% FK计算并绘制机器人，及目标点
   plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); 
   hold on;
    DHfk_finger0_Lnya(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5,1);
    %获取机械臂末端当前位置
    ex=Link_finger0(6).p(1);
    ey=Link_finger0(6).p(2);
    ez=Link_finger0(6).p(3);
    %% 计算误差
    R_now=Link_finger0(6).R;
    p_err =[Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%计算位置误差
    R_err=R_now'*rpos;%R_err   
    if R_err==eye(3)
        w_err=[0,0,0]'; 
         Loss = norm(p_err) + norm(w_err) ; %误差评价 
    else
        theta_R_err=acos((R_err(1,1)+R_err(2,2)+R_err(3,3)-1)/2);
        w_err=(theta_R_err/(2*sin(theta_R_err)))*[R_err(3,2)-R_err(2,3),R_err(1,3)-R_err(3,1),R_err(2,1)-R_err(1,2)]';
        Loss = norm(p_err) + norm(w_err) ; %误差评价 
    end
    %% 小于期望误差则结束迭代
    if Loss<3
        break;
    end
    %否则计算雅可比矩阵并计算角度修正量
    J=Jacobian5DoF_finger0(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5);    %计算雅可比矩阵
        learning_rate = min(Loss/10, 0.8); %自适应学习率，根据误差大小调整，最大不超过0.66
%         learning_rate =0.2;
    
        dth = learning_rate * pinv(J) * [p_err; w_err];  %计算修正量，此处单位为弧度
    th_finger0_1=th_finger0_1+dth(1)*ToDeg;
    th_finger0_1 = max(th1_min, min(th1_max, th_finger0_1));
    th_finger0_2=th_finger0_2+dth(2)*ToDeg;
    th_finger0_2 = max(th2_min, min(th2_max, th_finger0_2));
    th_finger0_3=th_finger0_3+dth(3)*ToDeg;
    th_finger0_3 = max(th3_min, min(th3_max, th_finger0_3));
    th_finger0_4=th_finger0_4+dth(4)*ToDeg;
    th_finger0_4 = max(th4_min, min(th4_max, th_finger0_4));
    th_finger0_5=th_finger0_5+dth(5)*ToDeg;
    th_finger0_5 = max(th5_min, min(th5_max, th_finger0_5));
    x(num)=ex;
    y(num)=ey;
    z(num)=ez;
    num=num+1;
    plot3(x,y,z,'r.');
%     trplot(rpos,'frame','1','length',200,'rgb','width',2); 
%     trplot(Link(7).A,'frame','2','length',200,'rgb','width',2); 
    grid on;
    hold on;    
    for i = 1:length(x)
    fprintf('Step %d: Link(6).p(1) = %f, Link(6).p(2) = %f, Link(6).p(3) = %f, Loss=%f\n', i, Link_finger0(6).p(1), Link_finger0(6).p(2), Link_finger0(6).p(3),Loss); 
    end
end
toc
%%再次绘制机器人保持图像
plot3(x,y,z,'r.');
grid on;
DHfk_finger0_Lnya(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5,0);
plot3(Tpos(1),Tpos(2),Tpos(3),'kX');
hold on;






 