close all;
clear;
%绘图验证建模
figure; 
%初始角度
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
DHfk_finger0_Lnya(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5,0); 
% DHfk_finger1_Lnya(th_finger1_1,th_finger1_2,th_finger1_3,th_finger1_4,th_finger1_5,0); 
% DHfk_finger2_Lnya(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5,0); 
% DHfk_finger3_Lnya(th_finger3_1,th_finger3_2,th_finger3_3,th_finger3_4,th_finger3_5,0); 
% DHfk_finger4_Lnya(th_finger4_1,th_finger4_2,th_finger4_3,th_finger4_4,th_finger4_5,0); 
% DHfk_finger5_Lnya(th_finger5_1,th_finger5_2,th_finger5_3,th_finger5_4,th_finger5_5,0); 
% DHfk_finger6_Lnya(th_finger6_1,th_finger6_2,th_finger6_3,th_finger6_4,th_finger6_5,0); 
% DHfk_finger7_Lnya(th_finger7_1,th_finger7_2,th_finger7_3,th_finger7_4,th_finger7_5,0); 
hold on;
% h_sphere = [];
% h_points = [];
% while(1)
%     if ishandle(h_sphere)
%         delete(h_sphere);
%     end
%     if ishandle(h_points)
%         delete(h_points);
%     end
%     % 随机画球体
%     x = -10 + (10)*rand();
%     y = -10 + (10)*rand();
%     z = 300 + (50)*rand();
%     R = 15;
%     % 画球
% %     h_sphere = drawSphere(x, y, z, R);
%     P = [x;y;z];
%     A = [0;0;0];
%     [p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6, p_pos_7,P_1,h_points] = drawPointsOnSphere(P, R, A);
%     
%     %%切线的绘制测试     
%     % P_1是p_pos_0, p_pos_1, p_pos_2, p_pos_3, p_pos_4, p_pos_5, p_pos_6,p_pos_7所围成的圆的圆心
%     vec_p_pos_0P_1 = P_1 - p_pos_0;  % 计算向量P_1P和向量Pp_pos_0
%     vec_p_pos_0P = P - p_pos_0;   % 计算向量P_1P和向量Pp_pos_0
%     % 计算这两个向量的叉积，得到的向量就是基于点p_pos_0的平面P_1Pp_pos_0的法向量
%     vec_normal = cross(vec_p_pos_0P_1, vec_p_pos_0P);
%     % 计算向量p_pos_0P与法向量的叉积，得到的向量就是切线方向
%     vec_tangent = cross(vec_p_pos_0P, vec_normal);
%     % 将切线方向向量归一化，得到单位切线方向向量
%     unit_vec_tangent = vec_tangent / norm(vec_tangent);
%     % 计算旋转矩阵
%     % 选择一个与 unit_vec_tangent 不平行的向量
%     vec = [1; 0; 0];
%     if abs(dot(unit_vec_tangent, vec)) > 0.99  % 如果它们几乎平行，选择一个不同的向量
%         vec = [0; 1; 0];
%     end
%     
%     % 计算 vec 和 unit_vec_tangent 的叉积
%     vec_perp1 = cross(unit_vec_tangent, vec);
%     vec_perp1 = vec_perp1 / norm(vec_perp1);  % 归一化向量
%     
%     % 计算 vec_perp1 和 unit_vec_tangent 的叉积
%     vec_perp2 = cross(unit_vec_tangent, vec_perp1);
%     vec_perp2 = vec_perp2 / norm(vec_perp2);  % 归一化向量
%     
%     % 现在 unit_vec_tangent、vec_perp1 和 vec_perp2 形成了一个正交基
%     % 我们可以用它们来构造旋转矩阵
%     R = [unit_vec_tangent, vec_perp1, vec_perp2];
%     disp(R)
% 
% %     % 给定切线起始与末端位置，绘制切线
% %     C = p_pos_0 + 10*unit_vec_tangent;
% %     start_point = p_pos_0;
% %     end_point = C;
% %     quiver3(start_point(1), start_point(2), start_point(3), end_point(1)-start_point(1), end_point(2)-start_point(2), end_point(3)-start_point(3), 'r');
% 
%     % 延时1秒
%     pause(2);
% % end