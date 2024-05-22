close all;
clear;
%绘图验证建模
figure; 
%初始角度
th1=0;
th2=0;
th3=0;
th4=0;
th5=0;
th1_muzhi=0;
th2_muzhi=0;
th3_muzhi=0;
th4_muzhi=0;
th5_muzhi=0;

stp=10;
dtime=0.02;
DHfk_shizhi_Lnya(th1,th2,th3,th4,th5,0); 
DHfk_muzhi_Lnya(th1_muzhi,th2_muzhi,th3_muzhi,th4_muzhi,th5_muzhi,0)
view(125,52);
