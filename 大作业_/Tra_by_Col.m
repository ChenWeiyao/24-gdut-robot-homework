close all;
clear;
clc;
figure; 
%初始角度
global Link_finger0
global Link_finger1
global Link_finger2
global Link_finger3
global Link_finger4
global Link_finger5
global Link_finger6
global Link_finger7
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
DHfk_finger0_Lnya_calculate(th_finger0_1,th_finger0_2,th_finger0_3,th_finger0_4,th_finger0_5); 
DHfk_finger1_Lnya_calculate(th_finger1_1,th_finger1_2,th_finger1_3,th_finger1_4,th_finger1_5); 
DHfk_finger2_Lnya_calculate(th_finger2_1,th_finger2_2,th_finger2_3,th_finger2_4,th_finger2_5); 
DHfk_finger3_Lnya_calculate(th_finger3_1,th_finger3_2,th_finger3_3,th_finger3_4,th_finger3_5); 
DHfk_finger4_Lnya_calculate(th_finger4_1,th_finger4_2,th_finger4_3,th_finger4_4,th_finger4_5); 
DHfk_finger5_Lnya_calculate(th_finger5_1,th_finger5_2,th_finger5_3,th_finger5_4,th_finger5_5); 
DHfk_finger6_Lnya_calculate(th_finger6_1,th_finger6_2,th_finger6_3,th_finger6_4,th_finger6_5); 
DHfk_finger7_Lnya_calculate(th_finger7_1,th_finger7_2,th_finger7_3,th_finger7_4,th_finger7_5); 
for th1 = -40 : 10 :0
    for th2= -90:15:0
         for th3=-90:15:0
             for th4=-150:30:0
                 for th5=-150:30:0
                                     
                            Link_finger0(2).th=0*pi+th1*pi/180;
                            Link_finger0(3).th=0*pi+th2*pi/180;      
                            Link_finger0(4).th=0*pi+th3*pi/180;  
                            Link_finger0(5).th=0*pi+th4*pi/180;
                            Link_finger0(6).th=0*pi+th5*pi/180;
                            Link_finger1(2).th=0*pi+th1*pi/180;
                            Link_finger1(3).th=0*pi+th2*pi/180;
                            Link_finger1(4).th=0*pi+th3*pi/180;
                            Link_finger1(5).th=0*pi+th4*pi/180;
                            Link_finger1(6).th=0*pi+th5*pi/180;
                            Link_finger2(2).th=0*pi+th1*pi/180;
                            Link_finger2(3).th=0*pi+th2*pi/180;

                            
                        for i=1:6
                            Matrix_DH_finger0(i);
                        end
                        for i=2:6
                            Link_finger0(i).A=Link_finger0(i-1).A*Link_finger0(i).A;
                            Link_finger0(i).p= Link_finger0(i).A(:,4); 
                        end
                       
                 end
             end
        end
    end
end