%% 手指的工作空间绘制
clear;

global Link_finger0
finger0_DH

th=[0,0,0,0,0];
grid on;

for th1 = 0 : 10 :40
    for th2= -90:15:0
         for th3=-90:15:0
             for th4=-150:30:0
                 for th5=-150:30:0
                                     
                            Link_finger0(2).th=0*pi+th1*pi/180;
                            Link_finger0(3).th=0*pi+th2*pi/180;      
                            Link_finger0(4).th=0*pi+th3*pi/180;  
                            Link_finger0(5).th=0*pi+th4*pi/180;
                            Link_finger0(6).th=0*pi+th5*pi/180;
                            
                        for i=1:6
                            Matrix_DH_finger0(i);
                        end
                        for i=2:6
                            Link_finger0(i).A=Link_finger0(i-1).A*Link_finger0(i).A;
                            Link_finger0(i).p= Link_finger0(i).A(:,4); 
                        end
                        grid on;  
                        plot3(Link_finger0(6).p(1),Link_finger0(6).p(2),Link_finger0(6).p(3),'b*');
                        pause(0.0001);
                        hold on;
                    
                 end
             end
        end
    end
end
