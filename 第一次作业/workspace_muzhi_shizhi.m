% %%%食指工作空间
% 
close all;

clear;

global Link_shizhi
shizhi_DH;

th=[0,0,0,0,0];
grid on;


for th2= -45:15:15
   for th3=-90:22:20
      for th4=-120:20:0
         for th5=-90:15:0
                           
             
                Link_shizhi(3).th=0*pi+th2*pi/180;
                Link_shizhi(4).th=0*pi+th3*pi/180;  
                Link_shizhi(5).th=0*pi+th4*pi/180;
                Link_shizhi(6).th=0*pi+th5*pi/180;

                fprintf('%d %d %d %d %d \n',[0,th2,th3,th4,th5]');
            for i=1:6
                Matrix_DH_shizhi(i);
            end
            for i=2:6
                Link_shizhi(i).A=Link_shizhi(i-1).A*Link_shizhi(i).A;
                Link_shizhi(i).p= Link_shizhi(i).A(:,4); 
            end
            grid on;  
            plot3(Link_shizhi(6).p(1),Link_shizhi(6).p(2),Link_shizhi(6).p(3),'r*');
            pause(0.0001);
            hold on;
                    
          end
       end
   end
end



%%%%拇指工作空间



global Link_muzhi
muzhi_DH
for th2= -15:10:35
     for th3=-60:20:0
         for th4=-50:10:0
             for th5=-90:15:0
                                 
                        
                        Link_muzhi(3).th=0*pi+th2*pi/180;      
                        Link_muzhi(4).th=0*pi+th3*pi/180;  
                        Link_muzhi(5).th=0*pi+th4*pi/180;
                        Link_muzhi(6).th=0*pi+th5*pi/180;
                        
                        fprintf('%d %d %d %d %d %d  \n',[0,th2,th3,th4,th5]');
                    for i=1:6
                        Matrix_DH_muzhi(i);
                    end
                    for i=2:6
                        Link_muzhi(i).A=Link_muzhi(i-1).A*Link_muzhi(i).A;
                        Link_muzhi(i).p= Link_muzhi(i).A(:,4); 
                    end
                    grid on;  
                    plot3(Link_muzhi(6).p(1),Link_muzhi(6).p(2),Link_muzhi(6).p(3),'b*');
                    pause(0.0001);
                    hold on;
                
             end
         end
    end
end