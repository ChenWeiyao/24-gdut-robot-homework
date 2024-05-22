%%%仅食指工作空间

close all;

clear;

global Link_shizhi
shizhi_DH;

th=[0,0,0,0,0];
grid on;


for th2= -45:10:15
   for th3=-90:10:20
      for th4=-120:20:0
         for th5=-90:10:0
                           
             
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

