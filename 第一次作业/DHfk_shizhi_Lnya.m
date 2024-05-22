function DHfk_shizhi_Lnya(th1,th2,th3,th4,th5,fcla)

global Link_shizhi

shizhi_DH;
%画圆柱体
radius    = 5;  %25
len       = 40;  %60
joint_col = 0;


plot3(0,0,0,'ro'); 


 Link_shizhi(2).th=Link_shizhi(2).th+th1*pi/180;
 Link_shizhi(3).th=Link_shizhi(3).th+th2*pi/180;
 Link_shizhi(4).th=Link_shizhi(4).th+th3*pi/180;
 Link_shizhi(5).th=Link_shizhi(5).th+th4*pi/180;
 Link_shizhi(6).th=Link_shizhi(6).th+th5*pi/180;
 

p0=[0,0,0]';


for i=1:6
Matrix_DH_shizhi(i);
end


for i=2:6

      Link_shizhi(i).A=Link_shizhi(i-1).A*Link_shizhi(i).A;
      Link_shizhi(i).p= Link_shizhi(i).A(:,4);
      Link_shizhi(i).n= Link_shizhi(i).A(:,1);
      Link_shizhi(i).o= Link_shizhi(i).A(:,2);
      Link_shizhi(i).a= Link_shizhi(i).A(:,3);
      Link_shizhi(i).R=[Link_shizhi(i).n(1:3),Link_shizhi(i).o(1:3),Link_shizhi(i).a(1:3)];
      Connect3D(Link_shizhi(i-1).p,Link_shizhi(i).p,'b',2);
      hold on;
      plot3(Link_shizhi(i).p(1),Link_shizhi(i).p(2),Link_shizhi(i).p(3),'rx');
      hold on;
      if i<=6
          DrawCylinder(Link_shizhi(i-1).p, Link_shizhi(i-1).R * Link_shizhi(i).az, radius,len, joint_col); hold on;
      end 
end
% view(125,52);
% set (gcf,'Position',[650,100,700,600])
axis([-700,700,-700,700,-400,1000]);
xlabel('x');
ylabel('y'); 
zlabel('z');
grid on;
drawnow;
if(fcla)
    cla;
end




