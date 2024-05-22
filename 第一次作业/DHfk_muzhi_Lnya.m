function DHfk_muzhi_Lnya(th1_muzhi,th2_muzhi,th3_muzhi,th4_muzhi,th5_muzhi,fcla)

global Link_muzhi

muzhi_DH;
%画圆柱体
radius    = 5;  %25
len       = 40;  %60
joint_col = 0;


plot3(0,0,0,'ro'); 


 Link_muzhi(2).th=Link_muzhi(2).th+th1_muzhi*pi/180;
 Link_muzhi(3).th=Link_muzhi(3).th+th2_muzhi*pi/180;
 Link_muzhi(4).th=Link_muzhi(4).th+th3_muzhi*pi/180;
 Link_muzhi(5).th=Link_muzhi(5).th+th4_muzhi*pi/180;
 Link_muzhi(6).th=Link_muzhi(6).th+th5_muzhi*pi/180;


p0=[0,0,0]';


for i=1:6
Matrix_DH_muzhi(i);
end


for i=2:6

      Link_muzhi(i).A=Link_muzhi(i-1).A*Link_muzhi(i).A;
      Link_muzhi(i).p= Link_muzhi(i).A(:,4);
      Link_muzhi(i).n= Link_muzhi(i).A(:,1);
      Link_muzhi(i).o= Link_muzhi(i).A(:,2);
      Link_muzhi(i).a= Link_muzhi(i).A(:,3);
      Link_muzhi(i).R=[Link_muzhi(i).n(1:3),Link_muzhi(i).o(1:3),Link_muzhi(i).a(1:3)];
      Connect3D(Link_muzhi(i-1).p,Link_muzhi(i).p,'b',2);
      hold on;
      plot3(Link_muzhi(i).p(1),Link_muzhi(i).p(2),Link_muzhi(i).p(3),'rx');
      hold on;
      if i<=6
          DrawCylinder(Link_muzhi(i-1).p, Link_muzhi(i-1).R * Link_muzhi(i).az, radius,len, joint_col); hold on;
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




