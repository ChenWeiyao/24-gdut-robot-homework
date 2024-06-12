function DHfk_finger5_Lnya(th1_finger5,th2_finger5,th3_finger5,th4_finger5,th5_finger5,fcla)

global Link_finger5

finger5_DH;
%画圆柱体
radius    = 5;  %25
len       = 20;  %60
joint_col = 0;


plot3(0,0,0,'ro'); 


 Link_finger5(2).th=Link_finger5(2).th+th1_finger5*pi/180;
 Link_finger5(3).th=Link_finger5(3).th+th2_finger5*pi/180;
 Link_finger5(4).th=Link_finger5(4).th+th3_finger5*pi/180;
 Link_finger5(5).th=Link_finger5(5).th+th4_finger5*pi/180;
 Link_finger5(6).th=Link_finger5(6).th+th5_finger5*pi/180;


p0=[0,0,0]';


for i=1:6
Matrix_DH_finger5(i);
end


for i=2:6

      Link_finger5(i).A=Link_finger5(i-1).A*Link_finger5(i).A;
      Link_finger5(i).p= Link_finger5(i).A(:,4);
      Link_finger5(i).n= Link_finger5(i).A(:,1);
      Link_finger5(i).o= Link_finger5(i).A(:,2);
      Link_finger5(i).a= Link_finger5(i).A(:,3);
      Link_finger5(i).R=[Link_finger5(i).n(1:3),Link_finger5(i).o(1:3),Link_finger5(i).a(1:3)];
      Connect3D(Link_finger5(i-1).p,Link_finger5(i).p,'b',2);
      hold on;
      plot3(Link_finger5(i).p(1),Link_finger5(i).p(2),Link_finger5(i).p(3),'rx');
      hold on;
      if i<=6
          DrawCylinder(Link_finger5(i-1).p, Link_finger5(i-1).R * Link_finger5(i).az, radius,len, joint_col); hold on;
      end 
end
% view(125,52);
% set (gcf,'Position',[650,100,700,600])
axis([-400,400,-400,400,-200,500]);
xlabel('x');
ylabel('y'); 
zlabel('z');
grid on;
drawnow;
if(fcla)
    cla;
end




