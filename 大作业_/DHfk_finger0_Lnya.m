function DHfk_finger0_Lnya(th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0,fcla)

global Link_finger0

finger0_DH;  
 

% finger0_DH_mov(2.2255,450.1929, 444.1936,-11.1293);
radius    = 5;  %25
len       = 20;  %60
joint_col = 0;


plot3(0,0,0,'ro'); 


 Link_finger0(2).th=Link_finger0(2).th+th1_finger0*pi/180;
 Link_finger0(3).th=Link_finger0(3).th+th2_finger0*pi/180;
 Link_finger0(4).th=Link_finger0(4).th+th3_finger0*pi/180;
 Link_finger0(5).th=Link_finger0(5).th+th4_finger0*pi/180;
 Link_finger0(6).th=Link_finger0(6).th+th5_finger0*pi/180;


p0=[0,0,0]';


for i=1:6
Matrix_DH_finger0(i);
end


for i=2:6

      Link_finger0(i).A=Link_finger0(i-1).A*Link_finger0(i).A;
      Link_finger0(i).p= Link_finger0(i).A(:,4);
      Link_finger0(i).n= Link_finger0(i).A(:,1);
      Link_finger0(i).o= Link_finger0(i).A(:,2);
      Link_finger0(i).a= Link_finger0(i).A(:,3);
      Link_finger0(i).R=[Link_finger0(i).n(1:3),Link_finger0(i).o(1:3),Link_finger0(i).a(1:3)];
      Connect3D(Link_finger0(i-1).p,Link_finger0(i).p,'b',2);
      hold on;
      plot3(Link_finger0(i).p(1),Link_finger0(i).p(2),Link_finger0(i).p(3),'rx');
      hold on;
      if i<=6
          DrawCylinder(Link_finger0(i-1).p, Link_finger0(i-1).R * Link_finger0(i).az, radius,len, joint_col); hold on;
      end 
      %加一个画小正方体的，在末端画一个正方体
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




