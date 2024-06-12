% function DHfk_IRB120_Lnya(th1,th2,th3,th4,th5,th6,fcla)
function DHfk_IRB120_Lnya(th1,th2,th3,th4,th5,th6)
global Link_IRB120
IRB120_DH;
radius    = 25;  %25
len       = 60;  %60
joint_col = 0;
Link_IRB120(2).th=Link_IRB120(2).th+th1*pi/180;
Link_IRB120(3).th=Link_IRB120(3).th+th2*pi/180;
Link_IRB120(4).th=Link_IRB120(4).th+th3*pi/180;
Link_IRB120(5).th=Link_IRB120(5).th+th4*pi/180;
Link_IRB120(6).th=Link_IRB120(6).th+th5*pi/180;
Link_IRB120(7).th=Link_IRB120(7).th+th6*pi/180;    %for initial position
p0=[0,0,0]';
for i=1:7
Matrix_DH_IRB120(i);
end
for i=2:7

      Link_IRB120(i).A=Link_IRB120(i-1).A*Link_IRB120(i).A;
      Link_IRB120(i).p= Link_IRB120(i).A(:,4);
      Link_IRB120(i).n= Link_IRB120(i).A(:,1);
      Link_IRB120(i).o= Link_IRB120(i).A(:,2);
      Link_IRB120(i).a= Link_IRB120(i).A(:,3);
      Link_IRB120(i).R=[Link_IRB120(i).n(1:3),Link_IRB120(i).o(1:3),Link_IRB120(i).a(1:3)];

end




