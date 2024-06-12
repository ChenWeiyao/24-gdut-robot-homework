function DHfk_finger2_Lnya_calculate(th1_finger2,th2_finger2,th3_finger2,th4_finger2,th5_finger2)

global Link_finger2

finger2_DH;

 Link_finger2(2).th=Link_finger2(2).th+th1_finger2*pi/180;
 Link_finger2(3).th=Link_finger2(3).th+th2_finger2*pi/180;
 Link_finger2(4).th=Link_finger2(4).th+th3_finger2*pi/180;
 Link_finger2(5).th=Link_finger2(5).th+th4_finger2*pi/180;
 Link_finger2(6).th=Link_finger2(6).th+th5_finger2*pi/180;


p0=[0,0,0]';
for i=1:6
Matrix_DH_finger2(i);
end
for i=2:6

      Link_finger2(i).A=Link_finger2(i-1).A*Link_finger2(i).A;
      Link_finger2(i).p= Link_finger2(i).A(:,4);
      Link_finger2(i).n= Link_finger2(i).A(:,1);
      Link_finger2(i).o= Link_finger2(i).A(:,2);
      Link_finger2(i).a= Link_finger2(i).A(:,3);
      Link_finger2(i).R=[Link_finger2(i).n(1:3),Link_finger2(i).o(1:3),Link_finger2(i).a(1:3)];
      
end
% view(125,52);
% set (gcf,'Position',[650,100,700,600])






