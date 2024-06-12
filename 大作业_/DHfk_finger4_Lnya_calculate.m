function DHfk_finger4_Lnya_calculate(th1_finger4,th2_finger4,th3_finger4,th4_finger4,th5_finger4)

global Link_finger4

finger4_DH;



 Link_finger4(2).th=Link_finger4(2).th+th1_finger4*pi/180;
 Link_finger4(3).th=Link_finger4(3).th+th2_finger4*pi/180;
 Link_finger4(4).th=Link_finger4(4).th+th3_finger4*pi/180;
 Link_finger4(5).th=Link_finger4(5).th+th4_finger4*pi/180;
 Link_finger4(6).th=Link_finger4(6).th+th5_finger4*pi/180;


p0=[0,0,0]';


for i=1:6
Matrix_DH_finger4(i);
end


for i=2:6

      Link_finger4(i).A=Link_finger4(i-1).A*Link_finger4(i).A;
      Link_finger4(i).p= Link_finger4(i).A(:,4);
      Link_finger4(i).n= Link_finger4(i).A(:,1);
      Link_finger4(i).o= Link_finger4(i).A(:,2);
      Link_finger4(i).a= Link_finger4(i).A(:,3);
      Link_finger4(i).R=[Link_finger4(i).n(1:3),Link_finger4(i).o(1:3),Link_finger4(i).a(1:3)];
      
end
% view(125,52);
% set (gcf,'Position',[650,100,700,600])





