function DHfk_finger0_Lnya_calculate(th1_finger0,th2_finger0,th3_finger0,th4_finger0,th5_finger0)
global Link_finger0
finger0_DH;
Link_finger0(2).th=Link_finger0(2).th+th1_finger0*pi/180;
Link_finger0(3).th=Link_finger0(3).th+th2_finger0*pi/180;
Link_finger0(4).th=Link_finger0(4).th+th3_finger0*pi/180;
Link_finger0(5).th=Link_finger0(5).th+th4_finger0*pi/180;
Link_finger0(6).th=Link_finger0(6).th+th5_finger0*pi/180;
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
end





