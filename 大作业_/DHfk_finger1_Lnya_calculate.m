function DHfk_finger1_Lnya_calculate(th1_finger1,th2_finger1,th3_finger1,th4_finger1,th5_finger1)
global Link_finger1
finger1_DH;
Link_finger1(2).th=Link_finger1(2).th+th1_finger1*pi/180;
Link_finger1(3).th=Link_finger1(3).th+th2_finger1*pi/180;
Link_finger1(4).th=Link_finger1(4).th+th3_finger1*pi/180;
Link_finger1(5).th=Link_finger1(5).th+th4_finger1*pi/180;
Link_finger1(6).th=Link_finger1(6).th+th5_finger1*pi/180;
p0=[0,0,0]';
for i=1:6
Matrix_DH_finger1(i);
end
for i=2:6
      Link_finger1(i).A=Link_finger1(i-1).A*Link_finger1(i).A;
      Link_finger1(i).p= Link_finger1(i).A(:,4);
      Link_finger1(i).n= Link_finger1(i).A(:,1);
      Link_finger1(i).o= Link_finger1(i).A(:,2);
      Link_finger1(i).a= Link_finger1(i).A(:,3);
      Link_finger1(i).R=[Link_finger1(i).n(1:3),Link_finger1(i).o(1:3),Link_finger1(i).a(1:3)];
end





