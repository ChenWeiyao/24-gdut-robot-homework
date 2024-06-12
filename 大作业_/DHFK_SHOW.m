clc
%% 正运动学演示
J1 = [-45,0];
J2 = [0,-60];
J3 = [-85,0];
J4 = [0,-90];
J5 = [0,-90];
J1_interp = linspace(J1(1), J1(2), 30);
J2_interp = linspace(J2(1), J2(2), 30);
J3_interp = linspace(J3(1), J3(2), 30);
J4_interp = linspace(J4(1), J4(2), 30);
J5_interp = linspace(J5(1), J5(2), 30);
% while (1)
%     for i = 1:30
%         DHfk_fingers_Lnya_Final(J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i), 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
%     end
%     for i = 29:-1:1
%         DHfk_fingers_Lnya_Final(J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i), 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
%     end
% DHfk_fingers_Lnya_Final(0,-10,-20,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
DHfk_fingers_Lnya(0,-10,-20,-30,-40,0)
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final(0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 % 
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,0,0,0,0,0,1);
 %    end
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),0,0,0,0,0,1);
 %    end
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final( 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),1);
 %    end
 %    for i = 1:30
 %        DHfk_fingers_Lnya_Final( J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),1);
 %    end
 %    for i = 29:-1:1
 %        DHfk_fingers_Lnya_Final( J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),J1_interp(i), J2_interp(i), J3_interp(i), J4_interp(i), J5_interp(i),1);
 %    end
 % end