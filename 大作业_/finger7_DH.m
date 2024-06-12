%拇指的DH建模
ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';



%单位：mm
Link_finger7= struct('name','Body' , 'th', 0, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ); % DH表关节字典  theta  d  a  alpha

Link_finger7(1)= struct('name','Base' , 'th', 315*ToRad, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ); %BASE to 1

Link_finger7(2) = struct('name','J1' , 'th', 90*ToRad, 'dz', 40, 'dx',0, 'alf',180*ToRad,'az',UZ); %1 TO 2 

Link_finger7(3) = struct('name','J2' , 'th', 45*ToRad, 'dz', 0, 'dx',0, 'alf',90*ToRad,'az',UZ); %2 TO 3

Link_finger7(4) = struct('name','J3' , 'th', 0*ToRad, 'dz',0, 'dx', 50, 'alf',0*ToRad,'az',UZ); %3 TO 4

Link_finger7(5) = struct('name','J4' , 'th', 0*ToRad, 'dz', 0, 'dx', 40, 'alf',0*ToRad,'az',UZ); %4 TO 5

Link_finger7(6) = struct('name','J5' , 'th', 0*ToRad, 'dz', 0, 'dx', 30, 'alf',0,'az',UZ); %5 TO 食指指尖






