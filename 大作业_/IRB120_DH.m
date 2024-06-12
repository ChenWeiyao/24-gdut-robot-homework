% %Build Robot by D_H methods


ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
global Link_IRB120;
Link_IRB120   = struct('name','Body', 'th',  0,        'dz', 0,   'dx', 0,  'alf',0*ToRad,'az',UZ);     % az 
Link_IRB120(1)= struct('name','Base', 'th',  0*ToRad,  'dz', 0,   'dx', 0,  'alf',0*ToRad,'az',UZ);        %BASE to 1
Link_IRB120(2) = struct('name','J1' , 'th',  90*ToRad, 'dz', 103, 'dx', 0,  'alf',90*ToRad,'az',UZ);       %1 TO 2 
Link_IRB120(3) = struct('name','J2' , 'th',  90*ToRad, 'dz', 0,   'dx', 270,'alf',0*ToRad,'az',UZ);    %2 TO 3
Link_IRB120(4) = struct('name','J3' , 'th',  0*ToRad,  'dz', 0,   'dx', 70, 'alf',90*ToRad,'az',UZ);          %3 TO 4
Link_IRB120(5) = struct('name','J4' , 'th',  0*ToRad,  'dz', 374, 'dx', 0,  'alf',-90*ToRad,'az',UZ);          %4 TO 5
Link_IRB120(6) = struct('name','J5' , 'th',  0*ToRad,  'dz', 0,   'dx', 0,  'alf',90*ToRad,'az',UZ);          %5 TO 6
Link_IRB120(7) = struct('name','J6' , 'th',  0*ToRad,  'dz', 0,   'dx', 0,  'alf',0*ToRad,'az',UZ);          %6 TO E