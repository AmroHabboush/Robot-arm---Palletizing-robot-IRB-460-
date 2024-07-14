clear all

%we define the links in (mm)(theta,d,a,alpha,offset)
L(1)=Link([pi/2,   742.5,   -627, 0]);
L(1).m = 400
L(2)=Link([0,  0,  945,  175]); 
L(2).m = 250
L(3)=Link([0, 0 , 1024, -130]);
L(3).m = 160
L(4)=Link([pi/2, 492,  0, 0]);
L(3).m = 110

%We build the robot

IRB460 = SerialLink(L, 'name', 'IRB460');

IRB460  %here we check the DH table
 
%forward kinematics (home position)

a = [pi/2 0 0 pi/2]
IRB460.fkine(a)
IRB460.plot(a)

%Forward Kinematics (desired Position)
q = [pi 0.2706*pi -0.127*pi 0];
IRB460.fkine(q)
%visualization
IRB460.plot(q), 'workspace'

%inverse kinematics
q = [pi 0.2706*pi -0.127*pi 0];
T = IRB460.fkine(q)

qi = IRB460.ikine(T,'mask',[0,0,1,1,1,1]);
%'Number of robot DOF must be >= the number of 1s in the mask matrix')
qi % result

%Jacobian
qj = [pi 0.2706*pi -0.127*pi 0];
J = IRB460.jacobe(qj)
rank(J)


%dynamics matrix
IRB460.dyn

%simulation and animation
IRB460.teach
