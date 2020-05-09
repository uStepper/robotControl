L1o = 40;#40.0;
Zo = -70;#-77.0;
L_2 = 73.0;
Au = 188.0;
Al = 182.0;
Lo = 47.0;

UPPERARMLEN = Au;
LOWERARMLEN = Al;
XOFFSET = Lo;
ZOFFSET = L_2;
AZOFFSET = Zo;
AXOFFSET = L1o;

SHOULDEROFFSET = 142.0
ELBOWOFFSET = 45.0
GEARRATIO = 5.1
disp('Code implementation')
disp('Input angles:')
rot = -111.19
right = -412.10
left = -340.68

rot = deg2rad(rot/GEARRATIO);
left = deg2rad(((left-right)/GEARRATIO)+ELBOWOFFSET);
right = deg2rad((right/GEARRATIO)+SHOULDEROFFSET);


T1 = rot;#base
T2 = right;#shoulder
T3 = left;#elbow

#FW kinematics to get XYZ from angles:
disp('Calculated X, Y, Z:')
z = ZOFFSET + sin(right)*LOWERARMLEN - cos(left - (pi/2 - right))*UPPERARMLEN + AZOFFSET

k1 = sin(left - (pi/2 - right ))*UPPERARMLEN + cos(right)*LOWERARMLEN + XOFFSET + AXOFFSET ; 

x = cos(rot)*k1
y = sin(rot)*k1

##inverse kinematics to get angles from XYZ:

%x = 250
%y= 100
%z=0

rot = atan2(y,x);

x = x - cos(rot)*AXOFFSET;
y = y - sin(rot)*AXOFFSET;
z = z - AZOFFSET;

L1 = sqrt( x*x + y*y );

L2 = sqrt( (L1 - XOFFSET)*(L1 - XOFFSET) + (z - ZOFFSET)*(z - ZOFFSET) );

a = (z - ZOFFSET)/L2;
b = (L2*L2 + LOWERARMLEN*LOWERARMLEN - UPPERARMLEN*UPPERARMLEN)/(2*L2*LOWERARMLEN);
c = (LOWERARMLEN*LOWERARMLEN + UPPERARMLEN*UPPERARMLEN - L2*L2)/(2*LOWERARMLEN*UPPERARMLEN);

right = ( atan2(a, sqrt(1-(a*a)) ) + atan2(sqrt(1-(b*b)), b) );
left = atan2(sqrt(1-(c*c)), c);

right = rad2deg(right) - SHOULDEROFFSET;
left = rad2deg(left) - ELBOWOFFSET;

left = (left+right)*GEARRATIO;
right = right * GEARRATIO;
rot = rad2deg(rot) * GEARRATIO;

##output calculated angles
disp('Output angles:')
rot = rot
right = right
left = left