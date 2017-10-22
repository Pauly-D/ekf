%----------------------------------------------------------------------------
%LINE PARAMETERIZATION (Using start and end points)

%{
%Initital Conditions
ORIGINtowerA = [lat0,long0,h0]; %from RTK accurate parameters Pref 
%(just use straight from RTK algorithm output)
NEDtowerA = [0,0,hA]; %on the ground measurements
NEDtowerB = geodetic2NED(lat,long,h,ORIGINtowerA); 
%}
%just for testing the output
%test October: span measured 12.77, Gmaps says 12.78. Sag from horizontal:
%0.805m

%% Check uncertainties
NEDtowerA = [0,0,1.71];
NEDtowerB = [12.77*cos(78*pi/180);12.77*sin(78*pi/180);1.70];
a = 187.5e-4;
%Parameterize power line
r0 = 0;
%horizontal span of power line use pythag on X and Y for tower B 
r1 = sqrt((NEDtowerB(1)^2)+(NEDtowerB(2)^2));
%b and c constants
bC = inv([r0 1;r1 1])*[NEDtowerA(3)-a*r0^2;NEDtowerB(3)-a*r1^2];
b=bC(1);
c=bC(2);

%line angle
psiL=atan((NEDtowerB(2)-NEDtowerA(2))/(NEDtowerB(1)-NEDtowerA(1)));

%generate TRUE power line (no noise)
r=0:0.0001:r1; %generate points along horizontal projection of line from tower A to tower B
x=r*sin(psiL);
y=r*cos(psiL);
z=a*r.^2+bC(1)*r+bC(2);  %generate heights along the line (z value)


%generate GPS 'data' (use model but with a lot of white gaussian noise)
r=0:0.0076:r1;
xGPS=awgn(r*sin(psiL),60); 
yGPS=awgn(r*cos(psiL),60); 
zGPS=awgn(a*r.^2+bC(1)*r+bC(2),30);

%[xNEDg,yNEDg,zNEDg] = geodetic2ned(latitudedeg,longitudedeg,heightm,,longA,hA,wgs84Ellipsoid);

for i=2:length(xGPS)
%project velcotiy componenets onto line using line geometry 
vE(i) =  ((xGPS(i)-xGPS(i-1))/0.2); %diff. current-previous/deltaT
vN(i) =  ((yGPS(i)-yGPS(i-1))/0.2); %diff. current-previous/deltaT
v(i) = vN(i)*cos(psiL) + vE(i)*sin(psiL); 

%synthesis of rgps measurement (line position)
%origin = towerA: [0,0,0]
dxE(i) = xGPS(i) - xGPS(1); %current position - origin
dyN(i) = yGPS(i) - yGPS(1); %current position - origin
%project onto line using line geometry
r(i) = dyN(i)*cos(psiL) + dxE(i)*sin(psiL);  
end


%{
r=0:0.0076:r1;
xGPS=r*sin(psiL)+rand*0.05; 
yGPS=r*cos(psiL)+rand*0.05;
zGPS=a*r.^2+bC(1)*r+bC(2)+rand*0.1;
%}

hold all
grid on
scatter3(x,y,z,'.r');
%plot3(xGPS,yGPS,zGPS);
%scatter3(xGPS,yGPS,zGPS,'b');
%}

%{
%% up uncertainty 5cm (most extreme cases)
NEDtowerA2 = [0-0.1,0+0.1,1.71-0.05];
NEDtowerB2 = [12.77*cos(78*pi/180);12.77*sin(78*pi/180);1.70-0.05];

%Parameterize power line
r0 = 0;
%horizontal span of power line use pythag on X and Y for tower B since
%tower A is the origin
r1 = sqrt((NEDtowerB2(1)^2)+(NEDtowerB2(2)^2));
%b and c constants
bC = inv([r0 1;r1 1])*[NEDtowerA2(3)-a*r0^2;NEDtowerB2(3)-a*r1^2];
b=bC(1);
c=bC(2);

%power line angle
psiL=atan((NEDtowerB2(2)-NEDtowerA2(2))/(NEDtowerB2(1)-NEDtowerA2(1)));

%generate TRUE power line (no noise)
r=0:0.01:r1; %generate points along horizontal projection of line from tower A to tower B
x=r*sin(psiL);
y=r*cos(psiL);
z=a*r.^2+bC(1)*r+bC(2);  %generate heights along the line (z value)

hold all
grid on
scatter3(x,y,z,'.b');


%% N-E uncertainty
NEDtowerA3 = [0+0.02,0+0.02,1.71];
NEDtowerB3 = [12.77*cos(78*pi/180)-0.02;12.77*sin(78*pi/180)+0.02;1.70];
%Parameterize power line
r0 = 0;
%horizontal span of power line use pythag on X and Y for tower B since
%tower A is the origin
r1 = sqrt((NEDtowerB3(1)^2)+(NEDtowerB3(2)^2));
%b and c constants
bC = inv([r0 1;r1 1])*[NEDtowerA3(3)-a*r0^2;NEDtowerB3(3)-a*r1^2];
b=bC(1);
c=bC(2);

%power line angle
psiL=atan((NEDtowerB3(2)-NEDtowerA3(2))/(NEDtowerB3(1)-NEDtowerA3(1)));

%generate TRUE power line (no noise)
r=0:0.01:r1; %generate points along horizontal projection of line from tower A to tower B
x=r*sin(psiL);
y=r*cos(psiL);
z=a*r.^2+bC(1)*r+bC(2);  %generate heights along the line (z value)

hold all
grid on
scatter3(x,y,z,'.k');

%% N-E and up uncertainty
NEDtowerA4 = [0-0.02,0-0.02,1.71+0.05];
NEDtowerB4 = [12.77*cos(78*pi/180)-0.02;12.77*sin(78*pi/180)+0.02;1.70-0.05];
%Parameterize power line
r0 = 0;
%horizontal span of power line use pythag on X and Y for tower B since
%tower A is the origin
r1 = sqrt((NEDtowerB4(1)^2)+(NEDtowerB4(2)^2));
%b and c constants
bC = inv([r0 1;r1 1])*[NEDtowerA4(3)-a*r0^2;NEDtowerB4(3)-a*r1^2];
b=bC(1);
c=bC(2);

%power line angle
psiL=atan((NEDtowerB4(2)-NEDtowerA4(2))/(NEDtowerB4(1)-NEDtowerA4(1)));

%generate TRUE power line (no noise)
r=0:0.01:r1; %generate points along horizontal projection of line from tower A to tower B
x=r*sin(psiL);
y=r*cos(psiL);
z=a*r.^2+bC(1)*r+bC(2);  %generate heights along the line (z value)

hold all
grid on
scatter3(x,y,z,'.g');
%% 
%}







