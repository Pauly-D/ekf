%pre-process GPS
format long
%{
%get exact location for tower A: long survey (from RTKpost.txt)
%latA;
%longA;
%hA;

%import run thats trimmed for only the line movement A->B
%[xNED,yNED,zNED] = geodetic2ned(latitudedeg,longitudedeg,heightm,latitudedeg(10),longitudedeg(10),heightm(10),wgs84Ellipsoid);
%[xNED,yNED,zNED] = geodetic2ned(latitudedeg,longitudedeg,heightm,-33.956519396,18.463038259,110.7630,wgs84Ellipsoid);
%[xNED,yNED,zNED] = geodetic2ned(latitudedeg,longitudedeg,heightm,latA,longA,hA,wgs84Ellipsoid)
%}

%% true model
NEDtowerA = [0,0,1.71];
NEDtowerB = [12.77*cos(78*pi/180);12.77*sin(78*pi/180);1.70];
sag0 = 187.5e-4;
a=sag0;
%Parameterize power line
r0 = 0;
%horizontal span of power line use pythag on X and Y for tower B since
%tower A is the origin
r1 = sqrt((NEDtowerB(1)^2)+(NEDtowerB(2)^2));
%b and c constants
bC = inv([r0 1;r1 1])*[NEDtowerA(3)-a*r0^2;NEDtowerB(3)-a*r1^2];
b=bC(1);
c=bC(2);
%power line angle
psiL=atan((NEDtowerB(2)-NEDtowerA(2))/(NEDtowerB(1)-NEDtowerA(1)));

%generate TRUE power line (no noise)
r=0:0.0001:r1; %generate points along horizontal projection of line from tower A to tower B
x=r*sin(psiL);
y=r*cos(psiL);
z=a*r.^2+bC(1)*r+bC(2);  %generate heights along the line (z value)
%%
%generate GPS 'data' (use model but with a lot of white gaussian noise)
% generalize the jimmies taking in total time of test. distance covered etc
r=0:0.079:r1;
xGPS=awgn(r*sin(psiL),35); 
yGPS=awgn(r*cos(psiL),35); 
zGPS=awgn(a*r.^2+bC(1)*r+bC(2),30);

Tgps=1/5; %5Hz GPS receiver rate

for i=2:length(xGPS)-1
%synthesis of vgps measurement (a bit jimmy) but still probably better than
%vIMU
%vE(i) =  ((xGPS(i+1)-xGPS(i-1))/2*Tgps); %diff. current-previous/deltaT (symmetric difference quotient)
%vN(i) =  ((yGPS(i+1)-yGPS(i-1))/2*Tgps); %diff. current-previous/deltaT
vE(i) =  ((xGPS(i)-xGPS(i-1))/Tgps); %diff. current-previous/deltaT (symmetric difference quotient)
vN(i) =  ((yGPS(i)-yGPS(i-1))/Tgps); %diff. current-previous/deltaT
v(i) = vN(i)*cos(psiL) + vE(i)*sin(psiL); %project onto line

%synthesis of rgps measurement (line position)
dxE(i) = xGPS(i) - xGPS(1); %current position - origin
dyN(i) = yGPS(i) - yGPS(1); %current position - origin
r(i) = dyN(i)*cos(psiL) + dxE(i)*sin(psiL); %project onto line
end

v(length(xGPS))=0;

%% measurement vector
zk=zeros(length(xGPS)-1,3); %measurement vector (3 measurements)

%only GPS measurements
for i=1:length(zGPS)
    zk(i,1)=v(i); %GPS velocity pre-processed to give line direction velocity (mappped onto line)
    zk(i,2)=r(i); %GPS X and Y NED deltas projected onto line direction for line position r
    zk(i,3)=zGPS(i);%GPS height (referenced to zero at towerA so expect negative  
end

hold on      
grid on
scatter3(xGPS,yGPS,zGPS,'.b');
scatter3(zk(:,2)*sin(psiL),zk(:,2)*cos(psiL),zk(:,3),'.g');
scatter3(x,y,z,'.r');