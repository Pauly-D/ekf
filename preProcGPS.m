%pre-process GPS
format long
%import tower A lat,long,height
%import tower B lat,long,height
%import full file (vectors for lat,long,height)
%transform using geo2NED with latA,longA,hA (Pref - origin)
%populate vectors N E D
%Legacy code from importing LLH coordinates
%{
geo = zeros(length(latitudedeg),3);
geo(1,:) = [latitudedeg(1),longitudedeg(1),heightm(1)];

for i=2:length(latitudedeg)
    geo(i,:)=[latitudedeg(i),longitudedeg(i),heightm(i)];
end
%}
%% 
%pre-import accurate positions of towerA (origin)
latA;  %latitude of towerA
longA; %longitude of towerA
hA;    %height of towerA

%pre-import accurate positions of towerB (final point)
latB;  %latitude of towerB
longB; %longitude of towerB
hB;    %height of towerB
%tower B can be used as a sanity check for the final position (NED)
[xBNED,yBNED,zBNED] = geodetic2ned(latB,longB,hB,latA,longA,hA,wgs84Ellipsoid);
%check [xBNED,yBNED,zBNED]=[xNED(final),yNED(final),zNED(final)];
%% 
Tgps=1/5; %5Hz GPS receiver rate
%% Transfrom geodetic to NED (first from geo -> ECEF then ECEF -> NED)
[xNED,yNED,zNED] = geodetic2ned(latitudedeg,longitudedeg,heightm,latA,longA,hA,wgs84Ellipsoid);
%[xNED,yNED,zNED] = geodetic2ned(latitudedeg,longitudedeg,heightm,latitudedeg(200),longitudedeg(200),heightm(200),wgs84Ellipsoid);
%[xNED,yNED,zNED] = geodetic2ned(latitudedeg,longitudedeg,heightm,-33.956519396,18.463038259,110.7630,wgs84Ellipsoid);
%%
%populate vector of HDOP and VDOP (from imported text file) use for R
sdnm; %north HDOP
sdem; %east HDOP
sdum; %up/down VDOP

%synthesis of vgps measurement from RTK post output file (pre-imported)
vE; %north velocity component
vN; %east velocity component

%prealocate for speed======================================================
v=zeros(length(xNED),1);    %gps derived line velocity
dxE=zeros(length(xNED),1);  %deltaEast (m)
dyN=zeros(length(xNED),1);  %deltaNorth (m)
r=zeros(length(xNED),1);    %gps derived line position (m)
%==========================================================================

%% populate vGPS and rGPS measurements for comparison with states v and r
for i=2:length(xNED)
%project velcotiy componenets onto line using line geometry 
v(i) = vN(i)*cos(psiL) + vE(i)*sin(psiL); 

%synthesis of rgps measurement (line position)
%origin = towerA: [0,0,0]
dxE(i) = xNED(i) - xNED(1); %current position - origin
dyN(i) = yNED(i) - yNED(1); %current position - origin
%project onto line using line geometry
r(i) = dyN(i)*cos(psiL) + dxE(i)*sin(psiL); 
end
%%     
scatter3(xNED,yNED,zNED,'.b');
