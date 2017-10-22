%----------------------------------------------------------------------------
%Setup

T = t/1000;
dt=zeros(length(T),1);

for i = 1:length(T)-1
    dt(i) = t(i+1)-t(i);
end

%Initital Conditions
ORIGINtowerA = [lat0,long0,h0]; %from RTK accurate parameters Pref 
%(just use straight from RTK algorithm output)
NEDtowerA = geodetic2NED();
NEDtowerB = geodetic2NED();

%power line sag parmater
a = e-4;

%Parameterize power line
r0 = 0;
%horizontal span of power line use pythag on X and Y for tower B since
%tower A is the origin
r1 = sqrt((NEDtowerB(1)^2)+(NEDtowerB(2)^2));
%b and c constants
bC = inv([r0 1;r1 1])*[towerA(3)-a*r0^2;towerB(3)-a*r1^2];
b=bC(1);
c=bC(2);











