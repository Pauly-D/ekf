function rotG = rotGyro(p,phi,q,r,the)
%ROTGYRO
%    ROTG = ROTGYRO(P,PHI,Q,R,THE)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    14-Sep-2017 21:31:30

t2 = tan(the);
t3 = cos(phi);
t4 = sin(phi);
t5 = cos(the);
t6 = 1.0./t5;
rotG = [p+q.*t2.*t4+r.*t2.*t3;q.*t3+r.*t4;q.*t4.*t6+r.*t3.*t6];
