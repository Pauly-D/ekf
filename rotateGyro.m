syms p q r phi the  %note no psi (yaw unobservable)

R=[1, sin(phi)*tan(the), cos(phi)*tan(the);...
    0, cos(phi), sin(phi);...
    0, sin(phi)/cos(the), cos(phi)/cos(the)];

rotG = R*[p;q;r]; 

matlabFunction(rotG, 'File', 'rotGyro');


