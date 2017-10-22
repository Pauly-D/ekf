syms aX aY aZ psi the phi


%R = [ cos(the)*sin(phi) + cos(phi)*sin(psi)*sin(the),  cos(phi)*cos(psi), sin(phi)*sin(the) - cos(phi)*cos(the)*sin(psi);...
     %cos(phi)*cos(the) - sin(phi)*sin(psi)*sin(the), -cos(psi)*sin(phi), cos(phi)*sin(the) + cos(the)*sin(phi)*sin(psi);...
    %-cos(psi)*sin(the), sin(psi), cos(psi)*cos(the)];


R = [cos(psi)*cos(the), cos(psi)*sin(phi)*sin(the)-cos(phi)*sin(psi), sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(the);...
     cos(the)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(the), cos(phi)*sin(psi)*sin(the)-cos(psi)*sin(phi);...
     -sin(the), cos(the)*sin(phi), cos(phi)*cos(the)];
     

rotA = R*[aX;aY;aZ];

matlabFunction(rotA, 'File', 'rotAcc');