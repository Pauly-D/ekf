function f = f(a_r,v_r,r_r,sag,phi,the,psi,dphi,dthe,dpsi,dt)
%F
%    F = F(A_R,V_R,R_R,SAG,PHI,THE,PSI,DPHI,DTHE,DPSI,DT)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    16-Oct-2017 09:01:00

t2 = conj(dt);
t3 = conj(v_r);
f = [t3+t2.*conj(a_r),conj(r_r)+t2.*t3,conj(phi)+t2.*conj(dphi),conj(the)+t2.*conj(dthe),conj(psi)+t2.*conj(dpsi),conj(sag)];
