function R = nasnas_rot(phi, theta, psi)
%NASNAS_ROT  Body-to-Earth (NED) rotation matrix R_B^E = Rz(psi)Ry(theta)Rx(phi), Eq.(9)-(10).
cph=cos(phi); sph=sin(phi); cth=cos(theta); sth=sin(theta); cps=cos(psi); sps=sin(psi);
R = [cth*cps, sph*sth*cps-cph*sps, cph*sth*cps+sph*sps;
     cth*sps, sph*sth*sps+cph*cps, cph*sth*sps-sph*cps;
     -sth,    sph*cth,             cph*cth];
end
