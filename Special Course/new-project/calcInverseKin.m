function qc = calcInverseKin(u)

tx = u(1);
ty = u(2);
tz = u(3);



q1 = atan2(- tx^2 - ty^2, -(- tx^4 - 2*tx^2*ty^2 + (49*tx^2)/25 - ty^4 + (49*ty^2)/25)^(1/2)) - atan2(-tx, -ty);
q2 = - atan2(-tx, -ty) + atan2(- tx^2 - ty^2, (- tx^4 - 2*tx^2*ty^2 + (49*tx^2)/25 - ty^4 + (49*ty^2)/25)^(1/2));

qc = [q1;q2]

end