syms x y phi
syms u1 u2

f1 = x + u1*cos(phi+0.5*u2);
f2 = y + u1*sin(phi+0.5*u2);
f3 = phi + u2;

jacobian([f1, f2, f3], [x, y, phi]);
