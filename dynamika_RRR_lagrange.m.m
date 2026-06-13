% =========================================================
% Model Dynamiki Manipulatora RRR
% Lagrange II rodzaju + postać macierzowa
% Model zgodny z obliczeniami dla a0 = 0
% =========================================================
clear; clc;

% =========================================================
% 1. Deklaracja zmiennych
% =========================================================

syms theta1 theta2 theta3 real
syms dq1 dq2 dq3 real
syms ddq1 ddq2 ddq3 real

syms l real
syms m1 m2 m3 g real
syms I1 I2 I3 real

% Wektory stanu
q = [theta1; theta2; theta3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];

% Skróty trygonometryczne
c12 = cos(theta1 + theta2);
s12 = sin(theta1 + theta2);

c3 = cos(theta3);
s3 = sin(theta3);

% Skrót prędkości
dq12 = dq1 + dq2;

% =========================================================
% 2. Położenia środków mas
% =========================================================
% Przyjęto:
% - człony są jednorodne,
% - środki mas leżą w połowie rozpatrywanych odcinków,
% - masa efektora jest uwzględniona w masie trzeciego członu m3,
% - wysokość podstawy pominięto jako stałe przesunięcie.

pC1 = [
    0;
    0;
    0
];

pC2 = [
    l*c12;
    l*s12;
    l
];

pC3 = [
    l*(2 + c3)*c12;
    l*(2 + c3)*s12;
    l*(1 + s3)
];

% =========================================================
% 3. Prędkości liniowe środków mas
% =========================================================

vC1 = simplify(jacobian(pC1, q) * dq);
vC2 = simplify(jacobian(pC2, q) * dq);
vC3 = simplify(jacobian(pC3, q) * dq);

vC1_sq = simplify(vC1.' * vC1);
vC2_sq = simplify(vC2.' * vC2);
vC3_sq = simplify(vC3.' * vC3);

% =========================================================
% 4. Osie obrotu i prędkości kątowe
% =========================================================

z0 = [0; 0; 1];
z1 = [0; 0; 1];
z2 = [s12; -c12; 0];

omega1 = simplify(z0*dq1);
omega2 = simplify(z0*dq1 + z1*dq2);
omega3 = simplify(z0*dq1 + z1*dq2 + z2*dq3);

omega1_sq = simplify(omega1.' * omega1);
omega2_sq = simplify(omega2.' * omega2);
omega3_sq = simplify(omega3.' * omega3);

% =========================================================
% 5. Energia kinetyczna
% =========================================================
% Przyjęto uproszczony model bezwładności:
% I1, I2, I3 są skalarnymi momentami bezwładności członów.

Ek1 = simplify((1/2)*m1*vC1_sq + (1/2)*I1*omega1_sq);
Ek2 = simplify((1/2)*m2*vC2_sq + (1/2)*I2*omega2_sq);
Ek3 = simplify((1/2)*m3*vC3_sq + (1/2)*I3*omega3_sq);

Ek = simplify(Ek1 + Ek2 + Ek3);

% =========================================================
% 6. Energia potencjalna
% =========================================================
% z_Ci oznacza współrzędną z środka masy i-tego członu.

Ep1 = m1*g*pC1(3);
Ep2 = m2*g*pC2(3);
Ep3 = m3*g*pC3(3);

Ep = simplify(Ep1 + Ep2 + Ep3);

% =========================================================
% 7. Funkcja Lagrange'a
% =========================================================

Lag = simplify(Ek - Ep);

% =========================================================
% 8. Równania Lagrange'a II rodzaju
% tau_i = d/dt(dL/ddq_i) - dL/dq_i
% =========================================================

dL_ddq = jacobian(Lag, dq).';
dL_dq = jacobian(Lag, q).';

% Pełna pochodna po czasie:
% d/dt(f) = (df/dq)*dq + (df/ddq)*ddq

d_dt_dL_ddq = jacobian(dL_ddq, q)*dq + jacobian(dL_ddq, dq)*ddq;

tau = simplify(d_dt_dL_ddq - dL_dq);

% =========================================================
% 9. Postać macierzowa
% tau = M(q)*ddq + C(q,dq)*dq + G(q)
% =========================================================

M = simplify(jacobian(tau, ddq));

G = simplify(jacobian(Ep, q).');

C_dq = simplify(tau - M*ddq - G);

% Sprawdzenie symetrii macierzy M
M_symmetry_check = simplify(M - M.');

fprintf('WYNIKI: \n');

disp('Energia kinetyczna Ek:');
disp(Ek);


disp('Energia potencjalna Ep:');
disp(Ep);


disp('Macierz bezwladnosci M(q):');
disp(M);

disp('Wektor grawitacji G(q):');
disp(G);


disp('Wektor C (siły odśrodkowe i Coriolisa)(q,dq)*dq:');
disp(C_dq);
