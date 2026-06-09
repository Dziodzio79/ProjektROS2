% =========================================================
% Model Dynamiki Manipulatora
% =========================================================
clear; clc;

% Deklaracja zmiennych
syms d1 theta2 d3 real
syms dq1 dq2 dq3 real
syms ddq1 ddq2 ddq3 real

% Deklaracja stałych parametrów
syms m1 m2 m3 g L h0 d0 d2 I2 real

% Wektory stanu
q = [d1; theta2; d3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];

% 1. Położenia środków mas 
p1 = [0; 0; h0 + d1];
p2 = [L*cos(theta2); -d2; L*sin(theta2) + h0 + d1];
p3 = [L*cos(theta2); -d2 - d0 - d3; L*sin(theta2) + h0 + d1];

% 2. Prędkości liniowe 
v1 = jacobian(p1, q) * dq;
v2 = jacobian(p2, q) * dq;
v3 = jacobian(p3, q) * dq;

% 3. Energia kinetyczna (Ek)
Ek1 = (1/2) * m1 * (v1.' * v1);
Ek2 = (1/2) * m2 * (v2.' * v2) + (1/2) * I2 * dq2^2; 
Ek3 = (1/2) * m3 * (v3.' * v3);
Ek = simplify(Ek1 + Ek2 + Ek3);

% 4. Energia potencjalna (Ep)
Ep1 = m1 * g * p1(3);
Ep2 = m2 * g * p2(3);
Ep3 = m3 * g * p3(3);
Ep = simplify(Ep1 + Ep2 + Ep3);

% 5. Funkcja Lagrange'a
Lag = Ek - Ep;

% 6. Obliczenie równań Eulera-Lagrange'a
dL_ddq = jacobian(Lag, dq).'; 

% Pochodna Lagrangianu po położeniach
dL_dq = jacobian(Lag, q).';   

% Pełna pochodna po czasie
% Wzór: d/dt(f) = (df/dq)*dq + (df/ddq)*ddq
d_dt_dL_ddq = jacobian(dL_ddq, q) * dq + jacobian(dL_ddq, dq) * ddq;

% Ostateczny wektor momentów/sił
tau = simplify(d_dt_dL_ddq - dL_dq);

% 7. Ekstrakcja postaci macierzowej: M(q)*ddq + C(q,dq)*dq + G(q) = tau
fprintf('--- ROZWIĄZANIE DYNAMIKI ---\n\n');

% Macierz bezwładności M(q)
M = simplify(jacobian(tau, ddq));
disp('1. Macierz bezwładności M(q):');
disp(M);

% Wektor sił ciężkości G(q)
G = simplify(jacobian(Ep, q).');
disp('2. Wektor sił ciężkości G(q):');
disp(G);

% Wektor sił odśrodkowych i Coriolisa C(q,dq)*dq
C_dq = simplify(tau - M*ddq - G);
disp('3. Wektor sił Coriolisa i odśrodkowych C(q,dq)*dq:');
disp(C_dq);