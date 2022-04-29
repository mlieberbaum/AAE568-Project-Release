function Pf = PropCovar(P0, A, Q, tf)

odefun = @(t,x) fdot(t,x,A,Q);

x0 = reshape(P0,numel(A),1);

S = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[t,y] = ode45(odefun, [0 tf], x0, S);

Pf = reshape(y(end,:)',size(A));

end

function dx = fdot(t,x,A,Q)
P = reshape(x,size(A));
Pdot = A*P + P*A' + Q;
dx = reshape(Pdot,numel(A),1);
end