function Dx = model(t,x,u)

Dx(1) = x(2);
Dx(2) = -1 * x(1) - 2 * x(2)^2 + u;

end