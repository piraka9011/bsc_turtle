dist = 15;
deltaOp = 3;
f1 = @(x,y) max(0, 1-x/dist) * max(0, 1-(y/deltaOp)^2);
fsurf(f1, [0 20 -2*pi() 2*pi()]);
title('Parabolic Plot of BSC function');
xlabel('Distance from goal (m)');
ylabel({'Difference in operator and'; 'controller inputs (rad \cdot s^{-1})'});
zlabel('BSC parameter \alpha');
str = '$\alpha$ = max(0, 1 - $\frac{d}{d_0}$) $\cdot$ max(0, 1 - $\frac{\Delta}{\Delta_0}$)';
text(25, 17, str, 'Interpreter', 'latex', 'FontSize', 12);