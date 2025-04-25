function PendulumCart_Anim(x,theta,tout,l)
% Animation: Motion of Pendulum on a Cart
%
% PendulumCartAnim(x,theta,tout,l) creates an animation of the
% Inverted-Pendulum-on-a-Cart System. 
%
% Receives as arguments the position of the cart (x), the angular position of the
% pendulum (theta), the time vector for the simulation (tout), and the length
% of the pendulum (l). It computes the dimensions of the cart based on l. 

%%
figure(1); clf 
boxplot = [.1 .1 .8 .8*.75];
y = 0.6*l;
penthick = 0.03*l;
i = 1;
axes('Position', boxplot, 'Box', 'on');
title('Animation of Pendulum on a Cart')
if abs(x(i)) <= 1*l
    set(gca, 'XLim', [-2.0*l, 2.0*l], 'YLim', [-1.0*l, 2.0*l])
    Time = text(-1.9*l, 1.5*l, ['Time: ' num2str(tout(i)) ' s']);
else
    set(gca, 'XLim', [x(i)-2.0*l, x(i)+2.0*l], 'YLim', [-1.0*l, 2.0*l])
    Time = text(x(i)-1.9*l, 1.5*l, ['Time: ' num2str(tout(i)) ' s']);
end
hold on
Cart = rectangle('Position', [-0.3*l+x(i) 0.2*l 0.6*l 0.4*l], 'Curvature', [.3 .3], 'FaceColor', [.4 .4 .8]);
Ground = plot([-100 100], [0 0], 'k');
Wheel1 = rectangle('Position', [-0.31*l+x(i) 0.0*l 0.22*l 0.22*l], 'Curvature', [1 1], 'FaceColor', [.4 .4 .5]);
Wheel2 = rectangle('Position', [0.09*l+x(i) 0.0*l 0.22*l 0.22*l], 'Curvature', [1 1], 'FaceColor', [.4 .4 .5]);
xpend = [x(i)-penthick*cos(theta(i)), x(i)+l*sin(theta(i))-penthick*cos(theta(i)), x(i)+l*sin(theta(i))+penthick*cos(theta(i)), x(i)+penthick*cos(theta(i))];
ypend = [y+penthick*sin(theta(i)), y+l*cos(theta(i))+penthick*sin(theta(i)), y+l*cos(theta(i))-penthick*sin(theta(i)), y-penthick*sin(theta(i))];
Pendulum = patch([xpend, xpend(1)], [ypend, ypend(1)], [.5 .5 .5]);
Pivot = rectangle('Position', [-0.05*l+x(i) 0.55*l 0.1*l 0.1*l], 'Curvature', [1 1], 'FaceColor', [.4 .4 .5]);
xlabel('x (m)')
ylabel('y (m)')
for i = 2:length(tout)
    pause(tout(i)-tout(i-1))
    delete(Time);
    if abs(x(i)) <= 1*l
        set(gca, 'XLim', [-2.0*l, 2.0*l], 'YLim', [-1.0*l, 2.0*l])
        Time = text(-1.9*l, 1.5*l, ['Time: ' num2str(tout(i)) ' s']);
    else
        set(gca, 'XLim', [x(i)-2.0*l, x(i)+2.0*l], 'YLim', [-1.0*l, 2.0*l])
        Time = text(x(i)-1.9*l, 1.5*l, ['Time: ' num2str(tout(i)) ' s']);
    end
    % hold on
    set(Cart,'Position', [-0.3*l+x(i) 0.2*l 0.6*l 0.4*l]);
    set(Wheel1, 'Position', [-0.31*l+x(i) 0.0*l 0.22*l 0.22*l]);
    set(Wheel2, 'Position', [0.09*l+x(i) 0.0*l 0.22*l 0.22*l]);
    xpend = [x(i)-penthick*cos(theta(i)), x(i)+l*sin(theta(i))-penthick*cos(theta(i)), x(i)+l*sin(theta(i))+penthick*cos(theta(i)), x(i)+penthick*cos(theta(i))];
    ypend = [y+penthick*sin(theta(i)), y+l*cos(theta(i))+penthick*sin(theta(i)), y+l*cos(theta(i))-penthick*sin(theta(i)), y-penthick*sin(theta(i))];
    delete(Pendulum);
    Pendulum = patch([xpend, xpend(1)], [ypend, ypend(1)], [.5 .5 .5]);
    set(Pivot,'Position', [-0.05*l+x(i) 0.55*l 0.1*l 0.1*l]);
end
