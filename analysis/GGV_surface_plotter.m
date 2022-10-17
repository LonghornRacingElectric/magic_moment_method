T = readtable('GGV.csv');

x = table2array(T(:,["vehicle_accelerations_NTB_0"]));
y = table2array(T(:,["vehicle_accelerations_NTB_1"]));
z = table2array(T(:,["s_dot"]));

xlin = linspace(min(x), max(x), 100);
ylin = linspace(min(y), max(y), 100);
[X,Y] = meshgrid(xlin, ylin);
Z = griddata(x,y,z,X,Y,'natural');
%Z = griddata(x,y,z,X,Y,'cubic');
%Z = griddata(x,y,z,X,Y,'v4');
%mesh(X,Y,Z)
%axis tight; hold on
plot3(x,y,z,'.','MarkerSize',15)
grid on
ylabel("Lateral Acceleration (m/s^2)")
xlabel("Long Acceleration (m/s^2)")
zlabel("Speed (m/s)")