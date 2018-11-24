% format compact; clear; clf; clc

load ('magnetometer_data.mat')

mag_x = [];
mag_y = [];
mag_z = [];

for i=1:10:length(mag_data)
    mag_x = [mag_x ; mag_data(i,1)];
    mag_y = [mag_y ; mag_data(i,2)];
    mag_z = [mag_z ; mag_data(i,3)];
end

% plotting data
figure(1)
plot3(mag_x,mag_y,mag_z,'y-')
title('Magnetometer Reading')
xlabel('mag_x')
ylabel('mag_y')
zlabel('mag_z')

hold on
grid on
grid minor
axis equal

x_min = min(x); x_max = max(x);
y_min = min(y); y_max = max(y);
z_min = min(z); z_max = max(z);

xyz_min = min(x_min,min(y_min,z_min));
xyz_max = max(x_max,max(y_max,z_max));

ellipse_center_x = (min(mag_x) + max(mag_x))/2
ellipse_center_y = (min(mag_y) + max(mag_y))/2
ellipse_center_z = (min(mag_z) + max(mag_z))/2

XXXX = [mag_x(1:4), mag_y(1:4), mag_z(1:4), ones(4,1)];
YYYY = (mag_x(1:4)).^2 + (mag_y(1:4)).^2 + (mag_z(1:4)).^2;
beta = inv(XXXX'*XXXX)*(XXXX'*YYYY)
bb = pinv(XXXX)*YYYY

% plotting axis through center of elliple
plot3([xyz_min,xyz_max],[0+ellipse_center_y,0+ellipse_center_y],[0,0],'r-','lineWidth',3)
plot3([0+ellipse_center_x,0+ellipse_center_x],[xyz_min,xyz_max],[0+ellipse_center_z,0+ellipse_center_z],'g-','lineWidth',3)
plot3([0+ellipse_center_x,0+ellipse_center_x],[0+ellipse_center_y,0+ellipse_center_y],[xyz_min,xyz_max],'b-','lineWidth',3)

% plotting box
plot3([xyz_min,xyz_max,xyz_max,xyz_min,xyz_min],[xyz_min,xyz_min,xyz_max,xyz_max,xyz_min],[xyz_min,xyz_min,xyz_min,xyz_min,xyz_min],'k-')
plot3([xyz_min,xyz_max,xyz_max,xyz_min,xyz_min],[xyz_min,xyz_min,xyz_max,xyz_max,xyz_min],[xyz_max,xyz_max,xyz_max,xyz_max,xyz_max],'k-')
plot3([xyz_min,xyz_min],[xyz_min,xyz_min],[xyz_min,xyz_max],'k-')
plot3([xyz_min,xyz_min],[xyz_max,xyz_max],[xyz_min,xyz_max],'k-')
plot3([xyz_max,xyz_max],[xyz_max,xyz_max],[xyz_min,xyz_max],'k-')
plot3([xyz_max,xyz_max],[xyz_min,xyz_min],[xyz_min,xyz_max],'k-')

% plotting absolute coordinate
plot3([xyz_min,xyz_max],[0,0],[0,0],'r-','lineWidth',1.25)
plot3([0,0],[xyz_min,xyz_max],[0,0],'g-','lineWidth',1.25)
plot3([0,0],[0,0],[xyz_min,xyz_max],'b-','lineWidth',1.25)

% plotting displacement vector
plot3([0,ellipse_center_x],[0,ellipse_center_y],[0,ellipse_center_z],'k-','lineWidth',5)
plot3(0,0,0,'ks')
plot3(ellipse_center_x,ellipse_center_y,ellipse_center_z,'ko')

% displacing origin
mag_x_n = mag_x - ellipse_center_x;
mag_y_n = mag_y - ellipse_center_y;
mag_z_n = mag_z - ellipse_center_z;

% plotting displaced ellipse
plot3(mag_x_n,mag_y_n,mag_z_n,'c-')

% calculating radius of obtained ellipse
r = sqrt(mag_x_n.^2 + mag_y_n.^2 + mag_z_n.^2);
r_mean = mean(r);
r_var = var(r);

figure(2)
plot(r)
hold on
plot(r_mean*ones(1,length(r)))
legend('r','r_{mean}')