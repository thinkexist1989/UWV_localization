% 圆形运动
clear;clc;close all;

alpha = 0:0.1:2*pi; %1*n
n = size(alpha,2);
r = 1;
locmatrix = [r*cos(alpha'),r*sin(alpha'),zeros(n,1)];  %真实运动轨迹

%plot(loc(:,1), loc(:,2));
phi = (10*rand(n,1)-5)/180*pi;
theta = (10*rand(n,1)-5)/180*pi;
psi = pi/2 + alpha';
posmatrix = [phi,theta,psi];
%posmatrix = [zeros(n,1),zeros(n,1),psi];

d = zeros(n,2);
posprev1 = posmatrix(1,:)';
locprev1 = locmatrix(1,:)';
loccalc1 = zeros(n,3);

posprev2 = posmatrix(1,:)';
locprev2 = locmatrix(1,:)';
loccalc2 = zeros(n,3);
for i = 1:n
    loc = locmatrix(i,:)';
     plot(loc(1),loc(2),'*');
    set(gca,'YDir','reverse')%对Y方向反转
    hold on;
    pos = posmatrix(i,:)';
    d(i,:) = get_dis2(loc,pos); 
    dis = d(i,:);
    
    noise = 0.01*randn(size(dis));
    dis = dis + noise;
    
    loca1 = calc_xy(pos,posprev1,locprev1,dis);
    loca2 = calc_xy2(pos,posprev2,locprev2,dis);
    plot(loca1(1),loca1(2),'o',loca2(1),loca2(2),'-+');
    hold on;
    locprev1 = loca1;
    posprev1 = pos;
    loccalc1(i,:) = loca1;
    
    locprev2 = loca2;
    posprev2 = pos;
    loccalc2(i,:) = loca2;
end

figure('name','定位曲线');
plot(locmatrix(:,1), locmatrix(:,2),'--');
set(gca,'YDir','reverse')%对Y方向反转
axis equal;
hold on;
plot(loccalc1(:,1), loccalc1(:,2),'r',loccalc2(:,1), loccalc2(:,2),'g');

figure('name','定位误差bar图');
error(:,1) = 1000*sqrt((loccalc1(:,1)-locmatrix(:,1)).^2 + (loccalc1(:,2)-locmatrix(:,2)).^2);
error(:,2) = 1000*sqrt((loccalc2(:,1)-locmatrix(:,1)).^2 + (loccalc2(:,2)-locmatrix(:,2)).^2);
bar(error); %转换成mm

disp('方差');
disp(var(error));
disp('平均误差');
disp(mean(error));

figure('name','x坐标定位误差');
plot(1000*(loccalc1(:,1)-locmatrix(:,1)),'r'); %转换成mm
hold on;
plot(1000*(loccalc2(:,1)-locmatrix(:,1)),'b'); %转换成mm
figure('name','y坐标定位误差');
plot(1000*(loccalc1(:,2)-locmatrix(:,2)),'r'); %转换成mm
hold on;
plot(1000*(loccalc2(:,2)-locmatrix(:,2)),'b'); %转换成mm