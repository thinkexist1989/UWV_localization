% 矩形运动
clear;clc;close all;

x1 = -3.5:0.1:3.5; %1*n
n1 = size(x1,2);
locmatrix1 = [x1',-1*ones(n1,1),zeros(n1,1)];  %真实运动轨迹
phi1 = (10*rand(n1,1)-5)/180*pi;
theta1 = (10*rand(n1,1)-5)/180*pi;
psi1 = zeros(n1,1);
posmatrix1 = [phi1,theta1,psi1];


y2 = -1:0.1:1; %1*n
n2 = size(y2,2);
locmatrix2 = [3.5*ones(n2,1),y2',zeros(n2,1)];  %真实运动轨迹
phi2 = (10*rand(n2,1)-5)/180*pi;
theta2 = (10*rand(n2,1)-5)/180*pi;
psi2 = pi/2*ones(n2,1);
posmatrix2 = [phi2,theta2,psi2];

x3 = 3.5:-0.1:-3.5; %1*n
n3 = size(x3,2);
locmatrix3 = [x3',1*ones(n3,1),zeros(n3,1)];  %真实运动轨迹
phi3 = (10*rand(n3,1)-5)/180*pi;
theta3 = (10*rand(n3,1)-5)/180*pi;
psi3 = -pi*ones(n3,1);
posmatrix3 = [phi3,theta3,psi3];

y4 = 1:-0.1:-1; %1*n
n4 = size(y4,2);
locmatrix4 = [-3.5*ones(n4,1),y4',zeros(n4,1)];  %真实运动轨迹
phi4 = (10*rand(n4,1)-5)/180*pi;
theta4 = (10*rand(n4,1)-5)/180*pi;
psi4 = -pi/2*ones(n4,1);
posmatrix4 = [phi4,theta4,psi4];

locmatrix = [locmatrix1;locmatrix2;locmatrix3;locmatrix4];
posmatrix = [posmatrix1;posmatrix2;posmatrix3;posmatrix4];
n = n1 + n2 + n3 + n4;


d = zeros(n,2);
posprev1 = posmatrix(1,:)';
locprev1 = locmatrix(1,:)';
loccalc1 = zeros(n,3);

posprev2 = posmatrix(1,:)';
locprev2 = locmatrix(1,:)';
loccalc2 = zeros(n,3);

plot(locmatrix(:,1), locmatrix(:,2));
set(gca,'YDir','reverse')%对Y方向反转

figure(2);

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
    plot(loca1(1),loca1(2),'o',loca2(1),loca2(2),'+');
    hold on;
    
    locprev1 = loca1;
    posprev1 = pos;
    loccalc1(i,:) = loca1;
    
    locprev2 = loca2;
    posprev2 = pos;
    loccalc2(i,:) = loca2;
end

figure(3);
plot(locmatrix(:,1), locmatrix(:,2));
set(gca,'YDir','reverse')%对Y方向反转
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