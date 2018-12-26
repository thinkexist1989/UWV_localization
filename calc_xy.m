function loc = calc_xy(pos,posprev,locprev,dis)
    beta = 0; %矩阵旋转
    l = 4; w = 2;
    phi = pos(1); theta = pos(2); psi = pos(3);  %当前姿态
    if psi > pi
        psi = psi - 2*pi;
    elseif psi < -pi
        psi = psi + 2*pi;
    end
        
    phiprev = posprev(1); thetaprev = posprev(2); psiprev = posprev(3); %上一时刻姿态
    if psiprev > pi
        psiprev = psiprev - 2*pi;
    elseif psiprev < -pi
        psiprev = psiprev + 2*pi;
    end
    
    xprev = locprev(1); yprev = locprev(2); %上一时刻位置
    df = dis(1); ds = dis(2); %当前高度计数据
    sf = front_vector(phi,theta,psi); %前端发射向量
    ss = side_vector(phi,theta,psi); %侧面发射向量
    
   % n(1,:) = [1  0]; p(1,:) = [l  0];  c(1,:) = [l   w];% 面1 
   % n(2,:) = [0  1]; p(2,:) = [0  w];  c(2,:) = [-l  w];% 面2
   % n(3,:) = [-1 0]; p(3,:) = [-l 0];  c(3,:) = [-l -w];% 面3
   % n(4,:) = [0 -1]; p(4,:) = [0 -w];  c(4,:) = [l  -w];% 面4
    
    n = [1   0   -1    0;
         0   1    0   -1;
         0   0    0    0];  %面法向量  2*4
    p = [l   0   -l    0;
         0   w    0    w;
         0   0    0    0];  %面与坐标轴交点
    c = [l  -l   -l    l;
         w   w   -w   -w;
         0   0    0    0];  %角
     
    ns = [0 0 1]'; %平面法向量
    R = 1; B = 2; L = 3; U = 4;    
    if((psi >= 0) && (psi < pi/2))
        beta = 0; R = 1; B = 2; L = 3; U = 4;
    elseif ((psi >= pi/2) && (psi < pi))
        beta = pi/2; R = 2; B = 3; L = 4; U = 1;
    elseif ((psi >= -pi/2) && (psi < 0))
        beta = -pi/2; R = 4; B = 1; L = 2; U = 3;
    elseif ((psi >= -pi) && (psi < -pi/2))
        beta = -pi; R = 3; B = 4; L = 1; U = 2;
    end
    
    C = rotation_matrix(beta);  %旋转变换矩阵 3*3
    alpha = psi - beta;  %夹角
    alphaprev = psiprev - beta;
    xlocprev = C*locprev; nn = C*n; pp = C*p; cc = C*c;
    xxprev = xlocprev(1); yyprev = xlocprev(2);
    ddff = df * (1-(abs(dot(sf,ns))/(norm(sf)*norm(ns)))^2);
    ddss = ds * (1-(abs(dot(ss,ns))/(norm(ss)*norm(ns)))^2);
    
    
    
    k1 = (yyprev - cc(2,R))/(xxprev - cc(1,R));
    k2 = (yyprev - cc(2,B))/(xxprev - cc(1,B));
    
    region = 0;  %区域
    if     (k1 >= tan(alpha))&&(k2 >= -1/tan(alpha))
        region = 1;
    elseif (k1 >= tan(alpha))&&(k2 < -1/tan(alpha))
        region = 2;
    elseif (k1 < tan(alpha))&&(k2 < -1/tan(alpha))
        region = 3;
    elseif (k1 < tan(alpha))&&(k2 >= -1/tan(alpha))
        region = 4;
    end
    
    dr = abs(cos(alpha)*(yyprev-cc(2,R))-sin(alpha)*(xxprev-cc(1,R)));
    db = abs(sin(alpha)*(yyprev-cc(2,B))+cos(alpha)*(xxprev-cc(1,B)));
    tr = 0; tb = 0;
    if(dr< 0.3)
        tr = 0.8;
    end
    
    if(db< 0.3)
        tb = 0.8;
    end
    
    
    switch(region)
        case 1
            xx = norm(pp(:,R))-ddff*cos(alpha);
            yy = norm(pp(:,B))-ddss*cos(alpha);          
        case 2
            xx = norm(pp(:,R))-ddff*cos(alpha);
            yy = yyprev + (xx - xxprev)*tan((alphaprev + alpha)/2);
        case 3
            xx = -norm(pp(:,L)) + ddss*sin(alpha);
            yy = norm(pp(:,B)) - ddff*sin(alpha);
        case 4
            yy = norm(pp(:,B))-ddff*sin(alpha);
            xx = xxprev + (yy - yyprev)/tan((alphaprev+alpha)/2);
    end
    
    loc = pinv(C)*[xx yy 0]';
end