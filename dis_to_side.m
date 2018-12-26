function d = dis_to_side(loc,s,n,p)
    x0 = loc(1); y0 = loc(2); z0 = loc(3); %机器人位置
    a = s(1); b = s(2); c = s(3); % 高度计方向向量
    A = n(1); B = n(2); %侧壁方向向量
    xs = p(1); ys = p(2);%侧壁与坐标轴交点
    if (A*a+B*b)>0
        d = abs(A*x0+B*y0-A*xs-B*ys)*sqrt(a^2+b^2+c^2)/(A*a+B*b);
    else
        d = 9999;
    end
end
