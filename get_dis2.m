function d = get_dis2(loc,pos) 
    i = 1;
    for e = 0:0.01:pi/60
        for k = 0:0.01:2*pi
        sf = front_vector2(pos,[k;e]);
        ss = side_vector2(pos,[k;e]);
        d_matrix(i,1) = dis(loc,sf); %ǰ�˸߶ȼ�����
        d_matrix(i,2) = dis(loc,ss); %����߶ȼ�����
        i = i+1;
        end
    end
    
    d = min(d_matrix);
end