function d = get_dis(loc,pos)
    sf = front_vector(pos(1),pos(2),pos(3)); %前端发射向量
    ss = side_vector(pos(1),pos(2),pos(3)); %侧面发射向量
    d(1) = dis(loc,sf); %前端高度计数据
    d(2) = dis(loc,ss); %侧面高度计数据   
end