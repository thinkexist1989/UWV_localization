function d = get_dis(loc,pos)
    sf = front_vector(pos(1),pos(2),pos(3)); %ǰ�˷�������
    ss = side_vector(pos(1),pos(2),pos(3)); %���淢������
    d(1) = dis(loc,sf); %ǰ�˸߶ȼ�����
    d(2) = dis(loc,ss); %����߶ȼ�����   
end