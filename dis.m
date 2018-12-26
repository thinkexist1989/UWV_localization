function [d k]= dis(loc,s)
    l = 4 ; w = 2; % 长宽的一半
    n(1,:) = [1  0]; p(1,:) = [l  0];  % 面1 
    n(2,:) = [0  1]; p(2,:) = [0  w];  % 面2
    n(3,:) = [-1 0]; p(3,:) = [-l 0];  % 面3
    n(4,:) = [0 -1]; p(4,:) = [0 -w];  % 面4
    
    dd = [0 0 0 0];
    for i = 1:4
        dd(i) = dis_to_side(loc,s,n(i,:),p(i,:));
    end    
    [d,k]= min(dd);
end