function p = sigmod(x)
    a = 20;%��������
    p = abs(1./(1+exp(-a*(x-0.4))));
end