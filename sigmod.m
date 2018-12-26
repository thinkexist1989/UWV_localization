function p = sigmod(x)
    a = 20;%ÊÕÁ²ËÙÂÊ
    p = abs(1./(1+exp(-a*(x-0.4))));
end