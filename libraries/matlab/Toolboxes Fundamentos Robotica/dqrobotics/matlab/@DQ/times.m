%Decompositional multiplication 
function res = times(a,b)

    a = DQ(a);    
    b = DQ(b);
    
    res = T(a)*T(b)*a.P*b.P;
%      res = tplus(a)*tplus(b)*a.P*b.P;
end