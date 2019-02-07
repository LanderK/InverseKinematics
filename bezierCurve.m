function Q = bezierCurve(P0,P1,P2,P3,intervals)

    t = linspace(0,1,intervals);
    c3 = -P0 + 3*(P1-P2) + P3;
    c2 = 3*(P0 - (2*P1)+P2); 
    c1 = 3*(P1 - P0);
    c0 = P0;
    Q = [];
    for k=1:size(t,2)
    a = ((c3*t(k)+c2)*t(k)+c1)*t(k) + c0; 
    Q = cat(1,Q,a);
    end
end