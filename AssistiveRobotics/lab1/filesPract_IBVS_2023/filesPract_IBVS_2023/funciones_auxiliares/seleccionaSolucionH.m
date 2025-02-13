function [R_est, c_est]=seleccionaSolucionH(R1,t1,R2,t2,R_est,c_est)
%function [R_est, c_est]=seleccionaSolucionH(R1,t1,R2,t2,R_est,c_est)
%Gonzalo Lopez Nicolas Agosto 2022
%A partir de dos soluciones (t1 R1 y t2 R2) de la descomposición de la 
%homografía, se elige la solución correcta de movimiento. Estimación 
%inicial es R_est y c_est.

if nargin<5
    R_est=eye(3);
    c_est=[0 0 0]';
end

    mR1= sum(sum(abs(R1-R_est))) ;
    mR2= sum(sum(abs(R2-R_est))) ;
    if mR1<mR2,
        Ri=R1;
    else
        Ri=R2;
    end
    mt1= sum(abs(t1-c_est)) ;
    mt2= sum(abs(-t1-c_est)) ;
    mt3= sum(abs(t2-c_est)) ;
    mt4= sum(abs(-t2-c_est)) ;
    [mt, mti]=min([mt1, mt2, mt3, mt4]);
    switch mti
        case 1, ti= t1;
        case 2, ti= -t1;
        case 3, ti= t2;
        case 4, ti= -t2;
    end
    R_est=Ri;
    c_est=ti;