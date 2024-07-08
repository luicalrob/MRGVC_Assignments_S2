function [R_est, c_est]=seleccionaSolucionF(R1,R2,t,R_est,c_est)
%function [R_est, c_est]=seleccionaSolucionF(R1,R2,t,R_est,c_est)
%Gonzalo Lopez Nicolas Agosto 2022
%A partir de dos soluciones (t R1 y -t R2) de la descomposición de la 
%matriz fundamental, se elige la solución correcta de movimiento. Estimación 
%inicial es R_est y c_est.

if nargin<4
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
    mt1= sum(abs(t-c_est)) ;
    mt2= sum(abs(-t-c_est)) ;
    [mt, mti]=min([mt1, mt2]);
    switch mti
        case 1, ti= t;
        case 2, ti= -t;
    end
    ti=ti/norm(ti);
    R_est=Ri;
    c_est=ti;