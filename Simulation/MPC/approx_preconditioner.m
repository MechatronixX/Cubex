function [P,L]  = approx_preconditioner( R, m, iH, D)
% Approximate the preconditioner P (Lemma 4 from papper) using equation 26
% and 27 from the report 
    tol = 1e-10;
    P = inv(R);
    
    % Here we set elements of P(row,col) = 0 if P(row,col) < tol
    for r = 1 : size(P,1)
        for c = 1 : size(P,2)
            if abs(P(r,c)) < tol
               P(r,c) = 0;          
            end
            % Eq 28 from the report
            if abs(r-c) > m
                P(r,c) = 0;
            end
        end
    end    
    LL = P * D * iH * D' * P';
    L = norm(LL,2);
    P = (1/sqrt(L)).*P;
    LL = P * D * iH * D' * P';
    L = norm(LL,2);
    disp(['Condtion number of LL = ' num2str(cond(LL))])
end

