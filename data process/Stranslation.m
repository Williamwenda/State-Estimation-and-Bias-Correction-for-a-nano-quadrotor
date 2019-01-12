function [Z,arf0,arf]=Stranslation(Mu,Mzz)
%% Sigma point transltaion function    
    len=length(Mu);
    Z = zeros(12,2*len+1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    EPS = 10^-5;
    ZERO = 10^-10;
    sigma = Mzz;
    [r,err] = cholcov(sigma, 0);

    if (err ~= 0)
    % the covariance matrix is not positive definite!
         [v,d] = eig(sigma);
    % set any of the eigenvalues that are <= 0 to some small positive value
    for n = 1:size(d,1)
        if (d(n, n) <= ZERO)
            d(n, n) = EPS;
        end
    end
    % recompose the covariance matrix, now it should be positive definite.
    sigma = v*d*v';
    [r,err] = cholcov(sigma, 0);
    if (err ~= 0)
        disp('ERROR!');
    end
    Mzz=sigma;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
    L = chol(Mzz,'lower');
    Z(:,1)=Mu;
    K=2;      %% K need to be selected
    arf0=K/(len+K);
    arf=0.5*K/(len+K);
    for i=1:len
        Z(:,i+1)=Mu+sqrt(len+K)*L(:,i);
        Z(:,i+1+len)=Mu-sqrt(len+K)*L(:,i);
    end
end