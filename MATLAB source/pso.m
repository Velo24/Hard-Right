function gp = pso(func, lb, ub, n, round)
%H_OPT Summary of this function goes here
%   Detailed explanation goes here

pp      = linspace(lb,ub,n)';
pv      = zeros(n,1);
dp      = zeros(n,1);
% bv      = [];
% bp      = [];

gv_old  = [];
tol     = 10^-8;
counter = 0;

while counter < 3
    
    
    parfor i = 1:n
        pv(i) = func(pp(i)); %#ok<PFBNS>
    end
    
%     if isempty(bv)
%         bv = pv;
%         bp = pp;
%     else
%         parfor i = 1:n
%             [bv(i,1), I] = min([bv(i,1) pv(i,1)]);
%             
%             if I == 2
%                 bp(i,1) = pp(i,1);
%             end
%         end
%     end
    
    [gv,gi] = min(pv);
    gp      = pp(gi);
    
    parfor i = 1:n
        dp(i) = (2.5/pi)*((gp-pp(i))); %#ok<PFOUS>
        pp(i) = pp(i)+dp(i);
    end
    
    if isempty(gv_old)
        gv_old = gv;
    elseif abs((gv-gv_old)/gv) <= tol
        gv_old = gv;
        counter = counter+1;
    else
        gv_old = gv;
        counter = 0;
    end  
    
    disp(['round: ' round ', counter: ' num2str(counter) ', gv: ' num2str(gv)])
end

