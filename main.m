% Quadrotor Hovering Simulation With AR Drone Parameters
% created on 2018/04/17
% created by Kandai Watanabe
% Lisense belongs to
% Takahashi Lab @ Keio University
disp('What Change have you made?');
message = input('');
disp('Simulation Start')

dimX = 12;
dimU = 4;
% Genetic Algorithm Parameters
numvar = 11;                  % No. of Parameters for Controller Gain K
pops=30;                      % 個体数
maxgen=300;                   % 世代数
crossp=0.8;                   % 交叉確�?
mutatep=0.35;                 % 突然変異確�?
absolute_max = 2;             % パラメータの�?��値?�今回は10^(var)の変数varの�?��値)
bound=absolute_max*ones(numvar, 2); bound(:,1)=-absolute_max;
rng=(bound(:,2)-bound(:,1))'; % 変数の�?��
pop=zeros(pops,numvar);       % 個体�?初期�?
% randomly create betw. 0 ~ bound;     + % center to 0 (-bound ~ bound)
pop(:,1:numvar)=(ones(pops,1)*rng).*(rand(pops,numvar))+(ones(pops,1)*bound(:,1)'); % 個体�?生�?

%% Test Each Generation
tic;
for it=1:maxgen
    fpop=sim_drone(pop);    % 適応度の計�?
    [cs,inds]=max(fpop);    % エリート�?cs:�?��値  inds:�?��
    bchrom=pop(inds,:);     % エリート�?値の格�?
    % 選�?
    toursize=5;
    players=ceil(pops*rand(pops,toursize)); % 適応度の�?��合わ�?
    scores=fpop(players);
    [a,m]=max(scores');
    pind=zeros(1,pops);
    for i=1:pops
        pind(i)=players(i,m(i));
        parent(i,:)=pop(pind(i),:);
    end
    % 交�?
    child=mycross(parent,crossp);
    % 突然変異
    pop=mutate(child,mutatep,bound,rng);
    pop(1,:)=bchrom;
    elapsed_time=toc;
    left_time = elapsed_time/it*maxgen - elapsed_time; 
    disp(fpop');
    disp(sprintf('Time Left %0.0f [min]',left_time/60));
    disp(sprintf('Currently %0.1f Percent', it/maxgen*100));
    disp(sprintf('Max Value: %0.5f', cs));
    save pop.mat pop fpop
end

%%
content = strcat(message, sprintf('   Time taken %0.0f [min] ', elapsed_time/60));
sendmail('kandai@keio.jp', 'Simulation Ended', content );
%     Q = diag(10.^pop(1,1:length(X)));
%     R = diag(10.^pop(1,length(X)+1:length(X)+length(U)));
Q = diag(10.^[pop(i,1) pop(i,1) pop(i,2) pop(i,3) pop(i,3) pop(i,4) ...
    pop(i,5) pop(i,5) pop(i,6) pop(i,7) pop(i,7) pop(i,8)]);
R = diag(10.^[pop(i,9) pop(i,10) pop(i,10) pop(i,11)]);
disp(Q);
disp(R);
disp(K);
