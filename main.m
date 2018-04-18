% Quadrotor Hovering Simulation With AR Drone Parameters
% created on 2018/04/17
% created by Kandai Watanabe
% Lisense belongs to
% Takahashi Lab @ Keio University
disp('What Change have you made?');
message = input('');

dimX = 12;
dimU = 4;
% Genetic Algorithm Parameters
numvar = dimX + dimU;         % No. of Parameters for Controller Gain K
pops=30;                      % 個体数
maxgen=150;                   % 世代数
crossp=0.8;                   % 交叉確率
mutatep=0.35;                 % 突然変異確率
absolute_max = 2;             % パラメータの最大値（今回は10^(var)の変数varの最大値)
bound=absolute_max*ones(numvar, 2); bound(:,1)=-absolute_max;
rng=(bound(:,2)-bound(:,1))'; % 変数の範囲
pop=zeros(pops,numvar);       % 個体の初期化
% randomly create betw. 0 ~ bound;     + % center to 0 (-bound ~ bound)
pop(:,1:numvar)=(ones(pops,1)*rng).*(rand(pops,numvar))+(ones(pops,1)*bound(:,1)'); % 個体の生成

%% Test Each Generation
tic;
for it=1:maxgen
    fpop=sim_drone(pop);    % 適応度の計算
    [cs,inds]=max(fpop);    % エリート　cs:最大値  inds:順番
    bchrom=pop(inds,:);     % エリートの値の格納
    % 選択
    toursize=5;
    players=ceil(pops*rand(pops,toursize)); % 適応度の組み合わせ
    scores=fpop(players);
    [a,m]=max(scores');
    pind=zeros(1,pops);
    for i=1:pops
        pind(i)=players(i,m(i));
        parent(i,:)=pop(pind(i),:);
    end
    % 交叉
    child=cross(parent,crossp);
    % 突然変異
    pop=mutate(child,mutatep,bound,rng);
    pop(1,:)=bchrom;
    elapsed_time=toc;
    left_time = elapsed_time/it*maxgen - elapsed_time; 
    waitbar(it/maxgen,h,sprintf('Time Left %03i [min] 2i [sec]',left_time/60, rem(left_time,60)));
    save pop.mat pop
end

%%
content = strcat(message, sprintf('   Time taken %0.0f [min] ', elapsed_time/60));
sendmail('kandai@keio.jp', 'Simulation Ended', content );


