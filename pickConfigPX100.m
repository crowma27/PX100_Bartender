function cfg = pickConfigPX100()  % Define as entradas e saídas desta função.
% Dimensoes e tolerancias da pega do copo vermelho. Distancias em metros.
    % Geometria da pega
    cfg.cupHeight           = 0.070;       % m
    cfg.wristToFingerCenter = 0.100;       % m, offset do modelo atual da garra
    cfg.toolPitch           = deg2rad(0);  % 0 = garra horizontal
    cfg.elbow               = 'up';  % Atualiza cfg.elbow.
    cfg.approachClearance   = 0.080;       % m acima do ponto de pega
    cfg.sideApproachDistance = 0.050;     % m de recuo para baixar ao lado do copo
    cfg.rimClearance        = 0.012;       % m acima do rebordo para alinhar a garra
    cfg.gripBelowRim        = 0.015;       % m abaixo do rebordo para fechar no corpo do copo
    cfg.releaseClearance    = 0.005;       % m entre o fundo do copo e o apoio ao largar
    cfg.transportClearance  = 0.015;       % m de margem acima dos outros copos antes de transportar
    cfg.minimumCupClearance = 0.002;      % m de folga minima do copo sobre o apoio
    cfg.cupShiftTolerance   = 0.006;       % m de deslocamento permitido antes da pega
    cfg.cupTrackingTolerance = 0.018;      % m de desvio do copo enquanto esta preso
    cfg.placementTolerance  = 0.015;       % m, tolerancia para verificar o copo pousado

    % Convergencia das juntas
    cfg.maxJointStep        = deg2rad(0.5); % avanco maximo por passo de simulacao
    cfg.motionPause         = 0.05;        % segundos por passo para observar o movimento
    cfg.graspJointTolerance = deg2rad(1);  % erro maximo de cada junta para aceitar um alvo
    cfg.graspStableSteps    = 5;           % passos consecutivos dentro da tolerancia
    cfg.maxWaitSteps        = 500;         % passos max. a esperar por cada alvo

    % Passos de simulacao
    cfg.stepsGripper          = 60;        % abrir a garra
    cfg.gripperCloseSpeed     = 0.015;     % m/s, reduzir o impacto inicial no copo
    cfg.stepsGripperClose     = 200;       % compensa a menor velocidade de fecho
    cfg.finalHoldSeconds      = 5;         % tempo real para observar o copo
end  % Fim do bloco.
