function wristBase = directKinematicsPX100(joints,links,shoulderHeight)  % Define as entradas e saídas desta função.
% FK do PX100: devolve apenas a posicao do wrist na base (metros).
% O elo LC e a garra ficam fora deste ponto final.

    if nargin < 3 || isempty(shoulderHeight), shoulderHeight = 0.0931; end  % Verifica se o argumento opcional foi fornecido.
    if numel(joints) ~= 4 || any(~isfinite(joints))  % Testa a condição sobre numel, joints, any, isfinite.
        error('joints tem de conter quatro angulos finitos em radianos.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    if numel(links) < 4 || any(~isfinite(links(2:4)))  % Testa a condição sobre numel, links, any, isfinite.
        error('links tem de conter L2, Lm e LB nas posicoes 2 a 4.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.

    theta0 = double(joints(1));  % Orienta a base no plano horizontal.
    theta1 = double(joints(2));  % Atualiza theta1.
    theta2 = double(joints(3));  % Atualiza theta2.
    L2 = double(links(2));  % Atualiza L2.
    Lm = double(links(3));  % Atualiza Lm.
    LB = double(links(4));  % Atualiza LB.
    LA = hypot(L2,Lm);  % Calcula a distância no plano.
    beta = atan2(Lm,L2);  % Calcula o ângulo tendo em conta o quadrante.

    % Posicao do wrist no plano radial-vertical.
    radial = LA*sin(theta1+beta)+LB*cos(theta1+theta2);  % Obtém a direção horizontal da base para o copo.
    height = LA*cos(theta1+beta)-LB*sin(theta1+theta2)+shoulderHeight;  % Atualiza height.

    % Roda o plano radial para as coordenadas XYZ da base.
    wristBase = [radial*cos(theta0),radial*sin(theta0),height];  % Atualiza wristBase.
end  % Fim do bloco.
