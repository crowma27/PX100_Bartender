function [joints,info] = inverseKinematicsPX100(targetWristBase,links, ...  % Define as entradas e saídas desta função.
    minJointPos,maxJointPos,alpha3,shoulderHeight,elbowConfiguration)  % Completa os dados da instrução anterior.
% IK do PX100 para a posicao do wrist no referencial da base (metros).
% Orientacao da ferramenta: theta1 + theta2 + theta3 = alpha3.

    if nargin < 5 || isempty(alpha3), alpha3 = 0; end  % Verifica se o argumento opcional foi fornecido.
    if nargin < 6 || isempty(shoulderHeight), shoulderHeight = 0.0931; end  % Verifica se o argumento opcional foi fornecido.
    if nargin < 7 || isempty(elbowConfiguration), elbowConfiguration = 'up'; end  % Verifica se o argumento opcional foi fornecido.

    if numel(targetWristBase) ~= 3 || any(~isfinite(targetWristBase))  % Testa a condição sobre numel, targetWristBase, any, isfinite.
        error('targetWristBase tem de ser um vetor XYZ finito.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    if numel(links) < 4 || any(~isfinite(links(2:4)))  % Testa a condição sobre numel, links, any, isfinite.
        error('links tem de conter L2, Lm e LB nas posicoes 2 a 4.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    if numel(minJointPos) < 4 || numel(maxJointPos) < 4  % Testa a condição sobre numel, minJointPos, maxJointPos.
        error('Faltam os limites das quatro juntas.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.

    targetWristBase = double(targetWristBase(:)');  % Atualiza targetWristBase.
    LA = hypot(double(links(2)),double(links(3)));  % Calcula a distância no plano.
    LB = double(links(4));  % Atualiza LB.
    beta = atan2(double(links(3)),double(links(2)));  % Calcula o ângulo tendo em conta o quadrante.
    psi = pi/2-beta;  % Atualiza psi.

    % 1) Waist: orienta o plano do braco para o wrist.
    theta0 = atan2(targetWristBase(2),targetWristBase(1));  % Orienta a base no plano horizontal.

    % 2) Posicao do wrist relativa ao ombro, no plano radial-vertical.
    Xed = hypot(targetWristBase(1),targetWristBase(2));  % Obtém a distância horizontal do pulso ao eixo da base.
    Yed = targetWristBase(3)-shoulderHeight;  % Expressa a altura do pulso relativamente ao ombro.
    R = hypot(Xed,Yed);  % Mede a distância do ombro ao pulso.
    minReach = abs(LA-LB);  % Distância mínima com o braço dobrado.
    maxReach = LA+LB;  % Distância máxima com o braço estendido.
    if R < 1e-9 || R < minReach-1e-9 || R > maxReach+1e-9  % Testa a condição sobre R, minReach, maxReach.
        error('Wrist fora do alcance: R=%.4f m; intervalo [%.4f, %.4f] m.', ...  % Atualiza error('Wrist fora do alcance: R.
            R,minReach,maxReach);  % Completa os dados da instrução anterior.
    end  % Fim do bloco.

    % 3) Triangulo LA-LB-R: calcula os angulos beta1 e beta2.
    cosBeta1 = (LA^2+R^2-LB^2)/(2*LA*R);  % Aplica a lei dos cossenos ao triângulo do braço.
    cosBeta2 = (LA^2+LB^2-R^2)/(2*LA*LB);  % Aplica a lei dos cossenos ao triângulo do braço.
    % Limita o cosseno para evitar erros de arredondamento no acos.
    cosBeta1 = max(-1,min(1,cosBeta1));  % Limita o argumento de acos.
    cosBeta2 = max(-1,min(1,cosBeta2));  % Limita o argumento de acos.
    beta1 = acos(cosBeta1);  % Obtém o primeiro ângulo interno do triângulo.
    beta2 = acos(cosBeta2);  % Obtém o segundo ângulo interno do triângulo.
    gamma = atan2(Yed,Xed);  % Calcula a inclinação da linha entre o ombro e o pulso.

    % 4) Escolhe uma das duas configuracoes geometricas do cotovelo.
    switch lower(char(elbowConfiguration))  % Escolhe o ramo de acordo com lower(char(elbowConfiguration)).
        case 'up'  % Executa o ramo 'up'.
            theta1 = pi/2-beta-beta1-gamma;  % Atualiza theta1.
            theta2 = pi-psi-beta2;  % Atualiza theta2.
        case 'down'  % Executa o ramo 'down'.
            theta1 = pi/2-(gamma-beta1+beta);  % Atualiza theta1.
            theta2 = -pi+beta2-psi;  % Atualiza theta2.
        otherwise  % Trata o caso alternativo.
            error('elbowConfiguration deve ser ''up'' ou ''down''.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.

    % 5) Impoe a orientacao pedida da ferramenta.
    theta3 = alpha3-theta1-theta2;  % Compensa o ombro e o cotovelo para manter a orientação da mão.
    joints = [theta0,theta1,theta2,theta3];  % Atualiza joints.

    % 6) Rejeita solucoes fora dos limites articulares do PX100.
    if any(joints < double(minJointPos(1:4))) || ...  % Testa a condição sobre any, joints, double, minJointPos.
            any(joints > double(maxJointPos(1:4)))  % Completa os dados da instrução anterior.
        error('A solucao IK excede os limites de uma ou mais juntas.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.

    % A FK aqui devolve o wrist, igual ao ponto de entrada.
    fkWrist = directKinematicsPX100(joints,links,shoulderHeight);  % Reconstrói o pulso para verificar a solução inversa.
    info = struct('positionError',norm(fkWrist-targetWristBase));  % Reúne os campos numa estrutura.
end  % Fim do bloco.
