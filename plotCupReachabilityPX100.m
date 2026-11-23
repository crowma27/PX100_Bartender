function workspace = plotCupReachabilityPX100(simInterface,cupInfo)  % Define as entradas e saídas desta função.
% PLOTCUPREACHABILITYPX100 Varre o plano XY sem mover o robot nem o copo.
% Cada celula testa a IK elbow-down e o trajeto vertical de pega.

    if nargin < 2, cupInfo = []; end  % Verifica se o argumento opcional foi fornecido.
    arm = arm_interface(simInterface,'/px100',0);  % Atualiza arm.
    [~,links,minJointPos,maxJointPos] = arm.get_RobotCharacteristics();  % Atualiza [~,links,minJointPos,maxJointPos].

    % Mantem os mesmos parametros usados pela rotina de pega.
    alpha3 = pi/2;               % soma theta1+theta2+theta3: para baixo
    elbow = 'down';  % Atualiza elbow.
    tableZ = 0.0;  % Define tableZ.
    cupHeight = 0.070;  % Define cupHeight.
    tcpToFingerCenter = 0.0431;  % offset empirico TCP-centro dos dedos
    wristToFingerCenter = double(links(5))-tcpToFingerCenter;  % Atualiza wristToFingerCenter.
    clearance = 0.01;  % Define clearance.
    descentStep = 0.005;  % Define descentStep.
    safetyMargin = 0.03;          % 2.1 cm medidos + margem adicional
    graspZ = tableZ+cupHeight/2;  % Atualiza graspZ.

    % Grelha de 1 cm, centrada na base do PX100 (origem [0,0,0]).
    xValues = -0.35:0.01:0.35;  % Atualiza xValues.
    yValues = -0.35:0.01:0.35;  % Atualiza yValues.
    graspReachable = false(numel(yValues),numel(xValues));  % Conta os elementos.
    approachReachable = false(numel(yValues),numel(xValues));  % Conta os elementos.
    nominalReachable = false(numel(yValues),numel(xValues));  % Conta os elementos.
    maxPositionError = 0;  % Define maxPositionError.

    for ix = 1:numel(xValues)  % Percorre os valores do índice ix.
        for iy = 1:numel(yValues)  % Percorre os valores do índice iy.
            fingerTarget = [xValues(ix),yValues(iy),graspZ];  % Atualiza fingerTarget.
            wristTarget = fingerTarget + [0 0 wristToFingerCenter];  % Atualiza wristTarget.
            wristApproach = wristTarget + [0 0 cupHeight/2+clearance];  % Atualiza wristApproach.
            try  % Tenta executar a operação seguinte.
                [joints,~] = inverseKinematicsPX100( ...  % Calcula os ângulos para o pulso atingir o alvo.
                    wristTarget,links,minJointPos,maxJointPos,alpha3,[],elbow);  % Completa os dados da instrução anterior.
                graspReachable(iy,ix) = norm( ...  % Calcula a magnitude do desvio.
                    directKinematicsPX100(joints,links)-wristTarget) <= 1e-5;  % Atualiza directKinematicsPX100(joints,links)-wristTarget) <.
            catch  % Trata a falha da operação anterior.
                graspReachable(iy,ix) = false;  % Atualiza graspReachable(iy,ix).
            end  % Fim do bloco.
            try  % Tenta executar a operação seguinte.
                [joints,~] = inverseKinematicsPX100( ...  % Calcula os ângulos para o pulso atingir o alvo.
                    wristApproach,links,minJointPos,maxJointPos,alpha3,[],elbow);  % Completa os dados da instrução anterior.
                approachReachable(iy,ix) = norm( ...  % Calcula a magnitude do desvio.
                    directKinematicsPX100(joints,links)-wristApproach) <= 1e-5;  % Atualiza directKinematicsPX100(joints,links)-wristApproach) <.
            catch  % Trata a falha da operação anterior.
                approachReachable(iy,ix) = false;  % Atualiza approachReachable(iy,ix).
            end  % Fim do bloco.
            nSteps = max(1,ceil((wristApproach(3)-wristTarget(3))/descentStep));  % Atualiza nSteps.
            zPath = linspace(wristApproach(3),wristTarget(3),nSteps+1);  % Atualiza zPath.
            pathOk = true;  % Atualiza pathOk.
            for iz = 1:numel(zPath)  % Percorre os valores do índice iz.
                point = [wristTarget(1:2),zPath(iz)];  % Atualiza point.
                try  % Tenta executar a operação seguinte.
                    [joints,ikInfo] = inverseKinematicsPX100( ...  % Calcula os ângulos para o pulso atingir o alvo.
                        point,links,minJointPos,maxJointPos,alpha3,[],elbow);  % Completa os dados da instrução anterior.
                    fkTarget = directKinematicsPX100(joints,links);  % Atualiza fkTarget.
                    positionError = norm(fkTarget-point);  % Calcula a magnitude do desvio.
                    if ~isfinite(positionError) || positionError > 1e-5 || ...  % Testa a condição sobre isfinite, positionError.
                            ikInfo.positionError > 1e-5  % Completa os dados da instrução anterior.
                        pathOk = false;  % Atualiza pathOk.
                        break  % Termina o ciclo atual.
                    end  % Fim do bloco.
                    maxPositionError = max(maxPositionError,positionError);  % Atualiza maxPositionError.
                catch  % Trata a falha da operação anterior.
                    % Sem solucao IK ou fora dos limites em algum ponto do trajeto.
                    pathOk = false;  % Atualiza pathOk.
                    break  % Termina o ciclo atual.
                end  % Fim do bloco.
            end  % Fim do bloco.
            nominalReachable(iy,ix) = pathOk;  % Atualiza nominalReachable(iy,ix).
        end  % Fim do bloco.
    end  % Fim do bloco.

    % Erosao espacial: descarta pontos a menos de 3 cm da fronteira nominal.
    marginCells = ceil(safetyMargin/0.01);  % Atualiza marginCells.
    robustReachable = imerode(nominalReachable,strel('disk',marginCells,0));  % Atualiza robustReachable.
    % 0=grasp inalcançavel; 1=grasp alcancavel mas hover inalcançavel;
    % 2=trajeto nominal alcancavel; 3=trajeto com margem de erro.
    reachabilityClass = zeros(size(nominalReachable));  % Reserva memória inicializada a zero.
    reachabilityClass(graspReachable & ~nominalReachable) = 1;  % Define reachabilityClass(graspReachable & ~nominalReachable).
    reachabilityClass(nominalReachable) = 2;  % Define reachabilityClass(nominalReachable).
    reachabilityClass(robustReachable) = 3;  % Define reachabilityClass(robustReachable).
    reachableCount = nnz(robustReachable);  % Atualiza reachableCount.
    nominalCount = nnz(nominalReachable);  % Atualiza nominalCount.
    graspCount = nnz(graspReachable);  % Atualiza graspCount.
    approachCount = nnz(approachReachable);  % Atualiza approachCount.
    totalCount = numel(nominalReachable);  % Conta os elementos.
    workspace = struct('xValues',xValues,'yValues',yValues, ...  % Reúne os campos numa estrutura.
        'nominalReachable',nominalReachable, ...  % Completa os dados da instrução anterior.
        'graspReachable',graspReachable, ...  % Completa os dados da instrução anterior.
        'approachReachable',approachReachable, ...  % Completa os dados da instrução anterior.
        'robustReachable',robustReachable, ...  % Completa os dados da instrução anterior.
        'reachabilityClass',reachabilityClass, ...  % Completa os dados da instrução anterior.
        'safetyMargin',safetyMargin,'graspZ',graspZ,'alpha3',alpha3, ...  % Completa os dados da instrução anterior.
        'elbowConfiguration',elbow,'reachableCount',reachableCount, ...  % Completa os dados da instrução anterior.
        'nominalCount',nominalCount,'graspCount',graspCount, ...  % Completa os dados da instrução anterior.
        'approachCount',approachCount, ...  % Completa os dados da instrução anterior.
        'totalCount',totalCount,'reachablePercent',100*reachableCount/totalCount, ...  % Completa os dados da instrução anterior.
        'maxIkFkError',maxPositionError);  % Completa os dados da instrução anterior.

    figure('Name','Workspace de pega do copo vermelho');  % Chama figure com os parâmetros indicados.
    ax = axes();  % Atualiza ax.
    imagesc(ax,xValues,yValues,reachabilityClass);  % Chama imagesc com os parâmetros indicados.
    axis(ax,'xy'); axis(ax,'equal'); axis(ax,'tight');  % Chama axis com os parâmetros indicados.
    colormap(ax,[0.82 0.20 0.20; 0.95 0.70 0.18; 0.25 0.55 0.88; 0.20 0.68 0.28]);  % Chama colormap com os parâmetros indicados.
    clim(ax,[-0.5 3.5]);  % Chama clim com os parâmetros indicados.
    cb = colorbar(ax);  % Atualiza cb.
    cb.Ticks = [0 1 2 3];  % Atualiza cb.Ticks.
    cb.TickLabels = {'Centro sem IK','Centro ok; trajeto falha', ...  % Atualiza cb.TickLabels.
        'Trajeto nominal','Trajeto robusto (+/- 3 cm)'};  % Completa os dados da instrução anterior.
    hold(ax,'on');  % Chama hold com os parâmetros indicados.
    if isstruct(cupInfo) && isfield(cupInfo,'graspTargetBase') && ...  % Testa a condição sobre isstruct, cupInfo, isfield, graspTargetBase.
            ~isempty(cupInfo.graspTargetBase)  % Completa os dados da instrução anterior.
        cupMarker = plot(ax,cupInfo.graspTargetBase(1),cupInfo.graspTargetBase(2), ...  % Atualiza cupMarker.
            'kp','MarkerSize',13,'MarkerFaceColor','y','LineWidth',1.5);  % Completa os dados da instrução anterior.
        legend(ax,cupMarker,{'Posicao detetada do copo'},'Location','best');  % Chama legend com os parâmetros indicados.
    end  % Fim do bloco.
    xlabel(ax,'X relativamente a /px100 (m)');  % Chama xlabel com os parâmetros indicados.
    ylabel(ax,'Y relativamente a /px100 (m)');  % Chama ylabel com os parâmetros indicados.
    title(ax,sprintf(['Alcance IK para pega vertical | z dedo = %.3f m | ' ...  % Atualiza title(ax,sprintf(['Alcance IK para pega vertical | z dedo.
        'elbow down | grelha 1 cm'],graspZ));  % Completa os dados da instrução anterior.
    grid(ax,'on');  % Chama grid com os parâmetros indicados.
    hold(ax,'off');  % Chama hold com os parâmetros indicados.
    drawnow;  % Especifica drawnow.

    fprintf('Workspace calculado sem mover o robot nem o copo.\n');  % Apresenta a medição ou o estado na consola.
    fprintf('Centro de pega alcancavel: %d/%d.\n',graspCount,totalCount);  % Apresenta a medição ou o estado na consola.
    fprintf('Ponto de aproximacao alcancavel: %d/%d.\n',approachCount,totalCount);  % Apresenta a medição ou o estado na consola.
    fprintf('Trajeto IK nominal completo: %d/%d.\n',nominalCount,totalCount);  % Apresenta a medição ou o estado na consola.
    fprintf('Pontos robustos com margem XY de %.0f mm: %d/%d (%.1f%%).\n', ...  % Apresenta a medição ou o estado na consola.
        safetyMargin*1000,reachableCount,totalCount,workspace.reachablePercent);  % Completa os dados da instrução anterior.
    fprintf('Altura do centro dos dedos: %.3f m; orientacao para baixo; elbow down.\n', ...  % Apresenta a medição ou o estado na consola.
        graspZ);  % Completa os dados da instrução anterior.
    fprintf('Erro maximo IK/FK nos pontos alcancaveis: %.3g m.\n',maxPositionError);  % Apresenta a medição ou o estado na consola.
    disp('A classificacao considera IK e limites articulares; nao testa colisoes.');  % Chama disp com os parâmetros indicados.
end  % Fim do bloco.
