function [approach, fingerPath, jointPath, graspError] = ...  % Define as entradas e saídas desta função.
        graspAndLiftPX100(simInterface, cfg, graspTarget, refreshCamera, placement)  % Completa os dados da instrução anterior.
% Move a garra entre poses. Cada estado termina quando o alvo e confirmado.
    if nargin < 4  % Verifica se o argumento opcional foi fornecido.
        refreshCamera = @(phase) [];  % Guarda a função a chamar mais tarde.
    end  % Fim do bloco.
    if nargin < 5  % Verifica se o argumento opcional foi fornecido.
        placement = [];  % Atualiza placement.
    end  % Fim do bloco.

    arm = arm_interface(simInterface, '/px100', 0);  % Atualiza arm.
    [~, links, minQ, maxQ] = arm.get_RobotCharacteristics();  % Atualiza [~, links, minQ, maxQ].
    home = arm.get_joints();  % Guarda os ângulos iniciais para o regresso.
    approach = graspTarget + [0 0 cfg.approachClearance];  % Define o ponto de aproximação acima da pega.
    fingerPath = zeros(0, 3);  % Reserva memória inicializada a zero.
    jointPath = zeros(0, 4);  % Reserva memória inicializada a zero.
    fprintf('Alvo do centro dos dedos [m]: [%.4f %.4f %.4f]\n', graspTarget);  % Apresenta a medição ou o estado na consola.

    state = 'APROXIMAR';  % Define APROXIMAR como próximo estado.
    while ~strcmp(state, 'CONCLUIDO')  % Repete enquanto a condição se mantiver.
        fprintf('Estado: %s\n', state);  % Apresenta a medição ou o estado na consola.
        switch state  % Escolhe o ramo de acordo com state.
            case 'APROXIMAR'  % Executa o ramo 'APROXIMAR'.
                arm.open_hand();  % Chama arm.open_hand com os parâmetros indicados.
                advance(cfg.stepsGripper, 'A abrir a garra');  % Dá tempo ao movimento e atualiza a imagem.
                if isempty(placement)  % Verifica se há dados do apoio para esta operação.
                    moveTo(approach, state);  % Leva o centro dos dedos até ao alvo indicado.
                    moveTo(graspTarget, state);  % Leva o centro dos dedos até ao alvo indicado.
                else  % Trata o caso alternativo.
                    % A garra e horizontal. Baixa do lado da base antes de entrar no copo.
                    radial = graspTarget(1:2) / norm(graspTarget(1:2));  % Obtém a direção horizontal da base para o copo.
                    approach(1:2) = approach(1:2) - cfg.sideApproachDistance * radial;  % Atualiza approach(1:2).
                    moveTo(approach, state);  % Leva o centro dos dedos até ao alvo indicado.
                    graspTarget(3) = graspTarget(3) + cfg.cupHeight/2 - cfg.gripBelowRim;  % Atualiza graspTarget(3).
                    side = [approach(1:2), graspTarget(3)];  % Combina o recuo lateral com a altura de pega.
                    moveTo(side, 'A baixar ao lado do copo');  % Leva o centro dos dedos até ao alvo indicado.
                end  % Fim do bloco.
                state = 'ALINHAR';  % Define ALINHAR como próximo estado.

            case 'ALINHAR'  % Executa o ramo 'ALINHAR'.
                if ~isempty(placement)  % Verifica se há dados do apoio para esta operação.
                    % Usa a distancia real entre o centro dos dedos e o alvo da pega.
                    center = placement.readGripperCenter();  % Mede o ponto médio entre os dedos.
                    cupNow = placement.readCupPosition();  % Lê a posição atual do copo.
                    desired = [cupNow(1:2), graspTarget(3)];  % Usa o centro medido do copo e a altura escolhida.
                    graspTarget = side + (desired - center);  % Corrige o alvo pelo erro medido entre os dedos e o copo.
                    moveTo(graspTarget, 'A entrar com a garra aberta');  % Leva o centro dos dedos até ao alvo indicado.
                end  % Fim do bloco.
                state = 'AGARRAR';  % Define AGARRAR como próximo estado.

            case 'AGARRAR'  % Executa o ramo 'AGARRAR'.
                graspError = NaN;  % Atualiza graspError.
                if ~isempty(placement)  % Verifica se há dados do apoio para esta operação.
                    cupBeforeClose = placement.readCupPosition();  % Lê a posição do copo imediatamente antes de fechar.
                    center = placement.readGripperCenter();  % Mede o ponto médio entre os dedos.
                    graspError = norm(cupBeforeClose(1:2) - center(1:2));  % Calcula a magnitude do desvio.
                    shift = norm(cupBeforeClose(1:2) - placement.cupOrigin(1:2));  % Mede quanto o copo deslizou antes do fecho.
                    fprintf('Antes do fecho: copo deslocou-se %.1f mm, desvio XY na garra %.1f mm.\n', ...  % Apresenta a medição ou o estado na consola.
                        1000*shift, 1000*graspError);  % Converte as distâncias de metros para milímetros.
                    if shift > cfg.cupShiftTolerance || graspError > cfg.cupShiftTolerance  % Testa a condição sobre shift, cfg.cupShiftTolerance, graspError.
                        error('O copo saiu do centro durante a aproximacao. Garra aberta.');  % Interrompe a execução e indica a causa.
                    end  % Fim do bloco.
                end  % Fim do bloco.
                arm.close_hand(cfg.gripperCloseSpeed);  % Chama arm.close_hand com os parâmetros indicados.
                advance(cfg.stepsGripperClose, 'A fechar a garra');  % Dá tempo ao movimento e atualiza a imagem.
                state = 'LEVANTAR';  % Define LEVANTAR como próximo estado.

            case 'LEVANTAR'  % Executa o ramo 'LEVANTAR'.
                if isempty(placement)  % Verifica se há dados do apoio para esta operação.
                    moveTo(approach, state);  % Leva o centro dos dedos até ao alvo indicado.
                    state = 'CONCLUIDO';  % Define CONCLUIDO como próximo estado.
                else  % Trata o caso alternativo.
                    liftHeight = placement.supportTop(3) + cfg.transportClearance;  % Soma a altura do disco à folga de transporte.
                    lift = graspTarget + [0 0 liftHeight];  % Eleva o alvo mantendo as coordenadas horizontais.
                    moveTo(lift, state);  % Leva o centro dos dedos até ao alvo indicado.
                    liftedCup = placement.readCupPosition();  % Mede a posição do copo após a subida.
                    expectedZ = placement.cupOrigin(3) + placement.supportTop(3);  % Estima a altura da origem do copo quando assente no disco.
                    if liftedCup(3) - expectedZ < cfg.minimumCupClearance  % Testa a condição sobre liftedCup, expectedZ, cfg.minimumCupClearance.
                        error('O copo nao ficou suspenso. Transporte cancelado.');  % Interrompe a execução e indica a causa.
                    end  % Fim do bloco.
                    state = 'TRANSPORTAR';  % Define TRANSPORTAR como próximo estado.
                end  % Fim do bloco.

            case 'TRANSPORTAR'  % Executa o ramo 'TRANSPORTAR'.
                % Compensa a posicao do copo relativamente ao alvo da garra.
                startAngle = atan2(lift(2), lift(1));  % Obtém o azimute da posição de pega.
                endAngle = atan2(placement.supportTop(2), placement.supportTop(1));  % Obtém o azimute do disco na base.
                angle = endAngle - startAngle;  % Atualiza angle.
                rotation = [cos(angle) -sin(angle); sin(angle) cos(angle)];  % Constrói a rotação no plano da mesa.
                offsetXY = (rotation * (liftedCup(1:2)-lift(1:2))')';  % Roda o desvio copo–garra para a orientação de entrega.
                overSupport = [placement.supportTop(1:2)-offsetXY, lift(3)];  % Desconta o desvio para centrar o copo no disco.
                moveTo(overSupport, state);  % Leva o centro dos dedos até ao alvo indicado.
                state = 'POUSAR';  % Define POUSAR como próximo estado.

            case 'POUSAR'  % Executa o ramo 'POUSAR'.
                cupOverSupport = placement.readCupPosition();  % Guarda a posição antes de verificar a estabilidade.
                advance(cfg.graspStableSteps, 'A confirmar o copo sobre o apoio');  % Dá tempo ao movimento e atualiza a imagem.
                cupNow = placement.readCupPosition();  % Lê a posição atual do copo.
                gap = cupNow(3) - expectedZ;  % Mede a folga vertical relativamente à posição de repouso.
                correction = placement.supportTop(1:2) - cupNow(1:2);  % Calcula o deslocamento horizontal até ao centro do disco.
                if gap < cfg.minimumCupClearance || ...  % Testa a condição sobre gap, cfg.minimumCupClearance.
                        norm(cupNow-cupOverSupport) > cfg.cupShiftTolerance || ...  % Completa os dados da instrução anterior.
                        norm(correction) > cfg.cupTrackingTolerance  % Completa os dados da instrução anterior.
                    error('Largada cancelada: o copo perdeu altura, esta a mover-se ou esta fora do apoio.');  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
                drop = overSupport + [correction, min(0, cfg.releaseClearance-gap)];  % Corrige X e Y e limita a descida à folga de largada.
                moveTo(drop, 'A descer para o apoio');  % Leva o centro dos dedos até ao alvo indicado.
                cupNow = placement.readCupPosition();  % Lê a posição atual do copo.
                if norm(cupNow(1:2)-placement.supportTop(1:2)) > placement.supportRadius || ...  % Testa a condição sobre norm, cupNow, placement.supportTop, placement.supportRadius.
                        abs(cupNow(3)-expectedZ) > cfg.placementTolerance  % Completa os dados da instrução anterior.
                    error('Largada cancelada: o copo nao chegou ao apoio.');  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
                arm.open_hand();  % Chama arm.open_hand com os parâmetros indicados.
                advance(cfg.stepsGripper, 'A largar o copo');  % Dá tempo ao movimento e atualiza a imagem.
                state = 'SUBIR_APOS_LARGAR';  % Define SUBIR_APOS_LARGAR como próximo estado.

            case 'SUBIR_APOS_LARGAR'  % Executa o ramo 'SUBIR_APOS_LARGAR'.
                % Mantem X e Y da largada ate ambos os dedos passarem o rebordo.
                clearZ = placement.supportTop(3) + cfg.cupHeight + cfg.rimClearance;  % Determina a altura que os dois dedos devem ultrapassar.
                retreat = drop;  % Parte da posição onde a garra abriu.
                for attempt = 1:20  % Percorre os valores do índice attempt.
                    remaining = clearZ - placement.readFingerBottom();  % Mede quanto falta subir para passar o rebordo.
                    if remaining <= 0, break; end  % Termina a subida quando os dedos ultrapassam o rebordo.
                    retreat(3) = retreat(3) + min(0.005, remaining + 0.001);  % Atualiza retreat(3).
                    moveTo(retreat, state);  % Leva o centro dos dedos até ao alvo indicado.
                end  % Fim do bloco.
                if placement.readFingerBottom() < clearZ  % Testa a condição sobre placement.readFingerBottom, clearZ.
                    error('A garra ainda esta abaixo da altura de retirada.');  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
                state = 'VOLTAR_A_ORIGEM';  % Define VOLTAR_A_ORIGEM como próximo estado.

            case 'VOLTAR_A_ORIGEM'  % Executa o ramo 'VOLTAR_A_ORIGEM'.
                moveTo([lift(1:2), retreat(3)], 'A afastar do dispensador');  % Leva o centro dos dedos até ao alvo indicado.
                moveJoints(home, state);  % Chama moveJoints com os parâmetros indicados.
                cupFinal = placement.readCupPosition();  % Lê a posição final para confirmar a largada.
                if norm(cupFinal(1:2)-placement.supportTop(1:2)) > ...  % Testa a condição sobre norm, cupFinal, placement.supportTop.
                        min(placement.supportRadius, cfg.placementTolerance) || ...  % Completa os dados da instrução anterior.
                        abs(cupFinal(3)-expectedZ) > cfg.placementTolerance  % Completa os dados da instrução anterior.
                    error('O copo ficou fora da posicao esperada no apoio %s.', placement.supportPath);  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
                state = 'CONCLUIDO';  % Define CONCLUIDO como próximo estado.
        end  % Fim do bloco.
    end  % Fim do bloco.

    function moveTo(target, phase)  % Define as entradas e saídas desta função.
        % A IK recebe o pulso. Recua do centro dos dedos na direcao da garra.
        waist = atan2(target(2), target(1));  % Orienta a base na direção do alvo dos dedos.
        direction = [cos(cfg.toolPitch)*cos(waist), ...  % Atualiza direction.
            cos(cfg.toolPitch)*sin(waist), -sin(cfg.toolPitch)];  % Completa os dados da instrução anterior.
        wrist = target - cfg.wristToFingerCenter*direction;  % Recua dos dedos até ao pulso ao longo da ferramenta.
        joints = inverseKinematicsPX100(wrist, links, minQ, maxQ, cfg.toolPitch, [], cfg.elbow);  % Calcula os ângulos para o pulso atingir o alvo.
        moveJoints(joints, phase);  % Chama moveJoints com os parâmetros indicados.
        fingerPath(end+1,:) = target;  % Atualiza fingerPath(end+1,:).
        jointPath(end+1,:) = joints;  % Atualiza jointPath(end+1,:).
    end  % Fim do bloco.

    function moveJoints(target, phase)  % Define as entradas e saídas desta função.
        initial = arm.get_joints();  % Lê os ângulos atuais.
        delta = target - initial(:)';  % Calcula quanto cada junta tem de rodar.
        % Todas as juntas avancam na mesma proporcao ate a pose pedida.
        steps = max(1, ceil(max(abs(delta)) / cfg.maxJointStep));  % Divide o maior deslocamento pelo passo angular permitido.
        for k = 1:steps  % Percorre os valores do índice k.
            if arm.set_joints(initial(:)' + (k/steps)*delta) ~= 0  % Testa a condição sobre arm.set_joints, initial, k, steps, delta.
                error('O braco rejeitou o alvo durante %s.', phase);  % Interrompe a execução e indica a causa.
            end  % Fim do bloco.
            advance(1, phase);  % Dá tempo ao movimento e atualiza a imagem.
            pause(cfg.motionPause);  % Mantém a fase visível durante o intervalo configurado.
        end  % Fim do bloco.
        stableSteps = 0;  % Define stableSteps.
        for step = 1:cfg.maxWaitSteps  % Percorre os valores do índice step.
            advance(1, phase);  % Dá tempo ao movimento e atualiza a imagem.
            difference = mod(arm.get_joints()-target+pi, 2*pi)-pi;  % Mede o erro angular pelo caminho mais curto.
            if max(abs(difference)) <= cfg.graspJointTolerance  % Exige que todas as juntas estejam dentro da tolerância.
                stableSteps = stableSteps + 1;  % Atualiza stableSteps.
                if stableSteps >= cfg.graspStableSteps  % Confirma que a pose se manteve durante vários passos.
                    return;  % Regressa à função que fez a chamada.
                end  % Fim do bloco.
            else  % Trata o caso alternativo.
                stableSteps = 0;  % Define stableSteps.
            end  % Fim do bloco.
        end  % Fim do bloco.
        error('As juntas nao chegaram ao alvo durante %s.', phase);  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.

    function advance(count, phase)  % Define as entradas e saídas desta função.
        for step = 1:count  % Percorre os valores do índice step.
            simInterface.step_simulation();  % Avança um passo de simulação.
            refreshCamera(phase);  % Chama refreshCamera com os parâmetros indicados.
        end  % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.
