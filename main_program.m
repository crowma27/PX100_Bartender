function result = main_program(cupColor, supportPath)  % Executa um ciclo para o copo e o apoio escolhidos.
% Deteta um copo pela cor e pousa-o num apoio lilas do dispensador.
% Abre Cenario_px100_vision.ttt no CoppeliaSim e executa main_program.
% Sem argumentos, usa o copo vermelho e o apoio lilas mais proximo.

    if nargin < 1  % Verifica se o argumento opcional foi fornecido.
        cupColor = 'red';  % Usa o copo vermelho por defeito.
    end  % Fim do bloco.
    if nargin < 2  % Verifica se o argumento opcional foi fornecido.
        supportPath = '';  % Deixa o programa escolher o apoio mais próximo.
    end  % Fim do bloco.

    %% 1. Ligar ao simulador
    addProjectPaths();  % Disponibiliza as funções de visão e cinemática.
    cfg = pickConfigPX100();  % Carrega dimensões, passos e tolerâncias da pega.
    simInterface = connectToSimulator();  % Abre a ligação ao CoppeliaSim.
    cleanup = onCleanup(@() stopSimulation(simInterface));  % Regista a ação de fecho automático.
    simInterface.start_simulation();  % Inicia a física da cena em modo de passos.
    simInterface.step_simulation(); % Primeiro passo para gerar a imagem.

    %% 2. Detetar o copo e localizar o apoio na cena
    fprintf('Estado: DETETAR\n');  % Apresenta a medição ou o estado na consola.
    objectInfo = runVisionCupsPX100(simInterface, true, cfg.cupHeight);  % Deteta os copos e projeta os centros na base do braço.
    refreshCamera = openCameraViewPX100(simInterface, objectInfo.image);  % Abre a janela em direto e guarda a função de atualização.
    cup = findCup(objectInfo, cupColor);  % Seleciona a deteção da cor pedida.
    grasp = cup.graspTargetBase; % Centro da pega, em metros na base do PX100.
    placement = findPlacementPX100(simInterface, cup, supportPath);  % Escolhe o disco a partir do XY da câmara.
    fprintf('Apoio escolhido: %s [%.4f %.4f %.4f] m.\n', ...  % Apresenta a medição ou o estado na consola.
        placement.supportPath, placement.supportTop);  % Mostra o caminho do disco e o centro da sua face superior.

    %% 3. Pegar, transportar e pousar
    [approach, fingerPath, jointPath, graspError] = ...  % Recolhe os alvos de movimento e o erro antes do fecho.
        graspAndLiftPX100(simInterface, cfg, grasp, refreshCamera, placement);  % Executa a máquina de estados da pega e da largada.

    % Mantem a pose final visivel antes de parar a simulacao.
    refreshCamera('Posicao final');  % Atualiza a imagem após o regresso à origem.
    fprintf('Sequencia de largada concluida. A manter a pose durante %.0f segundos.\n', ...  % Apresenta a medição ou o estado na consola.
        cfg.finalHoldSeconds);  % Indica o tempo de observação da pose final.
    pause(cfg.finalHoldSeconds);  % Mantém a fase visível durante o intervalo configurado.

    result = struct('cupTarget', grasp, 'approachTarget', approach, ...  % Reúne os campos numa estrutura.
        'fingerPath', fingerPath, 'jointPath', jointPath, ...  % Guarda os alvos dos dedos e os ângulos correspondentes.
        'graspError', graspError, 'supportPath', placement.supportPath, ...  % Guarda o erro de centragem e o nome do disco.
        'supportTop', placement.supportTop, 'detectedCups', objectInfo.cups, ...  % Inclui a posição do disco e as deteções da câmara.
        'cfg', cfg);  % Regista os parâmetros usados nesta execução.
end  % Fim do bloco.

function addProjectPaths()  % Configura as pastas de pesquisa do MATLAB.
    projectRoot = fileparts(mfilename('fullpath'));  % Localiza a pasta deste ficheiro mesmo que a pasta atual seja outra.
    addpath(projectRoot, fullfile(projectRoot,'vision'), ...  % Inclui a pasta principal e as funções de visão.
        fullfile(projectRoot,'kinematics'));  % Inclui as funções que convertem posições em ângulos.
end  % Fim do bloco.

function simInterface = connectToSimulator()  % Cria a interface com a cena e verifica a ligação.
    [simInterface, simError] = simulator_interface(1);  % Ativa a leitura da câmara ao criar a interface.
    if simError ~= 0  % Deteta uma falha na inicialização do simulador.
        error('Nao foi possivel ligar ao CoppeliaSim.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
end  % Fim do bloco.

function stopSimulation(simInterface)  % Encerra a simulação no fim da execução.
    % Fecha a simulacao no fim da funcao, mesmo em caso de erro.
    try  % Tenta executar a operação seguinte.
        simInterface.terminate();  % Pede ao CoppeliaSim para parar a cena.
    catch  % Trata a falha da operação anterior.
    end  % Fim do bloco.
end  % Fim do bloco.

function cup = findCup(objectInfo, cupColor)  % Procura um copo pela cor nas deteções disponíveis.
    if isempty(objectInfo.cups)  % Verifica se a câmara encontrou algum copo.
        error('Nao foram detetados copos com rebordo preto.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    idx = find(strcmpi({objectInfo.cups.color}, cupColor), 1);  % Procura a primeira cor igual, ignorando maiúsculas.
    if isempty(idx)  % Deteta a ausência da cor pedida.
        error('O copo de cor %s nao foi detetado.', cupColor);  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    cup = objectInfo.cups(idx);  % Devolve a deteção selecionada.
end  % Fim do bloco.
