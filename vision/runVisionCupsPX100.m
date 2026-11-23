function objectInfo = runVisionCupsPX100(simInterface,showOutput,cupHeight)  % Obtém o ponto de pega de cada copo na base do robô.
% Captura uma imagem e converte os centros dos copos de pixeis para metros.

    if nargin < 2 || isempty(showOutput), showOutput = true; end  % Verifica se o argumento opcional foi fornecido.
    if nargin < 3 || isempty(cupHeight), cupHeight = 0.070; end  % Verifica se o argumento opcional foi fornecido.
    tableZ = 0.0;  % Define o tampo da mesa como altura zero.

    [image,~] = simInterface.get_camera_information();  % Captura uma imagem da câmara.
    image = uint8(image);  % Representa cada canal da imagem com oito bits.
    [cups,debug] = detectCupsPX100(image);  % Recolhe as deteções e as máscaras para visualização.

    for k = 1:numel(cups)  % Calcula uma posição para cada copo detetado.
        % Pixel do centro do rebordo -> intersecao com o plano do topo.
        topCenter = calibrationPX100('projectplane', ...  % Projeta o centro do rebordo no plano horizontal do topo.
            cups(k).centroid,tableZ+cupHeight,false);  % Usa o píxel medido e a altura conhecida do rebordo.
        cups(k).topCenterBase = topCenter;  % Guarda as coordenadas do topo em metros.

        % O X/Y mantem-se; Z desce meia altura para o centro do copo.
        cups(k).graspTargetBase = [topCenter(1:2), ...  % Usa as coordenadas horizontais do centro do rebordo.
            tableZ+cupHeight/2];  % Coloca o alvo inicial a meia altura do copo.
    end  % Fim do bloco.

    objectInfo = struct('image',image,'cups',cups, ...  % Reúne os campos numa estrutura.
        'cupHeight',cupHeight,'tableZ',tableZ);  % Inclui as alturas usadas na projeção.

    if showOutput  % Verifica se foram pedidos gráficos da deteção.
        showCupVisionPX100(image, cups, debug, tableZ+cupHeight);  % Mostra as máscaras, os centros e a projeção.
    end  % Fim do bloco.
end  % Fim do bloco.
