function varargout = calibrationPX100(action, varargin)  % Define as entradas e saídas desta função.
% Ferramentas de calibracao e geometria da camera do PX100.
% Comandos: capture, calibrate, transform, validate, inspect, project,
% projectplane, originpixel.

    if nargin == 0 || isempty(action)  % Verifica se o argumento opcional foi fornecido.
        error(['Indica um comando: capture, calibrate, transform, ' ...  % Interrompe a execução e indica a causa.
            'validate, inspect, project, projectplane ou originpixel.']);  % Completa os dados da instrução anterior.
    end  % Fim do bloco.
    switch lower(char(action))  % Escolhe o ramo de acordo com lower(char(action)).
        case 'capture'  % Executa o ramo 'capture'.
            captureImages(varargin{:});  % Chama captureImages com os parâmetros indicados.
            outputs = {};  % Atualiza outputs.
        case 'calibrate'  % Executa o ramo 'calibrate'.
            [cameraParams, estimationErrors] = calibrateIntrinsics(varargin{:});  % Atualiza [cameraParams, estimationErrors].
            outputs = {cameraParams, estimationErrors};  % Atualiza outputs.
        case 'transform'  % Executa o ramo 'transform'.
            T_base_camera = getCameraTransform(varargin{:});  % Atualiza T_base_camera.
            outputs = {T_base_camera};  % Atualiza outputs.
        case 'validate'  % Executa o ramo 'validate'.
            validation = validateCalibration(varargin{:});  % Atualiza validation.
            outputs = {validation};  % Atualiza outputs.
        case 'inspect'  % Executa o ramo 'inspect'.
            objects = inspectScene(varargin{:});  % Atualiza objects.
            outputs = {objects};  % Atualiza outputs.
        case 'project'  % Executa o ramo 'project'.
            pointsBase = projectPixelsToObjectTop(varargin{:});  % Atualiza pointsBase.
            outputs = {pointsBase};  % Atualiza outputs.
        case 'projectplane'  % Executa o ramo 'projectplane'.
            pointsBase = projectPixelsToHorizontalPlane(varargin{:});  % Atualiza pointsBase.
            outputs = {pointsBase};  % Atualiza outputs.
        case 'originpixel'  % Executa o ramo 'originpixel'.
            originPixel = projectBaseOrigin(varargin{:});  % Atualiza originPixel.
            outputs = {originPixel};  % Atualiza outputs.
        otherwise  % Trata o caso alternativo.
            error('Comando desconhecido: %s', char(action));  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    if nargout > 0  % Testa a condição sobre nargout.
        varargout = outputs(1:min(nargout, numel(outputs)));  % Conta os elementos.
    end  % Fim do bloco.
end  % Fim do bloco.


function pointsBase = projectPixelsToHorizontalPlane(pixelPoints,planeZ,verbose)  % Define as entradas e saídas desta função.
% Projeta pixeis para um plano horizontal z=planeZ, na base /px100.
% Usa apenas a calibracao guardada, sem consultar a pose de um objeto na cena.
    if nargin < 3 || isempty(verbose), verbose = true; end  % Verifica se o argumento opcional foi fornecido.
    if size(pixelPoints,2) ~= 2 || isempty(pixelPoints)  % Testa a condição sobre size, pixelPoints, isempty.
        error('pixelPoints deve ser uma matriz N x 2 [x y].');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    if ~isscalar(planeZ) || ~isfinite(planeZ)  % Testa a condição sobre isscalar, planeZ, isfinite.
        error('planeZ deve ser uma coordenada Z finita em metros.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    [calibrationFolder,~] = projectFolders();  % Atualiza [calibrationFolder,~].
    data = load(fullfile(calibrationFolder,'cameraCalibration.mat'));  % Constrói o caminho do ficheiro.
    required = {'cameraParams','T_base_camera','cameraToPinhole'};  % Atualiza required.
    if ~all(isfield(data,required))  % Testa a condição sobre all, isfield, data, required.
        error('Executa primeiro calibrate, transform e validate.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.

    % Converte do referencial usado no modelo da imagem para a base do PX100.
    T_base_pinhole = double(data.T_base_camera)/double(data.cameraToPinhole);  % Atualiza T_base_pinhole.
    cameraOrigin = T_base_pinhole(1:3,4);  % Extrai a posição da câmara no referencial da base.
    cameraRotation = T_base_pinhole(1:3,1:3);  % Extrai a rotação que leva direções da câmara à base.
    undistorted = undistortPoints(double(pixelPoints),data.cameraParams);  % Corrige o desvio dos píxeis causado pela lente.
    intrinsics = data.cameraParams.Intrinsics;  % Atualiza intrinsics.
    focal = double(intrinsics.FocalLength);  % Lê as distâncias focais horizontal e vertical.
    principal = double(intrinsics.PrincipalPoint);  % Lê a posição do ponto principal na imagem.
    % Cada pixel sem distorcao define a direcao de um raio na camera.
    rays = [(undistorted(:,1)-principal(1))/focal(1), ...  % Normaliza os píxeis para construir raios na câmara.
        (undistorted(:,2)-principal(2))/focal(2),ones(size(pixelPoints,1),1)];  % Completa os dados da instrução anterior.

    pointsBase = zeros(size(pixelPoints,1),3);  % Reserva memória inicializada a zero.
    for k = 1:size(pixelPoints,1)  % Percorre os valores do índice k.
        direction = cameraRotation*rays(k,:)';  % Atualiza direction.
        if abs(direction(3)) < 1e-10  % Testa a condição sobre abs, direction.
            error('O raio do pixel [%g %g] e paralelo ao plano z=%.4f.', ...  % Atualiza error('O raio do pixel [%g %g] e paralelo ao plano z.
                pixelPoints(k,1),pixelPoints(k,2),planeZ);  % Completa os dados da instrução anterior.
        end  % Fim do bloco.
        % Resolve origemZ + distance * direcaoZ = planeZ.
        distance = (planeZ-cameraOrigin(3))/direction(3);  % Intersecta o raio com o plano à altura conhecida.
        if distance <= 0  % Testa a condição sobre distance.
            error('O plano z=%.4f esta atras da camera para o pixel [%g %g].', ...  % Atualiza error('O plano z.
                planeZ,pixelPoints(k,1),pixelPoints(k,2));  % Completa os dados da instrução anterior.
        end  % Fim do bloco.
        pointsBase(k,:) = (cameraOrigin+distance*direction)';  % Atualiza pointsBase(k,:).
        pointsBase(k,3) = planeZ;  % Atualiza pointsBase(k,3).
    end  % Fim do bloco.
    if verbose  % Testa a condição sobre verbose.
        fprintf('XYZ no referencial /px100 (m), plano horizontal z=%.4f:\n',planeZ);  % Atualiza fprintf('XYZ no referencial /px100 (m), plano horizontal z.
        disp(pointsBase);  % Chama disp com os parâmetros indicados.
    end  % Fim do bloco.
end  % Fim do bloco.


function captureImages(numberOfImages, outputFolder)  % Define as entradas e saídas desta função.
    [calibrationFolder, projectFolder] = projectFolders();  % Atualiza [calibrationFolder, projectFolder].
    if nargin < 1 || isempty(numberOfImages), numberOfImages = 15; end  % Verifica se o argumento opcional foi fornecido.
    if nargin < 2 || isempty(outputFolder)  % Verifica se o argumento opcional foi fornecido.
        outputFolder = fullfile(calibrationFolder, 'images');  % Constrói o caminho do ficheiro.
    end  % Fim do bloco.
    if numberOfImages < 3 || numberOfImages > 15 || numberOfImages ~= floor(numberOfImages)  % Testa a condição sobre numberOfImages, floor.
        error('numberOfImages deve ser um inteiro entre 3 e 15.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    if ~exist(outputFolder, 'dir'), mkdir(outputFolder); end  % Testa a condição sobre exist, outputFolder, dir, mkdir, end.

    [client, sim] = connectRemote(projectFolder);  % Atualiza [client, sim].
    chessHandle = sim.getObject('/chess');  % Obtém a referência do objeto da cena.
    cameraHandle = sim.getObject('/Vision_sensor');  % Obtém a referência do objeto da cena.
    originalPosition = asRow(sim.getObjectPosition(chessHandle, sim.handle_world));  % Lê a posição do objeto no referencial pedido.
    originalOrientation = asRow(sim.getObjectOrientation(chessHandle, sim.handle_world));  % Obtém a referência do objeto da cena.

    % Cada linha: [dx dy dz dAlpha dBeta dGamma], metros e graus.
    poseOffsets = [ ...  % Atualiza poseOffsets.
         0.00  0.00 0.000   0    0    0; ...  % Deslocamento e orientação do padrão nesta imagem.
        -0.02  0.00 0.055   0    8    0; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.02  0.00 0.055   0   -8    0; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.00 -0.02 0.055   8    0    0; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.00  0.02 0.055  -8    0    0; ...  % Deslocamento e orientação do padrão nesta imagem.
        -0.02 -0.02 0.065   7    7    0; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.02  0.02 0.065  -7   -7    0; ...  % Deslocamento e orientação do padrão nesta imagem.
        -0.02  0.01 0.065   5   -5   10; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.02 -0.01 0.065  -5    5  -10; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.00  0.01 0.070  10    0    5; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.00 -0.01 0.070 -10    0   -5; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.01  0.00 0.075   0   10   10; ...  % Deslocamento e orientação do padrão nesta imagem.
        -0.01  0.00 0.075   0  -10  -10; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.00  0.01 0.090  10  -10   15; ...  % Deslocamento e orientação do padrão nesta imagem.
         0.00 -0.01 0.090 -10   10  -15];  % Deslocamento e orientação do padrão nesta imagem.

    cleanup = onCleanup(@()restoreScene(sim, chessHandle, ...  % Regista a ação de fecho automático.
        originalPosition, originalOrientation));  % Completa os dados da instrução anterior.
    sim.setStepping(true);  % Chama sim.setStepping com os parâmetros indicados.
    sim.startSimulation();  % Chama sim.startSimulation com os parâmetros indicados.
    fprintf('A capturar %d imagens do tabuleiro /chess.\n', numberOfImages);  % Apresenta a medição ou o estado na consola.
    for k = 1:numberOfImages  % Percorre os valores do índice k.
        offset = poseOffsets(k,:);  % Atualiza offset.
        position = originalPosition + offset(1:3);  % Atualiza position.
        orientation = originalOrientation + deg2rad(offset(4:6));  % Atualiza orientation.
        sim.setObjectPosition(chessHandle, position, sim.handle_world);  % Chama sim.setObjectPosition com os parâmetros indicados.
        sim.setObjectOrientation(chessHandle, orientation, sim.handle_world);  % Chama sim.setObjectOrientation com os parâmetros indicados.
        sim.step();  % Chama sim.step com os parâmetros indicados.
        [imageData, resolution] = sim.getVisionSensorImg(cameraHandle);  % Lê a imagem do sensor.
        image = formatCameraImage(imageData, resolution);  % Atualiza image.
        filename = fullfile(outputFolder, sprintf('calib_%02d.png', k));  % Formata o texto.
        imwrite(image, filename);  % Chama imwrite com os parâmetros indicados.
        fprintf('%02d/%02d guardada: %s\n', k, numberOfImages, filename);  % Apresenta a medição ou o estado na consola.
        imshow(image);  % Chama imshow com os parâmetros indicados.
        title(sprintf('Calibracao: imagem %d/%d', k, numberOfImages));  % Formata o texto.
        drawnow;  % Especifica drawnow.
    end  % Fim do bloco.
    clear cleanup  % Especifica clear.
    clear sim client  % Especifica clear.
    disp('Capturas concluidas. Executa calibrationPX100(''calibrate'').');  % Chama disp com os parâmetros indicados.
end  % Fim do bloco.


function [cameraParams, estimationErrors] = calibrateIntrinsics(imageFolder, squareSize)  % Define as entradas e saídas desta função.
    [calibrationFolder, ~] = projectFolders();  % Atualiza [calibrationFolder, ~].
    if nargin < 1 || isempty(imageFolder), imageFolder = fullfile(calibrationFolder, 'images'); end  % Verifica se o argumento opcional foi fornecido.
    if nargin < 2 || isempty(squareSize), squareSize = 0.0625; end  % Verifica se o argumento opcional foi fornecido.
    if ~isfolder(imageFolder), error('Pasta nao encontrada: %s', imageFolder); end  % Testa a condição sobre isfolder, imageFolder.

    extensions = {'*.png','*.jpg','*.jpeg','*.tif','*.bmp'};  % Atualiza extensions.
    files = struct([]);  % Reúne os campos numa estrutura.
    for k = 1:numel(extensions)  % Percorre os valores do índice k.
        files = [files; dir(fullfile(imageFolder, extensions{k}))]; %#ok<AGROW>
    end  % Fim do bloco.
    if isempty(files), error('Nao foram encontradas imagens em %s.', imageFolder); end  % Testa a condição sobre isempty, files.
    names = fullfile({files.folder}, {files.name});  % Constrói o caminho do ficheiro.
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(names);  % Atualiza [imagePoints, boardSize, imagesUsed].
    if nnz(imagesUsed) < 3  % Testa a condição sobre nnz, imagesUsed.
        error('O MATLAB detetou o tabuleiro em menos de 3 imagens.');  % Interrompe a execução e indica a causa.
    elseif nnz(imagesUsed) < 10  % Testa a condição sobre nnz, imagesUsed.
        warning('Foram usadas apenas %d imagens; recomenda-se 10 a 20.', nnz(imagesUsed));  % Chama warning com os parâmetros indicados.
    end  % Fim do bloco.

    firstImage = imread(names{1});  % Atualiza firstImage.
    imageSize = [size(firstImage,1), size(firstImage,2)];  % Atualiza imageSize.
    squareSizeMillimeters = squareSize * 1000;  % Atualiza squareSizeMillimeters.
    worldPoints = generateCheckerboardPoints(boardSize, squareSizeMillimeters);  % Atualiza worldPoints.
    [cameraParams, ~, estimationErrors] = estimateCameraParameters( ...  % Atualiza [cameraParams, ~, estimationErrors].
        imagePoints, worldPoints, 'ImageSize', imageSize, 'WorldUnits', 'mm');  % Completa os dados da instrução anterior.
    usedFiles = names(imagesUsed);  % Atualiza usedFiles.
    calibrationFile = fullfile(calibrationFolder, 'cameraCalibration.mat');  % Constrói o caminho do ficheiro.
    save(calibrationFile, 'cameraParams', 'estimationErrors', 'imagePoints', ...  % Chama save com os parâmetros indicados.
        'worldPoints', 'boardSize', 'imagesUsed', 'usedFiles', ...  % Completa os dados da instrução anterior.
        'squareSize', 'squareSizeMillimeters', 'imageSize');  % Completa os dados da instrução anterior.
    fprintf('Calibracao guardada em %s\n', calibrationFile);  % Apresenta a medição ou o estado na consola.
    fprintf('Imagens usadas: %d de %d\n', nnz(imagesUsed), numel(names));  % Apresenta a medição ou o estado na consola.
    fprintf('Erro medio de reprojecao: %.4f px\n', ...  % Apresenta a medição ou o estado na consola.
        mean(cameraParams.MeanReprojectionError));  % Completa os dados da instrução anterior.
    figure('Name','Erros da calibracao'); showReprojectionErrors(cameraParams);  % Chama figure com os parâmetros indicados.
    figure('Name','Extrinsecos por imagem'); showExtrinsics(cameraParams,'CameraCentric');  % Chama figure com os parâmetros indicados.
end  % Fim do bloco.


function T_base_camera = getCameraTransform(basePath, cameraPath)  % Define as entradas e saídas desta função.
    [calibrationFolder, projectFolder] = projectFolders();  % Atualiza [calibrationFolder, projectFolder].
    if nargin < 1 || isempty(basePath), basePath = '/px100'; end  % Verifica se o argumento opcional foi fornecido.
    if nargin < 2 || isempty(cameraPath), cameraPath = '/Vision_sensor'; end  % Verifica se o argumento opcional foi fornecido.
    calibrationFile = fullfile(calibrationFolder, 'cameraCalibration.mat');  % Constrói o caminho do ficheiro.
    if ~isfile(calibrationFile), error('Executa primeiro o comando calibrate.'); end  % Testa a condição sobre isfile, calibrationFile.
    [client, sim] = connectRemote(projectFolder);  % Atualiza [client, sim].
    baseHandle = sim.getObject(basePath);  % Obtém a referência do objeto da cena.
    cameraHandle = sim.getObject(cameraPath);  % Obtém a referência do objeto da cena.
    T_base_camera = toHomogeneous(sim.getObjectMatrix(cameraHandle, baseHandle));  % Lê a transformação do objeto.
    cameraPositionInBase = T_base_camera(1:3,4)';  % Atualiza cameraPositionInBase.
    save(calibrationFile, 'T_base_camera', 'cameraPositionInBase', ...  % Chama save com os parâmetros indicados.
        'basePath', 'cameraPath', '-append');  % Completa os dados da instrução anterior.
    fprintf('Transformacao camera -> base guardada em %s\n', calibrationFile);  % Apresenta a medição ou o estado na consola.
    fprintf('Posicao [x y z] (m): [%.6f %.6f %.6f]\n', cameraPositionInBase);  % Apresenta a medição ou o estado na consola.
    disp(T_base_camera);  % Chama disp com os parâmetros indicados.
    clear sim client  % Especifica clear.
end  % Fim do bloco.


function validation = validateCalibration(image, simInterface)  % Define as entradas e saídas desta função.
    [calibrationFolder, ~] = projectFolders();  % Atualiza [calibrationFolder, ~].
    calibrationFile = fullfile(calibrationFolder, 'cameraCalibration.mat');  % Constrói o caminho do ficheiro.
    data = load(calibrationFile);  % Atualiza data.
    [imagePoints, boardSize] = detectCheckerboardPoints(image);  % Atualiza [imagePoints, boardSize].
    if isempty(imagePoints)  % Testa a condição sobre isempty, imagePoints.
        validation = struct('valid',false,'errorPixels',NaN);  % Reúne os campos numa estrutura.
        disp('Tabuleiro nao detetado.'); return  % Chama disp com os parâmetros indicados.
    end  % Fim do bloco.
    if numel(boardSize) ~= 2 || any(boardSize < 4) || ...  % Testa a condição sobre numel, boardSize, any.
            size(imagePoints,1) ~= prod(boardSize-1)  % Atualiza size(imagePoints,1) ~.
        error('Dimensoes inesperadas: [%d %d], pontos=%d.', ...  % Atualiza error('Dimensoes inesperadas: [%d %d], pontos.
            boardSize(1),boardSize(2),size(imagePoints,1));  % Completa os dados da instrução anterior.
    end  % Fim do bloco.

    client = simInterface.get_connection();  % Atualiza client.
    sim = client.require('sim');  % Atualiza sim.
    baseHandle = sim.getObject('/px100');  % Obtém a referência do objeto da cena.
    chessHandle = sim.getObject('/chess');  % Obtém a referência do objeto da cena.
    cameraHandle = sim.getObject('/Vision_sensor');  % Obtém a referência do objeto da cena.
    currentCameraMatrix = toHomogeneous(sim.getObjectMatrix(cameraHandle,baseHandle));  % Lê a transformação do objeto.
    difference = max(abs(currentCameraMatrix(:)-double(data.T_base_camera(:))));  % Mede o erro angular pelo caminho mais curto.
    if difference > 1e-4  % Testa a condição sobre difference.
        fprintf('Pose da camera mudou (%.6g); atualizo T_base_camera.\n',difference);  % Apresenta a medição ou o estado na consola.
        T_base_camera = currentCameraMatrix;  % Atualiza T_base_camera.
        cameraPositionInBase = currentCameraMatrix(1:3,4)';  % Atualiza cameraPositionInBase.
        save(calibrationFile,'T_base_camera','cameraPositionInBase','-append');  % Chama save com os parâmetros indicados.
    end  % Fim do bloco.
    T_base_chess = toHomogeneous(sim.getObjectMatrix(chessHandle,baseHandle));  % Lê a transformação do objeto.

    boardSide = 0.50;  % Define boardSide.
    squareX = boardSide / boardSize(2);  % Atualiza squareX.
    squareY = boardSide / boardSize(1);  % Atualiza squareY.
    innerCols = boardSize(2)-1; innerRows = boardSize(1)-1;  % Atualiza innerCols.
    x = ((1:innerCols)-(innerCols+1)/2)*squareX;  % Atualiza x.
    y = ((1:innerRows)-(innerRows+1)/2)*squareY;  % Atualiza y.
    [X,Y] = meshgrid(x,y);  % Atualiza [X,Y].
    localPoints = [X(:),Y(:),zeros(numel(X),1),ones(numel(X),1)]';  % Conta os elementos.
    basePoints = (T_base_chess*localPoints)'; basePoints = basePoints(:,1:3);  % Atualiza basePoints.

    T_camera_base = inv(double(currentCameraMatrix));  % Atualiza T_camera_base.
    axisMaps = cat(3,[1 0;0 1],[1 0;0 -1],[-1 0;0 1],[-1 0;0 -1], ...  % Atualiza axisMaps.
        [0 1;1 0],[0 1;-1 0],[0 -1;1 0],[0 -1;-1 0]);  % Completa os dados da instrução anterior.
    bestError = inf; projected = []; bestAxisMap = [];  % Atualiza bestError.
    for k = 1:size(axisMaps,3)  % Percorre os valores do índice k.
        map3 = zeros(3); map3(1:2,1:2) = axisMaps(:,:,k);  % Reserva memória inicializada a zero.
        map3(3,3) = det(axisMaps(:,:,k));  % Atualiza map3(3,3).
        T_pinhole_base = [map3,zeros(3,1);0 0 0 1]*T_camera_base;  % Reserva memória inicializada a zero.
        R = T_pinhole_base(1:3,1:3);  % Mede a distância do ombro ao pulso.
        [U,~,V] = svd(R); R = U*diag([1 1 det(U*V')])*V';  % Atualiza [U,~,V].
        pointsCamera = (R*basePoints'+T_pinhole_base(1:3,4))';  % Atualiza pointsCamera.
        if any(pointsCamera(:,3)<=0), continue; end  % Testa a condição sobre any, pointsCamera, continue, end.
        candidate = worldToImage(data.cameraParams,R',T_pinhole_base(1:3,4)', ...  % Atualiza candidate.
            basePoints,'ApplyDistortion',true);  % Completa os dados da instrução anterior.
        if any(~isfinite(candidate),'all'), continue; end  % Testa a condição sobre any, isfinite, candidate, all, continue, end.
        d = hypot(candidate(:,1)-imagePoints(:,1)', ...  % Calcula a distância no plano.
            candidate(:,2)-imagePoints(:,2)');  % Completa os dados da instrução anterior.
        err = sqrt(mean([min(d,[],1)'; min(d,[],2)].^2));  % Atualiza err.
        if err < bestError  % Testa a condição sobre err, bestError.
            bestError = err; projected = candidate; bestAxisMap = axisMaps(:,:,k);  % Atualiza bestError.
        end  % Fim do bloco.
    end  % Fim do bloco.
    if isempty(projected), error('Nao foi possivel projetar os cantos do tabuleiro.'); end  % Testa a condição sobre isempty, projected.

    cameraToPinhole = eye(4);  % Atualiza cameraToPinhole.
    cameraToPinhole(1:2,1:2) = bestAxisMap;  % Atualiza cameraToPinhole(1:2,1:2).
    cameraToPinhole(3,3) = det(bestAxisMap);  % Atualiza cameraToPinhole(3,3).
    save(calibrationFile,'cameraToPinhole','bestAxisMap','-append');  % Chama save com os parâmetros indicados.
    validation = struct('valid',true,'pass',bestError<2, ...  % Reúne os campos numa estrutura.
        'errorPixels',bestError,'boardSize',boardSize,'axisMap',bestAxisMap, ...  % Completa os dados da instrução anterior.
        'detectedPoints',imagePoints,'projectedPoints',projected);  % Completa os dados da instrução anterior.
    fprintf('Tabuleiro: %d x %d quadrados (%d x %d cantos internos)\n', ...  % Apresenta a medição ou o estado na consola.
        boardSize(1),boardSize(2),innerRows,innerCols);  % Completa os dados da instrução anterior.
    fprintf('Espacamento inferido [x y]: [%.5f %.5f] m\n',squareX,squareY);  % Apresenta a medição ou o estado na consola.
    fprintf('Erro medio de validacao: %.3f px\n',bestError);  % Apresenta a medição ou o estado na consola.
    disp('Conversao de eixos do sensor para imagem:'); disp(bestAxisMap);  % Chama disp com os parâmetros indicados.
    if validation.pass, disp('Validacao passou (<2 px).');  % Testa a condição sobre validation.pass, disp, Validacao, passou, px.
    else, disp('Validacao acima de 2 px.'); end  % Chama else com os parâmetros indicados.
    figure('Name','Validacao camera-base com tabuleiro'); imshow(image); hold on  % Chama figure com os parâmetros indicados.
    plot(imagePoints(:,1),imagePoints(:,2),'go','MarkerSize',7,'LineWidth',1.5);  % Chama plot com os parâmetros indicados.
    plot(projected(:,1),projected(:,2),'r+','MarkerSize',8,'LineWidth',1.5);  % Chama plot com os parâmetros indicados.
    legend('Cantos detetados','Cantos projetados');  % Chama legend com os parâmetros indicados.
    title(sprintf('Erro medio: %.3f px',bestError)); hold off  % Formata o texto.
end  % Fim do bloco.


function objects = inspectScene(simInterface)  % Define as entradas e saídas desta função.
    [~,projectFolder] = projectFolders();  % Atualiza [~,projectFolder].
    if nargin < 1 || isempty(simInterface)  % Verifica se o argumento opcional foi fornecido.
        [client,sim] = connectRemote(projectFolder);  % Atualiza [client,sim].
    else  % Trata o caso alternativo.
        client = simInterface.get_connection(); sim = client.require('sim');  % Atualiza client.
    end  % Fim do bloco.
    baseHandle = sim.getObject('/px100');  % Obtém a referência do objeto da cena.
    handles = sim.getObjectsInTree(sim.handle_scene,sim.handle_all,0);  % Obtém a referência do objeto da cena.
    if iscell(handles), handles = cell2mat(handles); end  % Testa a condição sobre iscell, handles.
    objects = repmat(struct('path','','positionBase',[NaN NaN NaN], ...  % Reúne os campos numa estrutura.
        'orientationBase',[NaN NaN NaN],'bboxMin',[NaN NaN NaN], ...  % Completa os dados da instrução anterior.
        'bboxMax',[NaN NaN NaN]),numel(handles),1);  % Completa os dados da instrução anterior.
    fprintf('%-48s %27s %27s %27s\n','Objeto','Origem [x y z] base (m)', ...  % Apresenta a medição ou o estado na consola.
        'BBox min local (m)','BBox max local (m)');  % Completa os dados da instrução anterior.
    for k = 1:numel(handles)  % Percorre os valores do índice k.
        h = handles(k); path = sim.getObjectAlias(h,1);  % Obtém a referência do objeto da cena.
        pos = asRow(sim.getObjectPosition(h,baseHandle));  % Lê a posição do objeto no referencial pedido.
        ori = asRow(sim.getObjectOrientation(h,baseHandle));  % Obtém a referência do objeto da cena.
        mn = [NaN NaN NaN]; mx = mn;  % Atualiza mn.
        try  % Tenta executar a operação seguinte.
            mn = [getFloat(sim,h,sim.objfloatparam_objbbox_min_x), ...  % Atualiza mn.
                getFloat(sim,h,sim.objfloatparam_objbbox_min_y), ...  % Completa os dados da instrução anterior.
                getFloat(sim,h,sim.objfloatparam_objbbox_min_z)];  % Completa os dados da instrução anterior.
            mx = [getFloat(sim,h,sim.objfloatparam_objbbox_max_x), ...  % Atualiza mx.
                getFloat(sim,h,sim.objfloatparam_objbbox_max_y), ...  % Completa os dados da instrução anterior.
                getFloat(sim,h,sim.objfloatparam_objbbox_max_z)];  % Completa os dados da instrução anterior.
        catch  % Trata a falha da operação anterior.
        end  % Fim do bloco.
        if iscell(path), path = path{1}; end  % Testa a condição sobre iscell, path.
        objects(k) = struct('path',char(path),'positionBase',double(pos), ...  % Reúne os campos numa estrutura.
            'orientationBase',double(ori),'bboxMin',double(mn),'bboxMax',double(mx));  % Completa os dados da instrução anterior.
        fprintf('%-48s [%8.4f %8.4f %8.4f] [%8.4f %8.4f %8.4f] [%8.4f %8.4f %8.4f]\n', ...  % Apresenta a medição ou o estado na consola.
            objects(k).path,objects(k).positionBase,objects(k).bboxMin,objects(k).bboxMax);  % Completa os dados da instrução anterior.
    end  % Fim do bloco.
    clear sim client  % Especifica clear.
end  % Fim do bloco.


function pointsBase = projectPixelsToObjectTop(pixelPoints,objectPath,simInterface)  % Define as entradas e saídas desta função.
    if size(pixelPoints,2) ~= 2 || isempty(pixelPoints)  % Testa a condição sobre size, pixelPoints, isempty.
        error('pixelPoints deve ser uma matriz N x 2 [x y].');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    [calibrationFolder,~] = projectFolders();  % Atualiza [calibrationFolder,~].
    calibrationFile = fullfile(calibrationFolder,'cameraCalibration.mat');  % Constrói o caminho do ficheiro.
    data = load(calibrationFile);  % Atualiza data.
    required = {'cameraParams','T_base_camera','cameraToPinhole'};  % Atualiza required.
    if ~all(isfield(data,required))  % Testa a condição sobre all, isfield, data, required.
        error('Executa primeiro os comandos calibrate, transform e validate.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    client = simInterface.get_connection(); sim = client.require('sim');  % Atualiza client.
    baseHandle = sim.getObject('/px100');  % Obtém a referência do objeto da cena.
    cameraHandle = sim.getObject('/Vision_sensor');  % Obtém a referência do objeto da cena.
    objectHandle = sim.getObject(objectPath);  % Obtém a referência do objeto da cena.
    T_base_camera = toHomogeneous(sim.getObjectMatrix(cameraHandle,baseHandle));  % Lê a transformação do objeto.
    T_base_object = toHomogeneous(sim.getObjectMatrix(objectHandle,baseHandle));  % Lê a transformação do objeto.
    cameraPositionInBase = T_base_camera(1:3,4)';  % Atualiza cameraPositionInBase.
    save(calibrationFile,'T_base_camera','cameraPositionInBase','-append');  % Chama save com os parâmetros indicados.

    maxZ = getFloat(sim,objectHandle,sim.objfloatparam_objbbox_max_z);  % Atualiza maxZ.
    R_object = T_base_object(1:3,1:3);  % Atualiza R_object.
    planePoint = T_base_object(1:3,4)+R_object(:,3)*maxZ;  % Atualiza planePoint.
    planeNormal = R_object(:,3);  % Atualiza planeNormal.
    T_base_pinhole = double(T_base_camera)/double(data.cameraToPinhole);  % Atualiza T_base_pinhole.
    cameraOrigin = T_base_pinhole(1:3,4);  % Extrai a posição da câmara no referencial da base.
    cameraRotation = T_base_pinhole(1:3,1:3);  % Extrai a rotação que leva direções da câmara à base.

    uv = undistortPoints(double(pixelPoints),data.cameraParams);  % Atualiza uv.
    intrinsics = data.cameraParams.Intrinsics;  % Atualiza intrinsics.
    focal = double(intrinsics.FocalLength);  % Lê as distâncias focais horizontal e vertical.
    principal = double(intrinsics.PrincipalPoint);  % Lê a posição do ponto principal na imagem.
    rays = [(uv(:,1)-principal(1))/focal(1), ...  % Normaliza os píxeis para construir raios na câmara.
        (uv(:,2)-principal(2))/focal(2),ones(size(uv,1),1)];  % Completa os dados da instrução anterior.
    pointsBase = zeros(size(pixelPoints,1),3);  % Reserva memória inicializada a zero.
    for k = 1:size(pixelPoints,1)  % Percorre os valores do índice k.
        direction = cameraRotation*rays(k,:)';  % Atualiza direction.
        denominator = dot(planeNormal,direction);  % Atualiza denominator.
        if abs(denominator)<1e-10, error('Raio paralelo ao plano superior de %s.',objectPath); end  % Testa a condição sobre abs, denominator.
        distance = dot(planeNormal,planePoint-cameraOrigin)/denominator;  % Atualiza distance.
        if distance<=0, error('Plano de %s atras da camera.',objectPath); end  % Testa a condição sobre distance.
        pointsBase(k,:) = (cameraOrigin+distance*direction)';  % Atualiza pointsBase(k,:).
    end  % Fim do bloco.
    fprintf('XYZ no referencial /px100 (m), plano maxZ de %s:\n',objectPath);  % Apresenta a medição ou o estado na consola.
    disp(pointsBase);  % Chama disp com os parâmetros indicados.
end  % Fim do bloco.


function originPixel = projectBaseOrigin(simInterface)  % Define as entradas e saídas desta função.
    [calibrationFolder,~] = projectFolders();  % Atualiza [calibrationFolder,~].
    data = load(fullfile(calibrationFolder,'cameraCalibration.mat'));  % Constrói o caminho do ficheiro.
    required = {'cameraParams','cameraToPinhole'};  % Atualiza required.
    if ~all(isfield(data,required))  % Testa a condição sobre all, isfield, data, required.
        error('Executa primeiro os comandos calibrate e validate.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    client = simInterface.get_connection(); sim = client.require('sim');  % Atualiza client.
    baseHandle = sim.getObject('/px100');  % Obtém a referência do objeto da cena.
    cameraHandle = sim.getObject('/Vision_sensor');  % Obtém a referência do objeto da cena.
    T_base_camera = toHomogeneous(sim.getObjectMatrix(cameraHandle,baseHandle));  % Lê a transformação do objeto.
    T_base_pinhole = double(T_base_camera)/double(data.cameraToPinhole);  % Atualiza T_base_pinhole.
    T_pinhole_base = inv(T_base_pinhole);  % Atualiza T_pinhole_base.
    R = T_pinhole_base(1:3,1:3);  % Mede a distância do ombro ao pulso.
    [U,~,V] = svd(R); R = U*diag([1 1 det(U*V')])*V';  % Atualiza [U,~,V].
    t = T_pinhole_base(1:3,4)';  % Atualiza t.
    originCamera = R*[0;0;0]+t';  % Atualiza originCamera.
    if originCamera(3) <= 0  % Testa a condição sobre originCamera.
        error('A origem da base esta atras da camera e nao pode ser projetada.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    originPixel = worldToImage(data.cameraParams,R',t,[0 0 0], ...  % Atualiza originPixel.
        'ApplyDistortion',true);  % Completa os dados da instrução anterior.
end  % Fim do bloco.


function [client,sim] = connectRemote(projectFolder)  % Define as entradas e saídas desta função.
    addpath(projectFolder);  % Chama addpath com os parâmetros indicados.
    jarPath = fullfile(projectFolder,'jeromq-0.5.2.jar');  % Constrói o caminho do ficheiro.
    if ~isfile(jarPath), error('Biblioteca ZeroMQ nao encontrada: %s',jarPath); end  % Testa a condição sobre isfile, jarPath.
    try  % Tenta executar a operação seguinte.
        java.lang.Class.forName('org.zeromq.ZMsg');  % Chama java.lang.Class.forName com os parâmetros indicados.
    catch  % Trata a falha da operação anterior.
        javaaddpath(jarPath);  % Chama javaaddpath com os parâmetros indicados.
    end  % Fim do bloco.
    client = RemoteAPIClient();  % Atualiza client.
    sim = client.require('sim');  % Atualiza sim.
end  % Fim do bloco.


function [calibrationFolder,projectFolder] = projectFolders()  % Define as entradas e saídas desta função.
    calibrationFolder = fileparts(mfilename('fullpath'));  % Atualiza calibrationFolder.
    projectFolder = fileparts(calibrationFolder);  % Atualiza projectFolder.
end  % Fim do bloco.


function restoreScene(sim,chessHandle,position,orientation)  % Define as entradas e saídas desta função.
    try  % Tenta executar a operação seguinte.
        sim.setObjectPosition(chessHandle,position,sim.handle_world);  % Chama sim.setObjectPosition com os parâmetros indicados.
        sim.setObjectOrientation(chessHandle,orientation,sim.handle_world);  % Chama sim.setObjectOrientation com os parâmetros indicados.
        sim.stopSimulation();  % Chama sim.stopSimulation com os parâmetros indicados.
    catch  % Trata a falha da operação anterior.
    end  % Fim do bloco.
end  % Fim do bloco.


function image = formatCameraImage(imageData,resolution)  % Define as entradas e saídas desta função.
    width = resolution{1}; height = resolution{2};  % Atualiza width.
    image = flip(permute(reshape(imageData,3,width,height),[3 2 1]),1);  % Reorganiza as dimensões dos dados.
    image = normalizeImage(image);  % Atualiza image.
end  % Fim do bloco.


function image = normalizeImage(image)  % Define as entradas e saídas desta função.
    if isa(image,'uint8'), return; end  % Testa a condição sobre isa, image, uint8.
    image = double(image);  % Atualiza image.
    if ~isempty(image) && max(image(:))<=1, image = image*255; end  % Testa a condição sobre isempty, image, max.
    image = uint8(min(max(round(image),0),255));  % Atualiza image.
end  % Fim do bloco.


function T = toHomogeneous(matrixData)  % Define as entradas e saídas desta função.
    if iscell(matrixData), matrixData = cell2mat(matrixData); end  % Testa a condição sobre iscell, matrixData.
    T = [reshape(double(matrixData),4,3)';0 0 0 1];  % Reorganiza as dimensões dos dados.
end  % Fim do bloco.


function row = asRow(value)  % Define as entradas e saídas desta função.
    if iscell(value), value = cell2mat(value); end  % Testa a condição sobre iscell, value.
    row = double(value(:)');  % Atualiza row.
end  % Fim do bloco.


function value = getFloat(sim,handle,parameter)  % Define as entradas e saídas desta função.
    value = sim.getObjectFloatParam(handle,parameter);  % Lê um parâmetro geométrico da cena.
    if iscell(value), value = cell2mat(value); end  % Testa a condição sobre iscell, value.
    value = double(value);  % Atualiza value.
end  % Fim do bloco.
