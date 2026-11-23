function refresh = openCameraViewPX100(simInterface, firstImage)  % Define as entradas e saídas desta função.
% Abre a imagem da camara e devolve a funcao que atualiza a janela.
    window = figure('Name', 'Camara do PX100', 'NumberTitle', 'off');  % Atualiza window.
    viewAxes = axes('Parent', window);  % Atualiza viewAxes.
    frame = imshow(firstImage, 'Parent', viewAxes);  % Atualiza frame.
    phase = 'Copo detetado';  % Atualiza phase.
    title(viewAxes, phase);  % Chama title com os parâmetros indicados.
    lastFrame = tic;  % Atualiza lastFrame.
    drawnow;  % Especifica drawnow.
    refresh = @update;  % Guarda a função a chamar mais tarde.

    function update(nextPhase)  % Define as entradas e saídas desta função.
        if ~isgraphics(frame)  % Testa a condição sobre isgraphics, frame.
            return;  % Regressa à função que fez a chamada.
        end  % Fim do bloco.

        % Limita as capturas a dez por segundo. Uma nova fase aparece logo.
        phaseChanged = ~strcmp(nextPhase, phase);  % Atualiza phaseChanged.
        if ~phaseChanged && toc(lastFrame) < 0.1  % Testa a condição sobre phaseChanged, toc, lastFrame.
            return;  % Regressa à função que fez a chamada.
        end  % Fim do bloco.

        [cameraImage, ~] = simInterface.get_camera_information();  % Captura uma imagem da câmara.
        frame.CData = uint8(cameraImage);  % Atualiza frame.CData.
        if phaseChanged  % Testa a condição sobre phaseChanged.
            phase = nextPhase;  % Atualiza phase.
            title(viewAxes, phase);  % Chama title com os parâmetros indicados.
        end  % Fim do bloco.
        lastFrame = tic;  % Atualiza lastFrame.
        drawnow limitrate;  % Especifica drawnow.
    end  % Fim do bloco.
end  % Fim do bloco.
