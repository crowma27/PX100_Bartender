function visionInfo = runVisionPX100(simInterface, markerPath)  % Define as entradas e saídas desta função.
% Captura uma imagem, deteta a faixa azul e calcula os XYZ dos seus cantos.
% A simulacao deve estar ativa e a interface deve ter visao habilitada.

    if nargin < 2 || isempty(markerPath), markerPath = '/dummy/marcador'; end  % Verifica se o argumento opcional foi fornecido.
    [image,~] = simInterface.get_camera_information();  % Captura uma imagem da câmara.
    image = uint8(image);  % Atualiza image.
    stripe = detectBlueStripePX100(image);  % Atualiza stripe.
    visionInfo = struct('image',image,'stripe',stripe,'cornersXYZ',[]);  % Reúne os campos numa estrutura.

    figure('Name','Visao e faixa azul');  % Chama figure com os parâmetros indicados.
    imshow(image);  % Chama imshow com os parâmetros indicados.
    hold on  % Especifica hold.
    try  % Tenta executar a operação seguinte.
        originPixel = calibrationPX100('originpixel',simInterface);  % Atualiza originPixel.
        if originPixel(1)>=1 && originPixel(1)<=size(image,2) && ...  % Testa a condição sobre originPixel, size, image.
                originPixel(2)>=1 && originPixel(2)<=size(image,1)  % Atualiza originPixel(2)>.
            plot(originPixel(1),originPixel(2),'y*','MarkerSize',16,'LineWidth',2);  % Chama plot com os parâmetros indicados.
            text(originPixel(1)+8,originPixel(2),'Base (0,0,0)', ...  % Chama text com os parâmetros indicados.
                'Color','y','FontWeight','bold');  % Completa os dados da instrução anterior.
            fprintf('Origem da base projetada: [%.1f %.1f] px\n',originPixel);  % Apresenta a medição ou o estado na consola.
        else  % Trata o caso alternativo.
            fprintf('Origem da base fora da imagem: [%.1f %.1f] px\n',originPixel);  % Apresenta a medição ou o estado na consola.
        end  % Fim do bloco.
    catch ME  % Trata a falha da operação anterior.
        warning('Nao foi possivel projetar a origem da base: %s',ME.message);  % Chama warning com os parâmetros indicados.
    end  % Fim do bloco.

    if isempty(stripe)  % Testa a condição sobre isempty, stripe.
        title('Faixa azul nao detetada');  % Chama title com os parâmetros indicados.
        disp('Faixa azul nao detetada.');  % Chama disp com os parâmetros indicados.
    else  % Trata o caso alternativo.
        visboundaries(stripe.mask,'Color','y','LineWidth',1);  % Chama visboundaries com os parâmetros indicados.
        plot(stripe.lowerCorners(:,1),stripe.lowerCorners(:,2),'o', ...  % Chama plot com os parâmetros indicados.
            'MarkerSize',14,'MarkerFaceColor','r', ...  % Completa os dados da instrução anterior.
            'MarkerEdgeColor','y','LineWidth',2);  % Completa os dados da instrução anterior.
        text(stripe.lowerCorners(:,1)+5,stripe.lowerCorners(:,2), ...  % Chama text com os parâmetros indicados.
            {'Canto 1','Canto 2'},'Color','y','FontWeight','bold');  % Completa os dados da instrução anterior.
        title('Faixa azul e cantos inferiores');  % Chama title com os parâmetros indicados.
        fprintf('Canto 1 [x y] px: [%.1f %.1f]\n',stripe.lowerCorners(1,:));  % Apresenta a medição ou o estado na consola.
        fprintf('Canto 2 [x y] px: [%.1f %.1f]\n',stripe.lowerCorners(2,:));  % Apresenta a medição ou o estado na consola.
        try  % Tenta executar a operação seguinte.
            visionInfo.cornersXYZ = calibrationPX100('project', ...  % Atualiza visionInfo.cornersXYZ.
                stripe.lowerCorners,markerPath,simInterface);  % Completa os dados da instrução anterior.
        catch ME  % Trata a falha da operação anterior.
            warning('Nao foi possivel converter os cantos para XYZ: %s',ME.message);  % Chama warning com os parâmetros indicados.
        end  % Fim do bloco.
    end  % Fim do bloco.
    hold off  % Especifica hold.
    drawnow;  % Especifica drawnow.
    figure(gcf);  % Chama figure com os parâmetros indicados.
end  % Fim do bloco.
