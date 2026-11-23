function showCupVisionPX100(image, cups, debug, topZ)  % Define as entradas e saídas desta função.
% Mostra a detecao e o raio da camera para explicar a projecao.

    showVisionSteps(image, cups, debug, topZ);  % Chama showVisionSteps com os parâmetros indicados.
    if ~isempty(cups)  % Testa a condição sobre isempty, cups.
        showProjectionGeometry(cups, topZ);  % Chama showProjectionGeometry com os parâmetros indicados.
    end  % Fim do bloco.
end  % Fim do bloco.

function showVisionSteps(image,cups,debug,topZ)  % Define as entradas e saídas desta função.
    figure('Name','Detecao dos copos - passos da visao');  % Chama figure com os parâmetros indicados.
    tiledlayout(2,3,'TileSpacing','compact','Padding','compact');  % Chama tiledlayout com os parâmetros indicados.

    ax = nexttile;  % Atualiza ax.
    imshow(image,'Parent',ax); hold(ax,'on');  % Chama imshow com os parâmetros indicados.
    for k = 1:numel(cups)  % Percorre os valores do índice k.
        color = cupPlotColor(cups(k).color);  % Atualiza color.
        rectangle(ax,'Position',cups(k).boundingBox, ...  % Chama rectangle com os parâmetros indicados.
            'EdgeColor',color,'LineWidth',1.5);  % Completa os dados da instrução anterior.
        angle = linspace(0,2*pi,100);  % Atualiza angle.
        plot(ax,cups(k).centroid(1)+cups(k).radius*cos(angle), ...  % Chama plot com os parâmetros indicados.
            cups(k).centroid(2)+cups(k).radius*sin(angle), ...  % Completa os dados da instrução anterior.
            'k-','LineWidth',1.5);  % Completa os dados da instrução anterior.
        plot(ax,cups(k).centroid(1),cups(k).centroid(2),'y+', ...  % Chama plot com os parâmetros indicados.
            'MarkerSize',12,'LineWidth',2);  % Completa os dados da instrução anterior.
        text(ax,cups(k).centroid(1)+6,cups(k).centroid(2), ...  % Chama text com os parâmetros indicados.
            cups(k).color,'Color',color,'FontWeight','bold', ...  % Completa os dados da instrução anterior.
            'BackgroundColor','k','Margin',1);  % Completa os dados da instrução anterior.
    end  % Fim do bloco.
    title(ax,'1. Imagem e circulos aceites');  % Chama title com os parâmetros indicados.
    hold(ax,'off');  % Chama hold com os parâmetros indicados.

    ax = nexttile; imshow(debug.redMask,'Parent',ax); title(ax,'2. Mascara vermelha');  % Atualiza ax.
    ax = nexttile; imshow(debug.blueMask,'Parent',ax); title(ax,'3. Mascara azul');  % Atualiza ax.
    ax = nexttile; imshow(debug.greenMask,'Parent',ax); title(ax,'4. Mascara verde');  % Atualiza ax.

    ax = nexttile;  % Atualiza ax.
    imshow(debug.blackMask,'Parent',ax); hold(ax,'on');  % Chama imshow com os parâmetros indicados.
    for k = 1:numel(cups)  % Percorre os valores do índice k.
        plot(ax,cups(k).centroid(1),cups(k).centroid(2),'y+', ...  % Chama plot com os parâmetros indicados.
            'MarkerSize',12,'LineWidth',2);  % Completa os dados da instrução anterior.
    end  % Fim do bloco.
    title(ax,'5. Pixeis escuros e centros aceites');  % Chama title com os parâmetros indicados.
    hold(ax,'off');  % Chama hold com os parâmetros indicados.

    ax = nexttile; hold(ax,'on');  % Atualiza ax.
    plot(ax,0,0,'k+','MarkerSize',10,'LineWidth',1.5);  % Chama plot com os parâmetros indicados.
    for k = 1:numel(cups)  % Percorre os valores do índice k.
        color = cupPlotColor(cups(k).color);  % Atualiza color.
        xy = cups(k).topCenterBase(1:2);  % Atualiza xy.
        plot(ax,xy(1),xy(2),'o','Color',color, ...  % Chama plot com os parâmetros indicados.
            'MarkerFaceColor',color,'MarkerSize',8);  % Completa os dados da instrução anterior.
        text(ax,xy(1),xy(2),sprintf(' %s [%.3f, %.3f] m', ...  % Formata o texto.
            cups(k).color,xy(1),xy(2)),'Color',color, ...  % Completa os dados da instrução anterior.
            'VerticalAlignment','bottom');  % Completa os dados da instrução anterior.
    end  % Fim do bloco.
    axis(ax,'equal'); grid(ax,'on');  % Chama axis com os parâmetros indicados.
    xlabel(ax,'X na base /px100 (m)'); ylabel(ax,'Y na base /px100 (m)');  % Chama xlabel com os parâmetros indicados.
    title(ax,sprintf('6. Centros projetados no plano z = %.3f m',topZ));  % Atualiza title(ax,sprintf('6. Centros projetados no plano z.
    hold(ax,'off');  % Chama hold com os parâmetros indicados.

    sgtitle(sprintf(['Detecao passo a passo | pixel do rebordo preto -> XY ' ...  % Formata o texto.
        'no plano do topo (z = %.3f m)'],topZ));  % Completa os dados da instrução anterior.
end  % Fim do bloco.

function showProjectionGeometry(cups,topZ)  % Define as entradas e saídas desta função.
    calibrationFile = fullfile(fileparts(mfilename('fullpath')), ...  % Constrói o caminho do ficheiro.
        'cameraCalibration.mat');  % Completa os dados da instrução anterior.
    data = load(calibrationFile,'T_base_camera','cameraToPinhole');  % Atualiza data.
    T_base_pinhole = double(data.T_base_camera)/double(data.cameraToPinhole);  % Atualiza T_base_pinhole.
    cameraOrigin = T_base_pinhole(1:3,4);  % Extrai a posição da câmara no referencial da base.
    points = vertcat(cups.topCenterBase);  % Atualiza points.

    padding = 0.05;  % Define padding.
    xLimits = [min([cameraOrigin(1);points(:,1)])-padding, ...  % Atualiza xLimits.
        max([cameraOrigin(1);points(:,1)])+padding];  % Completa os dados da instrução anterior.
    yLimits = [min([cameraOrigin(2);points(:,2)])-padding, ...  % Atualiza yLimits.
        max([cameraOrigin(2);points(:,2)])+padding];  % Completa os dados da instrução anterior.
    if diff(xLimits) < 0.10, xLimits = xLimits+[-0.05 0.05]; end  % Testa a condição sobre diff, xLimits.
    if diff(yLimits) < 0.10, yLimits = yLimits+[-0.05 0.05]; end  % Testa a condição sobre diff, yLimits.

    figure('Name','Projecao geometrica: raio da camera e plano do copo');  % Chama figure com os parâmetros indicados.
    hold on;  % Especifica hold.
    patch([xLimits(1) xLimits(2) xLimits(2) xLimits(1)], ...  % Chama patch com os parâmetros indicados.
        [yLimits(1) yLimits(1) yLimits(2) yLimits(2)], ...  % Completa os dados da instrução anterior.
        topZ*ones(1,4),[0.65 0.82 1.0], ...  % Completa os dados da instrução anterior.
        'FaceAlpha',0.20,'EdgeColor',[0.3 0.5 0.8]);  % Completa os dados da instrução anterior.
    plot3(cameraOrigin(1),cameraOrigin(2),cameraOrigin(3), ...  % Chama plot3 com os parâmetros indicados.
        'ks','MarkerFaceColor','k','MarkerSize',8);  % Completa os dados da instrução anterior.
    text(cameraOrigin(1),cameraOrigin(2),cameraOrigin(3), ...  % Chama text com os parâmetros indicados.
        ' Camera/pinhole','VerticalAlignment','bottom');  % Completa os dados da instrução anterior.

    for k = 1:numel(cups)  % Percorre os valores do índice k.
        point = cups(k).topCenterBase;  % Atualiza point.
        color = cupPlotColor(cups(k).color);  % Atualiza color.
        plot3([cameraOrigin(1),point(1)], ...  % Chama plot3 com os parâmetros indicados.
            [cameraOrigin(2),point(2)], ...  % Completa os dados da instrução anterior.
            [cameraOrigin(3),point(3)],'--','Color',color,'LineWidth',1.5);  % Completa os dados da instrução anterior.
        plot3(point(1),point(2),point(3),'o', ...  % Chama plot3 com os parâmetros indicados.
            'Color',color,'MarkerFaceColor',color,'MarkerSize',7);  % Completa os dados da instrução anterior.
        text(point(1),point(2),point(3),[' ',cups(k).color]);  % Chama text com os parâmetros indicados.
    end  % Fim do bloco.
    xlabel('X /px100 (m)'); ylabel('Y /px100 (m)'); zlabel('Z /px100 (m)');  % Chama xlabel com os parâmetros indicados.
    title('Cada pixel define um raio; o plano conhecido fixa a profundidade');  % Chama title com os parâmetros indicados.
    axis equal; grid on; view(3); hold off;  % Chama axis com os parâmetros indicados.
end  % Fim do bloco.

function color = cupPlotColor(name)  % Define as entradas e saídas desta função.
    switch name  % Escolhe o ramo de acordo com name.
        case 'red'  % Executa o ramo 'red'.
            color = [1 0 0];  % Atualiza color.
        case 'blue'  % Executa o ramo 'blue'.
            color = [0 0.5 1];  % Atualiza color.
        case 'green'  % Executa o ramo 'green'.
            color = [0 1 0];  % Atualiza color.
        otherwise  % Trata o caso alternativo.
            color = [1 1 1];  % Atualiza color.
    end  % Fim do bloco.
end  % Fim do bloco.
