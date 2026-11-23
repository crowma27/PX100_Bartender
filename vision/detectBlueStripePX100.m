function stripe = detectBlueStripePX100(image)  % Define as entradas e saídas desta função.
% Segmenta a faixa azul e estima os dois cantos da aresta inferior.
% Devolve coordenadas de pixeis [x y], origem no canto superior esquerdo.

    hsvImage = rgb2hsv(image);  % Atualiza hsvImage.
    hue = hsvImage(:,:,1);  % Atualiza hue.
    saturation = hsvImage(:,:,2);  % Atualiza saturation.
    value = hsvImage(:,:,3);  % Atualiza value.
    mask = hue>=0.52 & hue<=0.72 & saturation>=0.35 & value>=0.20;  % Atualiza mask.
    mask = bwareaopen(mask,10);  % Elimina regiões com poucos píxeis.
    mask = imclose(mask,strel('rectangle',[3 15]));  % Une regiões separadas por pequenas falhas.
    mask = bwareafilt(mask,1); % Assume que a faixa e a maior regiao azul.
    if ~any(mask,'all')  % Testa a condição sobre any, mask, all.
        stripe = [];  % Atualiza stripe.
        return  % Regressa à função que fez a chamada.
    end  % Fim do bloco.

    [y,x] = find(mask);  % Atualiza [y,x].
    xy = [x,y];  % Atualiza xy.
    % O eixo de maior variacao dos pixeis segue o comprimento da faixa.
    centered = xy-mean(xy,1);  % Atualiza centered.
    [vectors,~] = eig(cov(centered));  % Atualiza [vectors,~].
    [~,order] = sort(var(centered*vectors,0,1),'descend');  % Atualiza [~,order].
    longAxis = vectors(:,order(1));  % Atualiza longAxis.
    shortAxis = [-longAxis(2);longAxis(1)];  % Atualiza shortAxis.
    axesMatrix = [longAxis,shortAxis];  % Atualiza axesMatrix.
    % Mede o retangulo nos eixos da faixa para acompanhar a sua inclinacao.
    projected = centered*axesMatrix;  % Atualiza projected.
    lower = min(projected,[],1);  % Atualiza lower.
    upper = max(projected,[],1);  % Atualiza upper.
    localCorners = [lower(1),lower(2); upper(1),lower(2); ...  % Atualiza localCorners.
        upper(1),upper(2); lower(1),upper(2)];  % Completa os dados da instrução anterior.
    imageCorners = localCorners*axesMatrix'+mean(xy,1);  % Atualiza imageCorners.
    edgeA = imageCorners(1:2,:);  % Atualiza edgeA.
    edgeB = imageCorners(3:4,:);  % Atualiza edgeB.
    % Na imagem, o maior Y corresponde a aresta que aparece mais abaixo.
    if mean(edgeA(:,2)) > mean(edgeB(:,2))  % Testa a condição sobre mean, edgeA, edgeB.
        corners = edgeA;  % Atualiza corners.
    else  % Trata o caso alternativo.
        corners = edgeB;  % Atualiza corners.
    end  % Fim do bloco.
    [~,leftToRight] = sort(corners(:,1));  % Atualiza [~,leftToRight].
    stripe.mask = mask;  % Atualiza stripe.mask.
    stripe.lowerCorners = corners(leftToRight,:);  % Atualiza stripe.lowerCorners.
end  % Fim do bloco.
