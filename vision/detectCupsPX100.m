function [cups,debug] = detectCupsPX100(image)  % Define as entradas e saídas desta função.
% Deteta copos vermelhos, azuis e verdes pelo corpo e pelo rebordo preto.
% centroid devolve o centro do circulo preto em pixeis [x y].

    if size(image,3) ~= 3  % Testa a condição sobre size, image.
        error('A imagem da camera tem de ser RGB.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.

    rgb = im2uint8(image);  % Atualiza rgb.
    hsvImage = rgb2hsv(rgb);  % Atualiza hsvImage.
    hue = hsvImage(:,:,1);  % Atualiza hue.
    saturation = hsvImage(:,:,2);  % Atualiza saturation.
    value = hsvImage(:,:,3);  % Atualiza value.
    gray = im2gray(rgb);  % Atualiza gray.
    blackMask = value < 0.24;  % Marca os píxeis escuros candidatos ao rebordo.
    debug = struct('redMask',false(size(value)), ...  % Reúne os campos numa estrutura.
        'blueMask',false(size(value)),'greenMask',false(size(value)), ...  % Completa os dados da instrução anterior.
        'blackMask',blackMask);  % Completa os dados da instrução anterior.

    % Aceita no maximo um copo por cor.
    colors = {'red','blue','green'};  % Atualiza colors.
    template = struct('color','','centroid',[],'bodyCentroid',[], ...  % Reúne os campos numa estrutura.
        'boundingBox',[],'area',0,'radius',0,'score',0, ...  % Completa os dados da instrução anterior.
        'topCenterBase',[],'graspTargetBase',[]);  % Completa os dados da instrução anterior.
    cups = repmat(template,0,1);  % Atualiza cups.

    for colorIndex = 1:numel(colors)  % Percorre os valores do índice colorIndex.
        switch colors{colorIndex}  % Escolhe o ramo de acordo com colors{colorIndex}.
            case 'red'  % Executa o ramo 'red'.
                % O vermelho aparece nas duas extremidades da escala de matiz.
                colorMask = (hue <= 0.055 | hue >= 0.94) & ...  % Atualiza colorMask.
                    saturation >= 0.30 & value >= 0.12;  % Atualiza saturation >.
            case 'blue'  % Executa o ramo 'blue'.
                colorMask = hue >= 0.50 & hue <= 0.75 & ...  % Atualiza colorMask.
                    saturation >= 0.30 & value >= 0.12;  % Atualiza saturation >.
            case 'green'  % Executa o ramo 'green'.
                colorMask = hue >= 0.18 & hue <= 0.46 & ...  % Atualiza colorMask.
                    saturation >= 0.30 & value >= 0.12;  % Atualiza saturation >.
        end  % Fim do bloco.

        % Remove ruido e une os pixeis do corpo antes de medir cada mancha.
        colorMask = imopen(colorMask,strel('disk',2,0));  % Remove pequenas manchas na máscara.
        colorMask = imclose(colorMask,strel('disk',4,0));  % Une regiões separadas por pequenas falhas.
        colorMask = imfill(colorMask,'holes');  % Preenche buracos na máscara.
        colorMask = bwareaopen(colorMask,40);  % Elimina regiões com poucos píxeis.
        debug.([colors{colorIndex},'Mask']) = colorMask;  % Atualiza debug.([colors{colorIndex},'Mask']).
        components = bwconncomp(colorMask);  % Agrupa píxeis vizinhos da máscara.
        stats = regionprops(components,'Area','Centroid','BoundingBox');  % Mede as regiões segmentadas.
        [~,componentOrder] = sort([stats.Area],'descend');  % Atualiza [~,componentOrder].

        % Comeca pela maior mancha desta cor e so aceita uma ROI com rebordo.
        for orderIndex = 1:numel(componentOrder)  % Percorre os valores do índice orderIndex.
            componentIndex = componentOrder(orderIndex);  % Atualiza componentIndex.
            box = double(stats(componentIndex).BoundingBox);  % Atualiza box.
            width = box(3);  % Atualiza width.
            height = box(4);  % Atualiza height.
            smallSide = min(width,height);  % Atualiza smallSide.
            aspect = width/max(height,eps);  % Atualiza aspect.
            if stats(componentIndex).Area < 40 || smallSide < 10 || ...  % Testa a condição sobre stats, componentIndex, Area, smallSide.
                    aspect < 0.40 || aspect > 2.5  % Completa os dados da instrução anterior.
                continue  % Passa à iteração seguinte.
            end  % Fim do bloco.

            [center,radius,score,found] = findBlackCircle( ...  % Atualiza [center,radius,score,found].
                gray,blackMask,box,double(stats(componentIndex).Centroid));  % Completa os dados da instrução anterior.
            if ~found  % Testa a condição sobre found.
                continue  % Passa à iteração seguinte.
            end  % Fim do bloco.

            cup = template;  % Atualiza cup.
            cup.color = colors{colorIndex};  % Atualiza cup.color.
            cup.centroid = center;  % Atualiza cup.centroid.
            cup.bodyCentroid = double(stats(componentIndex).Centroid);  % Atualiza cup.bodyCentroid.
            cup.boundingBox = box;  % Atualiza cup.boundingBox.
            cup.area = double(stats(componentIndex).Area);  % Atualiza cup.area.
            cup.radius = radius;  % Atualiza cup.radius.
            cup.score = score;  % Atualiza cup.score.
            cups(end+1,1) = cup; %#ok<AGROW>
            break  % Termina o ciclo atual.
        end  % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.

function [bestCenter,bestRadius,bestScore,found] = findBlackCircle( ...  % Define as entradas e saídas desta função.
    gray,blackMask,box,bodyCenter)  % Completa os dados da instrução anterior.

    imageHeight = size(gray,1);  % Atualiza imageHeight.
    imageWidth = size(gray,2);  % Atualiza imageWidth.
    % Procura o rebordo apenas perto da mancha de cor, com uma margem.
    pad = 0.30*max(box(3),box(4));  % Atualiza pad.
    x1 = max(1,floor(box(1)-pad));  % Atualiza x1.
    y1 = max(1,floor(box(2)-pad));  % Atualiza y1.
    x2 = min(imageWidth,ceil(box(1)+box(3)+pad));  % Atualiza x2.
    y2 = min(imageHeight,ceil(box(2)+box(4)+pad));  % Atualiza y2.
    grayCrop = gray(y1:y2,x1:x2);  % Atualiza grayCrop.
    blackCrop = blackMask(y1:y2,x1:x2);  % Atualiza blackCrop.
    % Converte o centro para as coordenadas locais do recorte.
    bodyCenterCrop = bodyCenter-[x1-1,y1-1];  % Expressa o centro do corpo nas coordenadas do recorte.

    smallSide = min(box(3),box(4));  % Atualiza smallSide.
    radiusMin = max(4,floor(0.22*smallSide));  % Atualiza radiusMin.
    radiusMax = max(radiusMin+2,ceil(0.75*smallSide));  % Atualiza radiusMax.
    radiusMax = min(radiusMax,floor(min(size(grayCrop))/2));  % Atualiza radiusMax.
    if radiusMax <= radiusMin  % Testa a condição sobre radiusMax, radiusMin.
        bestCenter = []; bestRadius = 0; bestScore = 0; found = false;  % Atualiza bestCenter.
        return  % Regressa à função que fez a chamada.
    end  % Fim do bloco.

    [centers,radii,metrics] = imfindcircles(grayCrop, ...  % Atualiza [centers,radii,metrics].
        [radiusMin radiusMax],'ObjectPolarity','dark', ...  % Completa os dados da instrução anterior.
        'Sensitivity',0.92,'EdgeThreshold',0.10);  % Completa os dados da instrução anterior.

    bestCenter = [];  % Atualiza bestCenter.
    bestRadius = 0;  % Define bestRadius.
    bestScore = -inf;  % Atualiza bestScore.
    found = false;  % Atualiza found.
    maxCenterDistance = 0.50*max(box(3),box(4));  % Atualiza maxCenterDistance.

    for k = 1:size(centers,1)  % Percorre os valores do índice k.
        centerCrop = centers(k,:);  % Atualiza centerCrop.
        distanceToBody = norm(centerCrop-bodyCenterCrop);  % Calcula a magnitude do desvio.
        if distanceToBody > maxCenterDistance  % Testa a condição sobre distanceToBody, maxCenterDistance.
            continue  % Passa à iteração seguinte.
        end  % Fim do bloco.
        blackCoverage = circleBlackCoverage(blackCrop,centerCrop,radii(k));  % Mede a parte escura da banda circular.
        if blackCoverage < 0.12  % Testa a condição sobre blackCoverage.
            continue  % Passa à iteração seguinte.
        end  % Fim do bloco.
        circleConfidence = min(max(double(metrics(k)),0),1);  % Atualiza circleConfidence.
        centerConfidence = 1-distanceToBody/max(maxCenterDistance,1);  % Atualiza centerConfidence.
        % Prefere circulos bem definidos, escuros e perto do corpo.
        score = 0.45*circleConfidence+0.40*blackCoverage+ ...  % Atualiza score.
            0.15*centerConfidence;  % Deslocamento e orientação do padrão nesta imagem.
        if score > bestScore  % Testa a condição sobre score, bestScore.
            % Volta a expressar o centro na imagem inteira.
            bestCenter = centerCrop+[x1-1,y1-1];  % Atualiza bestCenter.
            bestRadius = double(radii(k));  % Atualiza bestRadius.
            bestScore = score;  % Atualiza bestScore.
            found = true;  % Atualiza found.
        end  % Fim do bloco.
    end  % Fim do bloco.

    % Se a Hough nao encontrar o aro, usa o centroide dos pixeis pretos
    % dentro da caixa do copo como alternativa simples.
    if ~found  % Testa a condição sobre found.
        closedBlack = imclose(blackCrop,strel('disk',2,0));  % Une regiões separadas por pequenas falhas.
        components = bwconncomp(closedBlack);  % Agrupa píxeis vizinhos da máscara.
        blackStats = regionprops(components,'Area','Centroid','BoundingBox');  % Mede as regiões segmentadas.
        bestFallbackScore = -inf;  % Atualiza bestFallbackScore.
        for k = 1:numel(blackStats)  % Percorre os valores do índice k.
            blackBox = double(blackStats(k).BoundingBox);  % Atualiza blackBox.
            blackWidth = blackBox(3);  % Atualiza blackWidth.
            blackHeight = blackBox(4);  % Atualiza blackHeight.
            blackAspect = min(blackWidth,blackHeight)/max(blackWidth,blackHeight);  % Atualiza blackAspect.
            centerCrop = double(blackStats(k).Centroid);  % Atualiza centerCrop.
            distanceToBody = norm(centerCrop-bodyCenterCrop);  % Calcula a magnitude do desvio.
            if blackStats(k).Area < 8 || blackAspect < 0.45 || ...  % Testa a condição sobre blackStats, k, Area, blackAspect.
                    distanceToBody > maxCenterDistance  % Completa os dados da instrução anterior.
                continue  % Passa à iteração seguinte.
            end  % Fim do bloco.
            centerConfidence = 1-distanceToBody/max(maxCenterDistance,1);  % Atualiza centerConfidence.
            areaFraction = double(blackStats(k).Area)/ ...  % Atualiza areaFraction.
                max(blackWidth*blackHeight,1);  % Completa os dados da instrução anterior.
            areaConfidence = min(areaFraction/0.30,1);  % Atualiza areaConfidence.
            score = 0.45*blackAspect+0.35*centerConfidence+ ...  % Atualiza score.
                0.20*areaConfidence;  % Deslocamento e orientação do padrão nesta imagem.
            if score > bestFallbackScore  % Testa a condição sobre score, bestFallbackScore.
                bestCenter = centerCrop+[x1-1,y1-1];  % Atualiza bestCenter.
                bestRadius = mean([blackWidth,blackHeight])/2;  % Atualiza bestRadius.
                bestScore = score;  % Atualiza bestScore.
                bestFallbackScore = score;  % Atualiza bestFallbackScore.
                found = true;  % Atualiza found.
            end  % Fim do bloco.
        end  % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.

function coverage = circleBlackCoverage(blackMask,center,radius)  % Define as entradas e saídas desta função.
    [x,y] = meshgrid(1:size(blackMask,2),1:size(blackMask,1));  % Atualiza [x,y].
    distance = hypot(x-center(1),y-center(2));  % Calcula a distância no plano.
    % Mede os pixeis escuros numa faixa fina em torno do circulo.
    band = abs(distance-radius) <= max(2,0.15*radius);  % Atualiza band.
    coverage = nnz(blackMask & band)/max(nnz(band),1);  % Atualiza coverage.
end  % Fim do bloco.
