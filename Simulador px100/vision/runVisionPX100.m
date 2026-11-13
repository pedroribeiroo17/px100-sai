function visionInfo = runVisionPX100(simInterface, markerPath)
% Captura uma imagem, deteta a faixa azul e calcula os XYZ dos seus cantos.
% A simulacao deve estar ativa e a interface deve ter visao habilitada.

    if nargin < 2 || isempty(markerPath), markerPath = '/dummy/marcador'; end
    [image,~] = simInterface.get_camera_information();
    image = uint8(image);
    stripe = detectBlueStripePX100(image);
    visionInfo = struct('image',image,'stripe',stripe,'cornersXYZ',[]);

    figure('Name','Visao e faixa azul');
    imshow(image);
    hold on
    try
        originPixel = calibrationPX100('originpixel',simInterface);
        if originPixel(1)>=1 && originPixel(1)<=size(image,2) && ...
                originPixel(2)>=1 && originPixel(2)<=size(image,1)
            plot(originPixel(1),originPixel(2),'y*','MarkerSize',16,'LineWidth',2);
            text(originPixel(1)+8,originPixel(2),'Base (0,0,0)', ...
                'Color','y','FontWeight','bold');
            fprintf('Origem da base projetada: [%.1f %.1f] px\n',originPixel);
        else
            fprintf('Origem da base fora da imagem: [%.1f %.1f] px\n',originPixel);
        end
    catch ME
        warning('Nao foi possivel projetar a origem da base: %s',ME.message);
    end

    if isempty(stripe)
        title('Faixa azul nao detetada');
        disp('Faixa azul nao detetada.');
    else
        visboundaries(stripe.mask,'Color','y','LineWidth',1);
        plot(stripe.lowerCorners(:,1),stripe.lowerCorners(:,2),'o', ...
            'MarkerSize',14,'MarkerFaceColor','r', ...
            'MarkerEdgeColor','y','LineWidth',2);
        text(stripe.lowerCorners(:,1)+5,stripe.lowerCorners(:,2), ...
            {'Canto 1','Canto 2'},'Color','y','FontWeight','bold');
        title('Faixa azul e cantos inferiores');
        fprintf('Canto 1 [x y] px: [%.1f %.1f]\n',stripe.lowerCorners(1,:));
        fprintf('Canto 2 [x y] px: [%.1f %.1f]\n',stripe.lowerCorners(2,:));
        try
            visionInfo.cornersXYZ = calibrationPX100('project', ...
                stripe.lowerCorners,markerPath,simInterface);
        catch ME
            warning('Nao foi possivel converter os cantos para XYZ: %s',ME.message);
        end
    end
    hold off
    drawnow;
    figure(gcf);
end
