# Visao e calibracao da camera

Esta pasta contem as rotinas de visao pedidas e os dados da calibracao:

- `detectBlueStripePX100.m` e `runVisionPX100.m`: deteccao da faixa azul e dos seus cantos, mantida como estava.
- `detectCupsPX100.m` e `runVisionCupsPX100.m`: detecao dos copos vermelho, azul e verde pelo corpo colorido e pelo centro do rebordo preto; estimativa do alvo XYZ.
- `calibrationPX100.m`: captura, calibracao, validacao e projecao pixel-plano.
- `cameraCalibration.mat`: parametros intrinsecos e transformacoes calibradas.
- `images/`: imagens do tabuleiro usadas na calibracao.

`main_program.m` deteta os tres copos, mas nesta fase so tenta agarrar o vermelho.
O centro do rebordo preto fornece X/Y; Z de pega e calculado como metade da altura conhecida do copo.

## Calibracao

Executa na pasta raiz:

```matlab
addpath('vision')
calibrationPX100('capture',15)
calibrationPX100('calibrate')
calibrationPX100('transform')
calibrationPX100('validate')
```

A grelha tem quadrados de `0,0625 m`. As coordenadas projetadas sao expressas em metros no referencial `/px100`.
