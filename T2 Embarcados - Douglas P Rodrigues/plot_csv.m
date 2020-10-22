clear all
close all
clc

dados = csvread('C:\Users\dougl\Google Drive\Faculdade\GitHub\Embarcados\Trabalho 2 - Douglas P Rodrigues\log2.csv');

figure(1)
subplot(3,3,1),plot(dados(:,1))
title('Temperatura [Cº]')
subplot(3,3,2), plot(dados(:,2))
title('Pressão [hPa]')
subplot(3,3,3), plot(dados(:,3))
title('Altitude [m]')
subplot(3,3,4),plot(dados(:,4))
title('Velocidade motor [%]')
subplot(3,3,5), plot(dados(:,5))
title('Direção do motor - 1D, 2E')
subplot(3,3,6), plot(dados(:,6))
title('Direção do vento [º]')
subplot(3,3,7),plot(dados(:,7))
title('Quantidade chuva [mm]%')
subplot(3,3,8), plot(dados(:,8))
title('Luminosidade - sol=1, par nublado=2, nublado=3, anoitecer=4, noite=5')
subplot(3,3,9), plot(dados(:,9))
title('Velocidade vento [km/h]')


