% === Configuración inicial ===
clear; clc;

% Configura el puerto serial (ajusta COMX por el puerto correcto)
serialPort = "COM33"; % <-- Cambia esto por el puerto correcto
baudRate = 115200;

% Abre la conexión serial
s = serialport(serialPort, baudRate);

% Variables para graficar
numSamples = 200; % Cantidad de muestras a mostrar en el gráfico
distanceData = nan(1, numSamples);
timeData = (1:numSamples);

% === Figura en tiempo real ===
figure;
hPlot = plot(timeData, distanceData, '-'); % Línea sin puntos
ylim([0 500]); % Ajusta según el rango real de VL53L1X (hasta 4000mm en modo long)
xlabel('Muestra');
ylabel('Distancia [mm]');
title('Lectura en Tiempo Real - VL53L1X (Short Mode)');
grid on;

% === Loop de adquisición ===
disp("Iniciando lectura, presiona Ctrl+C para detener.");

while true
    if s.NumBytesAvailable > 0
        % Lee la línea completa desde el Arduino
        dataLine = readline(s);
        
        % Busca solo las líneas que contienen 'Distance:'
        if contains(dataLine, "Distance:")
            % Extrae el número usando expresiones regulares
            distance = regexp(dataLine, 'Distance: (\d+)', 'tokens');
            
            if ~isempty(distance)
                % Convierte a número
                d = str2double(distance{1}{1});
                
                % Actualiza el vector de datos
                distanceData = [distanceData(2:end), d];
                
                % Actualiza el gráfico
                set(hPlot, 'YData', distanceData);
                drawnow;
            end
        end
    end
end
