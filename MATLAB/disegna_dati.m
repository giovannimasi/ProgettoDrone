% Nome del file da leggere (assicurati che il nome corrisponda a quello creato precedentemente)
dataFileName = 'Testdefinitivi/dati_seriale_10-02-2025_13-38.csv';  % Modifica con il nome corretto

% Lettura dei dati dal file CSV
data = readtable(dataFileName);

% Creazione del vettore temporale con intervallo di 0.1 secondi
numSamples = height(data);            % Numero totale di campioni
timeVector = (0:numSamples-1) * 0.1;  % Vettore temporale da 0 a (n-1)*0.1

% Creazione dei plot
figure;

% Primo subplot: Motori
subplot(3,1,1);
plot(timeVector, data.Motor1, '-', 'LineWidth', 2); hold on;
plot(timeVector, data.Motor2, '-', 'LineWidth', 2);
plot(timeVector, data.Motor3, '-', 'LineWidth', 2);
plot(timeVector, data.Motor4, '-', 'LineWidth', 2);
legend('Motor1', 'Motor2', 'Motor3', 'Motor4');
title('Motors Output');
xlabel('Time (s)');  % Aggiunta dell'etichetta temporale
grid on;

% Secondo subplot: Roll e Pitch
subplot(3,1,2);
plot(timeVector, data.Roll, '-', 'LineWidth', 2); hold on;
plot(timeVector, data.Pitch, '-', 'LineWidth', 2);
legend('Roll', 'Pitch');
title('Orientation');
xlabel('Time (s)');
grid on;

% Terzo subplot: Inputs
subplot(3,1,3);
plot(timeVector, data.Input2, '-', 'LineWidth', 2); hold on;
plot(timeVector, data.Input3, '-', 'LineWidth', 2);
legend('Input2', 'Input3');
title('Virtual Inputs');
xlabel('Time (s)');
grid on;

% Visualizzazione delle costanti PID se presenti
if width(data) > 10
    costanti = data{20, 11:16};

    % Aggiunta delle costanti PID come annotazione
    annotation('textbox', [0.03, 0.8, 0.2, 0.1], ...
        'String', sprintf(['\\color{blue}KPR: %.8f\n' ...
                           '\\color{blue}KIR: %.8f\n' ...
                           '\\color{blue}KDR: %.8f\n' ...
                           '\\color{orange}KPP: %.8f\n' ...
                           '\\color{orange}KIP: %.8f\n' ...
                           '\\color{orange}KDP: %.8f'], ...
            costanti(1), costanti(2), costanti(3), costanti(4), costanti(5), costanti(6)), ...
        'FontSize', 12, 'Interpreter', 'tex', 'EdgeColor', 'none');
end
