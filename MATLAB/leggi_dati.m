s = serialport('/dev/ttyACM0', 115200);  % Sostituisci COM3 con la tua porta seriale
configureTerminator(s, "LF");  % Imposta il terminatore di riga correttamente

% Definisci una funzione di pulizia da eseguire quando lo script si interrompe
cleanupObj = onCleanup(@() cleanupFunction(s));

dateStr = datestr(now, 'dd-mm-yyyy_HH-MM');
%title = input("Titolo: ");
dataFileName = sprintf('TestFinali/dati_seriale_%s.csv', dateStr);
dataFile = fopen(dataFileName, 'w');

fprintf(dataFile, 'Motor1, Motor2, Motor3, Motor4, Roll, Pitch, Input1, Input2, Input3, Input4, KPR, KIR, KDR, KPP, KIP, KDP\n');

numSamples = 2000;  % Numero di righe da leggere
try
    for i = 1:numSamples
        line = readline(s);
        values = sscanf(line, '%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f');
        if i<20
            continue;
        end
        fprintf(dataFile, '%.2f, %.2f, %.2f, %.2f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f\n', values);
    end
catch ME
    warning('Errore durante la lettura dei dati: %s', ME.message);
finally
    fclose(dataFile);
    delete(s);
    clear s;
    disp('Porta seriale chiusa correttamente.');
end

disp(['Dati salvati in ', dataFileName]);

function cleanupFunction(s)
    disp('Eseguendo operazioni di chiusura...');

    % Chiudi il file se è aperto
    try
        if exist('dataFile', 'var') && ~isempty(fopen(dataFile))
            fclose(dataFile);
        end
    catch
        warning('Errore nella chiusura del file.');
    end

    % Elimina la porta seriale
    try
        if isvalid(s)
            delete(s);
        end
    catch
        warning('Errore nella chiusura della porta seriale.');
    end

    % Chiudi tutte le connessioni seriali residue
    if ~isempty(serialportfind)
        delete(serialportfind);
    end

    % Pulisci completamente la memoria
    clear s;
    clear all;  % Prova con clear classes se il problema persiste
    disp('Risorse liberate correttamente.');
end