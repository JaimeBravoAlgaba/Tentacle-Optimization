function logService(type, text)
    %LOGSERVICE Prints a message in console
    %   Different levels or priority are defined:
    %   0. OFF
    %   1. FATAL
    %   2. ERROR
    %   3. WARN
    %   4. INFO
    %   5. TRACE
    %   6. DEBUG
    %   7. ALL
    
    level = 7;  %Level of priority for displaying the messages.
    
    switch type
        case 'FATAL'
            fatal(level, text);
        case 'ERROR'
            error(level, text);
        case 'WARN'
            warn(level, text);
        case 'INFO'
            info(level, text);
        case 'TRACE'
            trace(level, text);
        case 'DEBUG'
            debug(level, text);
        case 'TEST'
            fatal(level, text);
            error(level, text);
            warn(level, text);
            info(level, text);
            trace(level, text);
            debug(level, text);
        otherwise
        %Extract hour of the system
        fechaHora = datestr(datetime('now'));
        splitFH = regexp(fechaHora, ' ', 'split');

        disp(splitFH{2} + "  " + type + " >> " + text)
    end
end

%% FATAL MESSAGE
function fatal(level, text)
    %FATAL fatal error
    if level >= 1
        %Extract hour of the system
        fechaHora = datestr(datetime('now'));
        splitFH = regexp(fechaHora, ' ', 'split');

        disp(splitFH{2} + "  FATAL >> " + text)
    end
end

%% ERROR MESSAGE
function error(level, text)
    %ERROR error
    if level >= 2
        %Extract hour of the system
        fechaHora = datestr(datetime('now'));
        splitFH = regexp(fechaHora, ' ', 'split');

        disp(splitFH{2} + "  ERROR >> " + text)
    end
end

%% WARNING MESSAGE
function warn(level,text)
    %WARN warning
    if level >= 3
        %Extract hour of the system
        fechaHora = datestr(datetime('now'));
        splitFH = regexp(fechaHora, ' ', 'split');

        disp(splitFH{2} + "  WARN >> " + text)
    end
end

%% INFO MESSAGE
function info(level,text)
    %INFO info
    if level >= 4
        %Extract hour of the system
        fechaHora = datestr(datetime('now'));
        splitFH = regexp(fechaHora, ' ', 'split');

        disp(splitFH{2} + "  INFO >> " + text)
    end
end

%% TRACE MESSAGE
function trace(level,text)
    %TRACE trace
    if level >= 5
        %Extract hour of the system
        fechaHora = datestr(datetime('now'));
        splitFH = regexp(fechaHora, ' ', 'split');

        disp(splitFH{2} + "  TRACE >> " + text)
    end
end

%% DEBUG MESSAGE
function debug(level,text)
    %DEBUG debug
    if level >= 6
        %Extract hour of the system
        fechaHora = datestr(datetime('now'));
        splitFH = regexp(fechaHora, ' ', 'split');

        disp(splitFH{2} + "  DEBUG >> " + text)
    end
end

