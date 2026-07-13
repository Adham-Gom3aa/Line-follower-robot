function lineFollowerGUI
    % ---------- UI ----------
    fig = uifigure('Name','Robot PID Controller','Position',[100 100 520 360]);
    gl = uigridlayout(fig,[6 2]);
    gl.RowHeight = {'fit','fit','fit','fit','fit','fit'};
    gl.ColumnWidth = {'1x','1x'};

    uilabel(gl,'Text','Proportional Gain (Kp):');
    kpEdit = uieditfield(gl,'numeric','Value',0.5);

    uilabel(gl,'Text','Integral Gain (Ki):');
    kiEdit = uieditfield(gl,'numeric','Value',50);

    uilabel(gl,'Text','Derivative Gain (Kd):');
    kdEdit = uieditfield(gl,'numeric','Value',0.001);

    uilabel(gl,'Text','Setpoint:');
    setpointEdit = uieditfield(gl,'numeric','Value',0);

    updateButton = uibutton(gl,'Text','Update Parameters', ...
        'ButtonPushedFcn',@updateParameters);

    plotButton = uibutton(gl,'Text','Start Plotting', ...
        'ButtonPushedFcn',@togglePlotting);

    % ---------- Serial ----------
    port = "COM4";                 % <<< CHANGE THIS
    baud = 19200;

    serialObj = serialport(port, baud);
    configureTerminator(serialObj,"LF");
    serialObj.Timeout = 0.1;
    flush(serialObj);

    % ---------- Plot state ----------
    plotting = false;
    plotFig = [];
    ax = [];
    hLine = [];
    xData = [];
    yData = [];

    % Cleanup on close
    fig.CloseRequestFcn = @onClose;

    % ---------- Callbacks ----------
    function updateParameters(~,~)
        kp = kpEdit.Value;
        ki = kiEdit.Value;
        kd = kdEdit.Value;
        setpoint = setpointEdit.Value;

        % Send: "kp,ki,kd,setpoint" (Arduino must implement parsing to use it)
        writeline(serialObj, sprintf('%.6f,%.6f,%.6f,%.6f', kp, ki, kd, setpoint));
    end

    function togglePlotting(~,~)
        if ~plotting
            startPlotting();
            plotButton.Text = "Stop Plotting";
            plotting = true;
        else
            stopPlotting();
            plotButton.Text = "Start Plotting";
            plotting = false;
        end
    end

    function startPlotting()
        % Create plot window once
        if isempty(plotFig) || ~isvalid(plotFig)
            plotFig = figure('Name','Error Plot','Position',[700 100 650 420]);
            ax = axes('Parent',plotFig);
            grid(ax,'on');
            xlabel(ax,'Time (ms)');
            ylabel(ax,'Error');
            hLine = plot(ax, nan, nan, 'r-', 'LineWidth', 1.5);

            plotFig.CloseRequestFcn = @onPlotClose;
            xData = [];
            yData = [];
        end

        % Trigger on each complete line
        configureCallback(serialObj,"terminator",@onSerialLine);
    end

    function stopPlotting()
        configureCallback(serialObj,"off");
    end

    function onSerialLine(src,~)
        % If plot window was closed, stop reading
        if isempty(plotFig) || ~isvalid(plotFig)
            configureCallback(src,"off");
            return;
        end

        line = strtrim(readline(src));
        if startsWith(line,"Time") % skip CSV header if Arduino prints it
            return;
        end

        parts = split(line,',');

        % Support either:
        %  - "time,error" (2 fields)
        %  - "time,position,goal,error,pid" (5 fields)
        if numel(parts) >= 5
            t_ms = str2double(parts{1});
            err  = str2double(parts{4});
        elseif numel(parts) >= 2
            t_ms = str2double(parts{1});
            err  = str2double(parts{2});
        else
            return;
        end

        if isnan(t_ms) || isnan(err)
            return;
        end

        xData(end+1) = t_ms;
        yData(end+1) = err;

        set(hLine, 'XData', xData, 'YData', yData);
        drawnow limitrate
    end

    function onPlotClose(~,~)
        % Stop callbacks before closing plot window
        configureCallback(serialObj,"off");
        delete(plotFig);
        plotFig = [];
    end

    function onClose(~,~)
        % Stop serial callbacks and release the port
        try
            configureCallback(serialObj,"off");
            flush(serialObj);
        catch
        end
        clear serialObj
        delete(fig);
    end
end