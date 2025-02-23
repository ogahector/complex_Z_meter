clc; clear;
close all

%% --- Settings --- %%

comp_name = '100R';
% Choose the component by selecting component=index for the following list
% COMPONENTS = [100pF, 10nF, 1uF, 100nH, 100R];
component = 5;
save = 1;

%% ---- Import ---- %%

filename = strcat('data/', comp_name, '_cartesian.txt'); % Specify the input file

file = fopen(filename, 'r');
data = containers.Map; % Initialize container map to store data

while ~feof(file)
    % Read the line containing the step information
    line = fgetl(file);
    if startsWith(line, 'Step Information:')
        % Extract setup name from step information
        setup_name = split(line, 'Step Information: ');
        setup_name = setup_name(2); setup_name = setup_name{1};
        setup_name = setup_name(1:end-13); % Remove the "(Step: X/Y)" part

        % Extract stepped parameters
        Fp = str2double(extractBetween(setup_name, 'Fp=', ' '));
        A_str = extractBetween(setup_name, 'A=', 'K R');
        if isempty(A_str) % if not K then 1M
            A = 1e6;
        else
            A = 100e3;
        end
        Rg_str = extractBetween(setup_name, 'Rg=', 'K');
        if isempty(Rg_str)
            Rg_str = extractBetween(setup_name, 'Rg=', 'M');
        end
        if strcmp(Rg_str, '1')
            Rg = 1e3;
        elseif strcmp(Rg_str, '100')
            Rg = 100e3;
        elseif strcmp(Rg_str, '10')
            Rg = 10e6;
        end
        % Calculate GBW and add to setup_name
        gbw = Fp * A;
        gbw_str = num2str(gbw/1e6) + "M";
        setup_name = setup_name + "GBW=" + gbw_str;

        % Initialize containers for frequency and response
        freq = [];
        response = [];

        % Read frequency and response data until the next step or EOF
        while ~feof(file)
            line = fgetl(file);
            if startsWith(line, 'Step Information:') || isempty(line)
                fseek(file, -length(line)-2, 'cof'); % Move back to re-read the step information
                break;
            end

            % Parse frequency and response values
            values = sscanf(line, '%f\t%f,%f');
            if length(values) == 3
                freq(end+1) = values(1);
                response(end+1) = values(2) + 1j * values(3);
            end
        end

        % Store data in the container map
        Vout.freq = freq;
        Vout.response = response;
        Vout.Fp = Fp;
        Vout.A = A;
        Vout.gbw = gbw;
        Vout.Rg = Rg;
        data(setup_name) = Vout;
    end
end
fclose(file);

%% ---- Display ---- %%

% Plot data for each setup
setups = data.keys();
Errors = zeros(length(setups), length(Vout.response));
RMSEs = zeros(length(setups), 1); % Initialize mean square error array
colors = linspecer(length(setups)); % Assign different colors for each setup

figure;
set(gcf, 'Position', [10 50 900 600]);
% Assign ideal values for different components
idealR = 100 .* ones(1, length(Vout.freq));
idealL = 2 .* pi .* 100e-9 .* Vout.freq;
idealC100p = 1 ./ (2 .* pi .* 100e-12 .* Vout.freq);
idealC10n = 1 ./ (2 .* pi .* 10e-9 .* Vout.freq);
idealC1u = 1 ./ (2 .* pi .* 1e-6 .* Vout.freq);
ideal = [idealC100p; idealC10n; idealC1u; idealL; idealR];

for i = 1:length(setups)
    SETUP = setups{i};
    Vout = data(SETUP);

    % Calculate Zx
    Zx.freq = Vout.freq;
    Zx.response = -Vout.Rg ./ Vout.response;

    % Calculate percentage error
    % percentage_error = abs((Zx.response - ideal(component, :)) ./ ideal(component, :)) * 100;
    percentage_error = abs((log10(Zx.response) - log10(ideal(component, :))) ./ log10(ideal(component, :))) * 100;
    Errors(i, :) = percentage_error;

    % Calculate mean square error
    log_difference = log10(abs(Zx.response)) - log10(abs(ideal(component, :)));
    rmse = sqrt(mean(log_difference.^2));
    RMSEs(i) = rmse;

    % Plot Vout and Zx responses

    % Plot Vout response (polar)
    % subplot(1, 3, 1);
    % yyaxis right;
    % semilogx(Vout.freq, 180/pi*angle(Vout.response), '--', 'Color', colors(i, :), LineWidth=2);
    % ylabel('Phase [deg]');
    % hold on;
    % yyaxis left;
    % semilogx(Vout.freq, 20*log10(abs(Vout.response)), '-', 'Color', colors(i, :), LineWidth=2);
    % ylabel('Amplitude [dB]');
    % grid on;
    % title("Vout");
    % xlim([10, 1e6]);

    % Plot Zx response
    %subplot(1, 2, 1);
    yyaxis right;
    semilogx(Zx.freq, 180/pi*angle(Zx.response), '--', 'Color', colors(i, :), LineWidth=1);
    ylabel('Phase [deg]');
    hold on;
    yyaxis left;
    semilogx(Zx.freq, log10(abs(Zx.response)), '-', 'Color', colors(i, :), LineWidth=2);
    if i==length(setups)
        semilogx(Zx.freq, log10(ideal(component, :)), '--k', LineWidth=1);
    end
    ylabel('log|Zx|');
    grid on;
    title("Zx");
    xlim([10, 1e6]);

    % Plot Zx response (complex impedance)
    % subplot(1, 2, 2);
    % yyaxis left;
    % semilogx(Zx.freq, real(Zx.response), '-', 'Color', colors(i, :), LineWidth=2);
    % ylabel('Real [Ohms]');
    % hold on;
    % yyaxis right;
    % semilogx(Zx.freq, imag(Zx.response), '--', 'Color', colors(i, :), LineWidth=2);
    % ylabel('Imaginary [Ohms]');
    % grid on;
    % title("Zx (complex impedance)");
    % xlim([10, 1e6]);
end

if save
    saveas(gcf, fullfile('figures', strcat(comp_name, '_Zx_response.fig')));
end

% Create a new figure for the legend
figure;
% Dummy plot to create legend entries
hold on;
for i = 1:length(setups)
    plot(NaN, NaN, 'Color', colors(i, :), 'LineWidth', 4); % Create invisible lines for legend
end
plot(NaN, NaN, '--k', 'LineWidth', 2);
hold off;

% Create legend
setups_leg = setups;
setups_leg{end+1} = 'Ideal Component';
legend(setups_leg, 'Location', 'north', 'Interpreter', 'none', 'FontSize', 16);
axis off; % Hide axes
title('Legend', 'FontSize', 20);

% Save legend figure
if save
    saveas(gcf, fullfile('figures', 'Legend.fig'));
end

% Plot percentage error vs frequency
figure;
set(gcf, 'Position', [10 100 600 600]);
for i = 1:length(setups)
    semilogx(Zx.freq, Errors(i, :), 'Color', colors(i, :), LineWidth=2);
    hold on;
end
grid on;
xlabel('Frequency [Hz]');
ylabel('Percentage Error [%]');
xlim([10, 1e6]);
ylim([0, 140]);
title('Percentage Error');
% legend(setups, 'Interpreter', 'none');
if save
    saveas(gcf, fullfile('figures', strcat(comp_name, '_Percentage_Error.fig')));
end

% Plot MSE vs Rg, GBW, A, and Fp
Rg_values = cellfun(@(s) data(s).Rg, setups);
GBW_values = cellfun(@(s) data(s).gbw, setups);
A_values = cellfun(@(s) data(s).A, setups);
Fp_values = cellfun(@(s) data(s).Fp, setups);

figure;
set(gcf, 'Position', [620 100 500 500]);
subplot(2, 2, 1);
hold on;
for i=1:length(Rg_values)
    scatter(Rg_values(i) / 1e3, RMSEs(i), 40, 'o', 'LineWidth', 2.5, 'MarkerEdgeColor', colors(i, :));
end
xlabel('Rg [KOhms]');
ylabel('Root Mean Square Error');
title('RMSE vs Rg');
grid on;
% Adjust limits with buffer
x_buffer = 0.1 * range(Rg_values / 1e3); % 10% buffer for x-axis
y_buffer = 0.1 * range(RMSEs);          % 10% buffer for y-axis
xlim([min(Rg_values / 1e3) - x_buffer, max(Rg_values / 1e3) + x_buffer]);
ylim([min(RMSEs) - y_buffer, max(RMSEs) + y_buffer]);

subplot(2, 2, 2);
hold on;
for i=1:length(GBW_values)
    scatter(GBW_values(i) / 1e6, RMSEs(i), 40, 'o', 'LineWidth', 2.5, 'MarkerEdgeColor', colors(i, :));
end
xlabel('GBW [MHz]');
ylabel('Root Mean Square Error');
title('RMSE vs GBW');
grid on;
x_buffer = 0.1 * range(GBW_values/1e6); % 10% buffer for x-axis
xlim([min(GBW_values/1e6) - x_buffer, max(GBW_values/1e6) + x_buffer]);
ylim([min(RMSEs) - y_buffer, max(RMSEs) + y_buffer]);

subplot(2, 2, 3);
hold on;
for i=1:length(A_values)
    scatter(A_values(i), RMSEs(i), 40, 'o', 'LineWidth', 2.5, 'MarkerEdgeColor', colors(i, :));
end
xlabel('A [Gain]');
ylabel('Root Mean Square Error');
title('RMSE vs A');
grid on;
x_buffer = 0.1 * range(A_values); % 10% buffer for x-axis
xlim([min(A_values) - x_buffer, max(A_values) + x_buffer]);
ylim([min(RMSEs) - y_buffer, max(RMSEs) + y_buffer]);

subplot(2, 2, 4);
hold on;
for i=1:length(Fp_values)
    scatter(Fp_values(i), RMSEs(i), 40, 'o', 'LineWidth', 2.5, 'MarkerEdgeColor', colors(i, :));
end
xlabel('Fp [Hz]');
ylabel('Root Mean Square Error');
title('RMSE vs Fp');
grid on;
x_buffer = 0.1 * range(Fp_values); % 10% buffer for x-axis
xlim([min(Fp_values) - x_buffer, max(Fp_values) + x_buffer]);
ylim([min(RMSEs) - y_buffer, max(RMSEs) + y_buffer]);
if save
    saveas(gcf, fullfile('figures', strcat(comp_name, '_RMSE_Plots.fig')));
end

%% Colors
function colors = distinguishable_colors(n_colors,bg,func)
% DISTINGUISHABLE_COLORS: pick colors that are maximally perceptually distinct
%
% When plotting a set of lines, you may want to distinguish them by color.
% By default, Matlab chooses a small set of colors and cycles among them,
% and so if you have more than a few lines there will be confusion about
% which line is which. To fix this problem, one would want to be able to
% pick a much larger set of distinct colors, where the number of colors
% equals or exceeds the number of lines you want to plot. Because our
% ability to distinguish among colors has limits, one should choose these
% colors to be "maximally perceptually distinguishable."
%
% This function generates a set of colors which are distinguishable
% by reference to the "Lab" color space, which more closely matches
% human color perception than RGB. Given an initial large list of possible
% colors, it iteratively chooses the entry in the list that is farthest (in
% Lab space) from all previously-chosen entries. While this "greedy"
% algorithm does not yield a global maximum, it is simple and efficient.
% Moreover, the sequence of colors is consistent no matter how many you
% request, which facilitates the users' ability to learn the color order
% and avoids major changes in the appearance of plots when adding or
% removing lines.
%
% Syntax:
%   colors = distinguishable_colors(n_colors)
% Specify the number of colors you want as a scalar, n_colors. This will
% generate an n_colors-by-3 matrix, each row representing an RGB
% color triple. If you don't precisely know how many you will need in
% advance, there is no harm (other than execution time) in specifying
% slightly more than you think you will need.
%
%   colors = distinguishable_colors(n_colors,bg)
% This syntax allows you to specify the background color, to make sure that
% your colors are also distinguishable from the background. Default value
% is white. bg may be specified as an RGB triple or as one of the standard
% "ColorSpec" strings. You can even specify multiple colors:
%     bg = {'w','k'}
% or
%     bg = [1 1 1; 0 0 0]
% will only produce colors that are distinguishable from both white and
% black.
%
%   colors = distinguishable_colors(n_colors,bg,rgb2labfunc)
% By default, distinguishable_colors uses the image processing toolbox's
% color conversion functions makecform and applycform. Alternatively, you
% can supply your own color conversion function.
%
% Example:
%   c = distinguishable_colors(25);
%   figure
%   image(reshape(c,[1 size(c)]))
%
% Example using the file exchange's 'colorspace':
%   func = @(x) colorspace('RGB->Lab',x);
%   c = distinguishable_colors(25,'w',func);
% Copyright 2010-2011 by Timothy E. Holy
  % Parse the inputs
  if (nargin < 2)
    bg = [1 1 1];  % default white background
  else
    if iscell(bg)
      % User specified a list of colors as a cell aray
      bgc = bg;
      for i = 1:length(bgc)
	bgc{i} = parsecolor(bgc{i});
      end
      bg = cat(1,bgc{:});
    else
      % User specified a numeric array of colors (n-by-3)
      bg = parsecolor(bg);
    end
  end
  
  % Generate a sizable number of RGB triples. This represents our space of
  % possible choices. By starting in RGB space, we ensure that all of the
  % colors can be generated by the monitor.
  n_grid = 30;  % number of grid divisions along each axis in RGB space
  x = linspace(0,1,n_grid);
  [R,G,B] = ndgrid(x,x,x);
  rgb = [R(:) G(:) B(:)];
  if (n_colors > size(rgb,1)/3)
    error('You can''t readily distinguish that many colors');
  end
  
  % Convert to Lab color space, which more closely represents human
  % perception
  if (nargin > 2)
    lab = func(rgb);
    bglab = func(bg);
  else
    C = makecform('srgb2lab');
    lab = applycform(rgb,C);
    bglab = applycform(bg,C);
  end
  % If the user specified multiple background colors, compute distances
  % from the candidate colors to the background colors
  mindist2 = inf(size(rgb,1),1);
  for i = 1:size(bglab,1)-1
    dX = bsxfun(@minus,lab,bglab(i,:)); % displacement all colors from bg
    dist2 = sum(dX.^2,2);  % square distance
    mindist2 = min(dist2,mindist2);  % dist2 to closest previously-chosen color
  end
  
  % Iteratively pick the color that maximizes the distance to the nearest
  % already-picked color
  colors = zeros(n_colors,3);
  lastlab = bglab(end,:);   % initialize by making the "previous" color equal to background
  for i = 1:n_colors
    dX = bsxfun(@minus,lab,lastlab); % displacement of last from all colors on list
    dist2 = sum(dX.^2,2);  % square distance
    mindist2 = min(dist2,mindist2);  % dist2 to closest previously-chosen color
    [~,index] = max(mindist2);  % find the entry farthest from all previously-chosen colors
    colors(i,:) = rgb(index,:);  % save for output
    lastlab = lab(index,:);  % prepare for next iteration
  end
end
function c = parsecolor(s)
  if ischar(s)
    c = colorstr2rgb(s);
  elseif isnumeric(s) && size(s,2) == 3
    c = s;
  else
    error('MATLAB:InvalidColorSpec','Color specification cannot be parsed.');
  end
end
function c = colorstr2rgb(c)
  % Convert a color string to an RGB value.
  % This is cribbed from Matlab's whitebg function.
  % Why don't they make this a stand-alone function?
  rgbspec = [1 0 0;0 1 0;0 0 1;1 1 1;0 1 1;1 0 1;1 1 0;0 0 0];
  cspec = 'rgbwcmyk';
  k = find(cspec==c(1));
  if isempty(k)
    error('MATLAB:InvalidColorString','Unknown color string.');
  end
  if k~=3 || length(c)==1,
    c = rgbspec(k,:);
  elseif length(c)>2,
    if strcmpi(c(1:3),'bla')
      c = [0 0 0];
    elseif strcmpi(c(1:3),'blu')
      c = [0 0 1];
    else
      error('MATLAB:UnknownColorString', 'Unknown color string.');
    end
  end
end


% function lineStyles = linspecer(N)
% This function creates an Nx3 array of N [R B G] colors
% These can be used to plot lots of lines with distinguishable and nice
% looking colors.
% 
% lineStyles = linspecer(N);  makes N colors for you to use: lineStyles(ii,:)
% 
% colormap(linspecer); set your colormap to have easily distinguishable 
%                      colors and a pleasing aesthetic
% 
% lineStyles = linspecer(N,'qualitative'); forces the colors to all be distinguishable (up to 12)
% lineStyles = linspecer(N,'sequential'); forces the colors to vary along a spectrum 
% 
% % Examples demonstrating the colors.
% 
% LINE COLORS
% N=6;
% X = linspace(0,pi*3,1000); 
% Y = bsxfun(@(x,n)sin(x+2*n*pi/N), X.', 1:N); 
% C = linspecer(N);
% axes('NextPlot','replacechildren', 'ColorOrder',C);
% plot(X,Y,'linewidth',5)
% ylim([-1.1 1.1]);
% 
% SIMPLER LINE COLOR EXAMPLE
% N = 6; X = linspace(0,pi*3,1000);
% C = linspecer(N)
% hold off;
% for ii=1:N
%     Y = sin(X+2*ii*pi/N);
%     plot(X,Y,'color',C(ii,:),'linewidth',3);
%     hold on;
% end
% 
% COLORMAP EXAMPLE
% A = rand(15);
% figure; imagesc(A); % default colormap
% figure; imagesc(A); colormap(linspecer); % linspecer colormap
% 
%   See also NDHIST, NHIST, PLOT, COLORMAP, 43700-cubehelix-colormaps
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by Jonathan Lansey, March 2009-2013 � Lansey at gmail.com               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%% credits and where the function came from
% The colors are largely taken from:
% http://colorbrewer2.org and Cynthia Brewer, Mark Harrower and The Pennsylvania State University
% 
% 
% She studied this from a phsychometric perspective and crafted the colors
% beautifully.
% 
% I made choices from the many there to decide the nicest once for plotting
% lines in Matlab. I also made a small change to one of the colors I
% thought was a bit too bright. In addition some interpolation is going on
% for the sequential line styles.
% 
% 
%%
function lineStyles=linspecer(N,varargin)
if nargin==0 % return a colormap
    lineStyles = linspecer(128);
    return;
end
if ischar(N)
    lineStyles = linspecer(128,N);
    return;
end
if N<=0 % its empty, nothing else to do here
    lineStyles=[];
    return;
end
% interperet varagin
qualFlag = 0;
colorblindFlag = 0;
if ~isempty(varargin)>0 % you set a parameter?
    switch lower(varargin{1})
        case {'qualitative','qua'}
            if N>12 % go home, you just can't get this.
                warning('qualitiative is not possible for greater than 12 items, please reconsider');
            else
                if N>9
                    warning(['Default may be nicer for ' num2str(N) ' for clearer colors use: whitebg(''black''); ']);
                end
            end
            qualFlag = 1;
        case {'sequential','seq'}
            lineStyles = colorm(N);
            return;
        case {'white','whitefade'}
            lineStyles = whiteFade(N);return;
        case 'red'
            lineStyles = whiteFade(N,'red');return;
        case 'blue'
            lineStyles = whiteFade(N,'blue');return;
        case 'green'
            lineStyles = whiteFade(N,'green');return;
        case {'gray','grey'}
            lineStyles = whiteFade(N,'gray');return;
        case {'colorblind'}
            colorblindFlag = 1;
        otherwise
            warning(['parameter ''' varargin{1} ''' not recognized']);
    end
end      
% *.95
% predefine some colormaps
  set3 = colorBrew2mat({[141, 211, 199];[ 255, 237, 111];[ 190, 186, 218];[ 251, 128, 114];[ 128, 177, 211];[ 253, 180, 98];[ 179, 222, 105];[ 188, 128, 189];[ 217, 217, 217];[ 204, 235, 197];[ 252, 205, 229];[ 255, 255, 179]}');
set1JL = brighten(colorBrew2mat({[228, 26, 28];[ 55, 126, 184]; [ 77, 175, 74];[ 255, 127, 0];[ 255, 237, 111]*.85;[ 166, 86, 40];[ 247, 129, 191];[ 153, 153, 153];[ 152, 78, 163]}'));
set1 = brighten(colorBrew2mat({[ 55, 126, 184]*.85;[228, 26, 28];[ 77, 175, 74];[ 255, 127, 0];[ 152, 78, 163]}),.8);
% colorblindSet = {[215,25,28];[253,174,97];[171,217,233];[44,123,182]};
colorblindSet = {[215,25,28];[253,174,97];[171,217,233]*.8;[44,123,182]*.8};
set3 = dim(set3,.93);
if colorblindFlag
    switch N
        %     sorry about this line folks. kind of legacy here because I used to
        %     use individual 1x3 cells instead of nx3 arrays
        case 4
            lineStyles = colorBrew2mat(colorblindSet);
        otherwise
            colorblindFlag = false;
            warning('sorry unsupported colorblind set for this number, using regular types');
    end
end
if ~colorblindFlag
    switch N
        case 1
            lineStyles = { [  55, 126, 184]/255};
        case {2, 3, 4, 5 }
            lineStyles = set1(1:N);
        case {6 , 7, 8, 9}
            lineStyles = set1JL(1:N)';
        case {10, 11, 12}
            if qualFlag % force qualitative graphs
                lineStyles = set3(1:N)';
            else % 10 is a good number to start with the sequential ones.
                lineStyles = cmap2linspecer(colorm(N));
            end
        otherwise % any old case where I need a quick job done.
            lineStyles = cmap2linspecer(colorm(N));
    end
end
lineStyles = cell2mat(lineStyles);
end
% extra functions
function varIn = colorBrew2mat(varIn)
for ii=1:length(varIn) % just divide by 255
    varIn{ii}=varIn{ii}/255;
end        
end
function varIn = brighten(varIn,varargin) % increase the brightness
if isempty(varargin),
    frac = .9; 
else
    frac = varargin{1}; 
end
for ii=1:length(varIn)
    varIn{ii}=varIn{ii}*frac+(1-frac);
end        
end
function varIn = dim(varIn,f)
    for ii=1:length(varIn)
        varIn{ii} = f*varIn{ii};
    end
end
function vOut = cmap2linspecer(vIn) % changes the format from a double array to a cell array with the right format
vOut = cell(size(vIn,1),1);
for ii=1:size(vIn,1)
    vOut{ii} = vIn(ii,:);
end
end
%%
% colorm returns a colormap which is really good for creating informative
% heatmap style figures.
% No particular color stands out and it doesn't do too badly for colorblind people either.
% It works by interpolating the data from the
% 'spectral' setting on http://colorbrewer2.org/ set to 11 colors
% It is modified a little to make the brightest yellow a little less bright.
function cmap = colorm(varargin)
n = 100;
if ~isempty(varargin)
    n = varargin{1};
end
if n==1
    cmap =  [0.2005    0.5593    0.7380];
    return;
end
if n==2
     cmap =  [0.2005    0.5593    0.7380;
              0.9684    0.4799    0.2723];
          return;
end
frac=.95; % Slight modification from colorbrewer here to make the yellows in the center just a bit darker
cmapp = [158, 1, 66; 213, 62, 79; 244, 109, 67; 253, 174, 97; 254, 224, 139; 255*frac, 255*frac, 191*frac; 230, 245, 152; 171, 221, 164; 102, 194, 165; 50, 136, 189; 94, 79, 162];
x = linspace(1,n,size(cmapp,1));
xi = 1:n;
cmap = zeros(n,3);
for ii=1:3
    cmap(:,ii) = pchip(x,cmapp(:,ii),xi);
end
cmap = flipud(cmap/255);
end
function cmap = whiteFade(varargin)
n = 100;
if nargin>0
    n = varargin{1};
end
thisColor = 'blue';
if nargin>1
    thisColor = varargin{2};
end
switch thisColor
    case {'gray','grey'}
        cmapp = [255,255,255;240,240,240;217,217,217;189,189,189;150,150,150;115,115,115;82,82,82;37,37,37;0,0,0];
    case 'green'
        cmapp = [247,252,245;229,245,224;199,233,192;161,217,155;116,196,118;65,171,93;35,139,69;0,109,44;0,68,27];
    case 'blue'
        cmapp = [247,251,255;222,235,247;198,219,239;158,202,225;107,174,214;66,146,198;33,113,181;8,81,156;8,48,107];
    case 'red'
        cmapp = [255,245,240;254,224,210;252,187,161;252,146,114;251,106,74;239,59,44;203,24,29;165,15,21;103,0,13];
    otherwise
        warning(['sorry your color argument ' thisColor ' was not recognized']);
end
cmap = interpomap(n,cmapp);
end
% Eat a approximate colormap, then interpolate the rest of it up.
function cmap = interpomap(n,cmapp)
    x = linspace(1,n,size(cmapp,1));
    xi = 1:n;
    cmap = zeros(n,3);
    for ii=1:3
        cmap(:,ii) = pchip(x,cmapp(:,ii),xi);
    end
    cmap = (cmap/255); % flipud??
end

