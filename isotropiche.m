
% MATLAB Code from Sensor Array Analyzer App

% Assign Frequencies and Propagation Speed
Frequency = 300000000;
PropagationSpeed = 300000000;
%% puntamento array lineare antenne isotropiche
% Create a Uniform Linear Array Object
Frequency = 300000000;
PropagationSpeed = 300000000;
Array = phased.ULA('NumElements',6,...
'ArrayAxis','x');
% The multiplication factor for lambda units to meter conversion
Array.ElementSpacing = 0.5*1;
Array.Taper = ones(1,6).';

% Create an isotropic antenna element
Elem = phased.IsotropicAntennaElement;
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
% Assign Steering Angles
angle=[0 45 90];
close all 
for i=1:numel(angle)

SteeringAngles = [angle(i);0];
% Assign Phase shift quantization bits
PhaseShiftBits = 0;

% Create Figure

% Plot Array Geometry
% figure;
% viewArray(Array,'ShowNormal',false,...
%   'ShowTaper',false,'ShowIndex','None');

% Calculate Steering Weights

Freq3D = 300000000;
% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
SteerVector = phased.SteeringVector('SensorArray', Array,...
 'PropagationSpeed', PropagationSpeed, 'NumPhaseShifterBits', PhaseShiftBits(1));
for idx = 1:length(Frequency)
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% Plot 3d graph
format = 'polar';
%figure;
subfigure(3,3,i);
pattern(Array, Freq3D , 'PropagationSpeed', PropagationSpeed,...
 'Type','directivity', 'CoordinateSystem', format,'weights', w(:,1));

% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
for idx = 1:length(Frequency)
    SteerVector = phased.SteeringVector('SensorArray', Array,...
      'PropagationSpeed', PropagationSpeed, ...
      'NumPhaseShifterBits', PhaseShiftBits(idx));
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% Plot 2d azimuth graph
format = 'polar';
cutAngle = 0;
%figure;
subfigure(3,3,i+3);
pattern(Array, Frequency, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
for idx = 1:length(Frequency)
    SteerVector = phased.SteeringVector('SensorArray', Array,...
       'PropagationSpeed', PropagationSpeed,...
       'NumPhaseShifterBits', PhaseShiftBits(idx));
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% Plot 2d elevation graph
format = 'polar';
cutAngle = 0;
%figure;
subfigure(3,3,i+6);
pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% % Plot Grating Lobe Diagram
% figure;
% plotGratingLobeDiagram(Array,Frequency(1),SteeringAngles(:,1),PropagationSpeed);
pause()
end

%% elementi array lineare antenne isotropiche
% Create a Uniform Linear Array Object
close all
Array = phased.ULA('NumElements',6,...
'ArrayAxis','x');
% The multiplication factor for lambda units to meter conversion
Array.ElementSpacing = 0.5*1;
Array.Taper = ones(1,6).';

% Create an isotropic antenna element
Elem = phased.IsotropicAntennaElement;
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
% Assign Steering Angles
SteeringAngles = [0;0];
% Assign Phase shift quantization bits
PhaseShiftBits = 0;

close all 
elementi=[4 8 16 20];
for i=1:numel(elementi)
Array = phased.ULA('NumElements',elementi(i),...
'ArrayAxis','x');


% Create Figure

% Plot Array Geometry
% figure;
% viewArray(Array,'ShowNormal',false,...
%   'ShowTaper',false,'ShowIndex','None');

% Calculate Steering Weights

Freq3D = 300000000;
% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
SteerVector = phased.SteeringVector('SensorArray', Array,...
 'PropagationSpeed', PropagationSpeed, 'NumPhaseShifterBits', PhaseShiftBits(1));
for idx = 1:length(Frequency)
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% Plot 3d graph
format = 'polar';
%figure;
subfigure(2,4,i);
pattern(Array, Freq3D , 'PropagationSpeed', PropagationSpeed,...
 'Type','directivity', 'CoordinateSystem', format,'weights', w(:,1));

% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
for idx = 1:length(Frequency)
    SteerVector = phased.SteeringVector('SensorArray', Array,...
      'PropagationSpeed', PropagationSpeed, ...
      'NumPhaseShifterBits', PhaseShiftBits(idx));
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% Plot 2d azimuth graph
format = 'polar';
cutAngle = 0;
%figure;
subfigure(2,4,i+4);
pattern(Array, Frequency, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
for idx = 1:length(Frequency)
    SteerVector = phased.SteeringVector('SensorArray', Array,...
       'PropagationSpeed', PropagationSpeed,...
       'NumPhaseShifterBits', PhaseShiftBits(idx));
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% % Plot 2d elevation graph
% format = 'polar';
% cutAngle = 0;
% %figure;
% subfigure(3,3,i+6);
% pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
%  'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% % Plot Grating Lobe Diagram
% figure;
% plotGratingLobeDiagram(Array,Frequency(1),SteeringAngles(:,1),PropagationSpeed);
pause()
end

%% grating lobe e spacing array lineare antenne isotropiche
% Create a Uniform Linear Array Object
Array = phased.ULA('NumElements',6,...
'ArrayAxis','x');
% The multiplication factor for lambda units to meter conversion

Array.Taper = ones(1,6).';

% Create an isotropic antenna element
Elem = phased.IsotropicAntennaElement;
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
% Assign Steering Angles
SteeringAngles = [0;0];
% Assign Phase shift quantization bits
PhaseShiftBits = 0;

close all 
spacing=[0.2 0.5 0.7 1 3];
for i=1:numel(spacing)
Array.ElementSpacing = spacing(i)*1;


% Create Figure

% Plot Array Geometry
% figure;
% viewArray(Array,'ShowNormal',false,...
%   'ShowTaper',false,'ShowIndex','None');

% Calculate Steering Weights

Freq3D = 300000000;
% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
SteerVector = phased.SteeringVector('SensorArray', Array,...
 'PropagationSpeed', PropagationSpeed, 'NumPhaseShifterBits', PhaseShiftBits(1));
for idx = 1:length(Frequency)
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% Plot 3d graph
format = 'polar';
%figure;
subfigure(3,5,i);
pattern(Array, Freq3D , 'PropagationSpeed', PropagationSpeed,...
 'Type','directivity', 'CoordinateSystem', format,'weights', w(:,1));

% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
for idx = 1:length(Frequency)
    SteerVector = phased.SteeringVector('SensorArray', Array,...
      'PropagationSpeed', PropagationSpeed, ...
      'NumPhaseShifterBits', PhaseShiftBits(idx));
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

% Plot 2d azimuth graph
format = 'polar';
cutAngle = 0;
%figure;
subfigure(3,5,i+5);
pattern(Array, Frequency, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% Find the weights
w = zeros(getNumElements(Array), length(Frequency));
for idx = 1:length(Frequency)
    SteerVector = phased.SteeringVector('SensorArray', Array,...
       'PropagationSpeed', PropagationSpeed,...
       'NumPhaseShifterBits', PhaseShiftBits(idx));
    w(:, idx) = step(SteerVector, Frequency(idx), SteeringAngles(:, idx));
end

subfigure(3,5,i+10);

plotGratingLobeDiagram(Array,Frequency(1),SteeringAngles(:,1),PropagationSpeed);
pause()
end

