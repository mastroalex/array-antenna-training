%% puntamento array lineare antenne cosine
% MATLAB Code from Sensor Array Analyzer App

% Assign Frequencies and Propagation Speed
Frequency = 300000000;
PropagationSpeed = 300000000;

Array = phased.ULA('NumElements',6,...
'ArrayAxis','y');
% The multiplication factor for lambda units to meter conversion
Array.ElementSpacing = 0.5*1;
Array.Taper = ones(1,6).';

% Create a cosine antenna element
Elem = phased.CosineAntennaElement;
Elem.CosinePower = [1 1];
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;

% Assign Steering Angles
angle=[0 30 60 90];
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
subfigure(3,4,i);
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
subfigure(3,4,i+4);
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
subfigure(3,4,i+8);
pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% % Plot Grating Lobe Diagram
% figure;
% plotGratingLobeDiagram(Array,Frequency(1),SteeringAngles(:,1),PropagationSpeed);
pause(0.2)
end

%% elementi array lineare antenne cosine
Frequency = 300000000;
PropagationSpeed = 300000000;

% Assign Steering Angles
elementi=[6 8 16 22];
close all 
for i=1:numel(elementi)
Array = phased.ULA('NumElements',elementi(i),...
'ArrayAxis','y');
% The multiplication factor for lambda units to meter conversion
Array.ElementSpacing = 0.5*1;
Array.Taper = ones(1,elementi(i)).';

% Create a cosine antenna element
Elem = phased.CosineAntennaElement;
Elem.CosinePower = [1 1];
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
SteeringAngles = [30;0];
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


end
%% grating lobe e spacing array lineare antenne cosine

Frequency = 300000000;
PropagationSpeed = 300000000;

% Assign Steering Angles
spacing=[0.2 0.5 0.7 1 3];
close all 
for i=1:numel(spacing)
Array = phased.ULA('NumElements',8,...
'ArrayAxis','y');
% The multiplication factor for lambda units to meter conversion
Array.ElementSpacing = spacing(i)*1;
Array.Taper = ones(1,8).';

% Create a cosine antenna element
Elem = phased.CosineAntennaElement;
Elem.CosinePower = [1 1];
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
SteeringAngles = [30;0];
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
subfigure(3,5,i+10);
plotGratingLobeDiagram(Array,Frequency(1),SteeringAngles(:,1),PropagationSpeed);
pause(0.2)

end


