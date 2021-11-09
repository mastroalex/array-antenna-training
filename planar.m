% Create a uniform rectangular array
% cosine antenna
%%
close all
vec=[4 10 24];
indice=[1 2 3 4; 5 6 7 8;9 10 11 12];
for i=1:numel(vec)
Array = phased.URA('Size',[vec(i) 4],...
'Lattice','Rectangular','ArrayNormal','x');
Array.ElementSpacing = [0.5 0.5];
% Calculate Row taper
rwind = ones(1,4).';
% Calculate Column taper
cwind = ones(1,vec(i)).';
% Calculate taper
taper = rwind*cwind.';
Array.Taper = taper.';

% Create a cosine antenna element
Elem = phased.CosineAntennaElement;
Elem.CosinePower = [1 1];
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
% Assign Frequencies and Propagation Speed
Frequency = 300000000;
PropagationSpeed = 300000000;

% Assign Steering Angles
SteeringAngles = [0;0];

% Create Figure
subfigure(3,4,indice(i,1));
% Plot Array Geometry
viewArray(Array,'ShowNormal',false,...
  'ShowTaper',false,'ShowIndex','None');

% Calculate Steering Weights

Freq3D = 300000000;
% Find the weights
w = ones(getNumElements(Array), length(Frequency));
subfigure(3,4,indice(i,2));
% Plot 3d graph
format = 'polar';

pattern(Array, Freq3D , 'PropagationSpeed', PropagationSpeed,...
 'Type','directivity', 'CoordinateSystem', format,'weights', w(:,1));

% Find the weights
w = ones(getNumElements(Array), length(Frequency));
subfigure(3,4,indice(i,3));
% Plot 2d azimuth graph
format = 'polar';
cutAngle = 0;
pattern(Array, Frequency, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% Find the weights
w = ones(getNumElements(Array), length(Frequency));
subfigure(3,4,indice(i,4));
% Plot 2d elevation graph
format = 'polar';
cutAngle = 0;
pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
    
pause(0.2)

end

%%
close all
vec=[4 10 16];
indice=[1 2 3 4; 5 6 7 8;9 10 11 12];
for i=1:numel(vec)
Array = phased.URA('Size',[vec(i) vec(i)],...
'Lattice','Rectangular','ArrayNormal','x');
Array.ElementSpacing = [0.5 0.5];
% Calculate Row taper
rwind = ones(1,vec(i)).';
% Calculate Column taper
cwind = ones(1,vec(i)).';
% Calculate taper
taper = rwind*cwind.';
Array.Taper = taper.';

% Create a cosine antenna element
Elem = phased.CosineAntennaElement;
Elem.CosinePower = [1 1];
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
% Assign Frequencies and Propagation Speed
Frequency = 300000000;
PropagationSpeed = 300000000;

% Assign Steering Angles
SteeringAngles = [0;0];

% Create Figure
subfigure(3,4,indice(i,1));
% Plot Array Geometry
viewArray(Array,'ShowNormal',false,...
  'ShowTaper',false,'ShowIndex','None');

% Calculate Steering Weights

Freq3D = 300000000;
% Find the weights
w = ones(getNumElements(Array), length(Frequency));
subfigure(3,4,indice(i,2));
% Plot 3d graph
format = 'polar';

pattern(Array, Freq3D , 'PropagationSpeed', PropagationSpeed,...
 'Type','directivity', 'CoordinateSystem', format,'weights', w(:,1));

% Find the weights
w = ones(getNumElements(Array), length(Frequency));
subfigure(3,4,indice(i,3));
% Plot 2d azimuth graph
format = 'polar';
cutAngle = 0;
pattern(Array, Frequency, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);

% Find the weights
w = ones(getNumElements(Array), length(Frequency));
subfigure(3,4,indice(i,4));
% Plot 2d elevation graph
format = 'polar';
cutAngle = 0;
pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
    
pause(0.2)

end
%%


close all
angle=[0 0; 45 45; 45 -45];
indice=[1 2 3 4; 5 6 7 8;9 10 11 12];
cut=[0 45 -45];
for i=1:numel(angle(:,1))
   % Create a uniform rectangular array
Array = phased.URA('Size',[10 10],...
'Lattice','Rectangular','ArrayNormal','x');
Array.ElementSpacing = [0.5 0.5];
% Calculate Row taper
rwind = hamming(10);
% Calculate Column taper
cwind = hamming(10);
% Calculate taper
taper = rwind*cwind.';
Array.Taper = taper.';

% Create a cosine antenna element
Elem = phased.CosineAntennaElement;
Elem.CosinePower = [1 1];
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
% Assign Frequencies and Propagation Speed
Frequency = 300000000;
PropagationSpeed = 300000000;

% Assign Steering Angles
SteeringAngles = angle(i,:)';
% Assign Phase shift quantization bits
PhaseShiftBits = 0;

% Create Figure
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
subfigure(3,4,indice(i,1));
format = 'polar';
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
subfigure(3,4,indice(i,2));

% Plot 2d azimuth graph
format = 'polar';
cutAngle = cut(i);
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
subfigure(3,4,indice(i,3));
format = 'polar';
cutAngle = 0;
pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
subfigure(3,4,indice(i,4));
% Plot Grating Lobe Diagram

plotGratingLobeDiagram(Array,Frequency(1),SteeringAngles(:,1),PropagationSpeed);


pause(0.2)
end

%%
close all
angle=[-60 45; -30 45; 30 45; 70 45; 70 0; 20 0; -20 0; -60 0; -60 -45; 0 -45; 30 -45; 70 -45;];
figure()
for i=1:numel(angle(:,1))
   % Create a uniform rectangular array
Array = phased.URA('Size',[14 14],...
'Lattice','Rectangular','ArrayNormal','x');
Array.ElementSpacing = [0.4 0.4];
% Calculate Row taper
rwind = hamming(14);
% Calculate Column taper
cwind = hamming(14);
% Calculate taper
taper = rwind*cwind.';
Array.Taper = taper.';

% Create a cosine antenna element
Elem = phased.CosineAntennaElement;
Elem.CosinePower = [1 1];
Elem.FrequencyRange = [0 300000000];
Array.Element = Elem;
% Assign Frequencies and Propagation Speed
Frequency = 300000000;
PropagationSpeed = 300000000;

% Assign Steering Angles
SteeringAngles = angle(i,:)';
% Assign Phase shift quantization bits
PhaseShiftBits = 0;

% Create Figure
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
pattern(Array, Freq3D , 'PropagationSpeed', PropagationSpeed,...
 'Type','directivity', 'CoordinateSystem', format,'weights', w(:,1));

pause(0.01)
end
