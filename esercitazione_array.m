%% Esercitazione sugli array
% WET A.A. 2021-2022
% Fallani Rebecca
% Mastrofini Alessandro
% Muscedere Erika
% code: https://bit.ly/3bWVldJ
% ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% Per il codice sono richiesti il toolobox 'subfigure' 
% e il Phased Array System Toolbox

%% test ed esempi iniziali
f=3e8 %3 MHz
lambda = physconst('LightSpeed')/f
%%
sensorArrayAnalyzer %avviare l'app
%%
Array=ula %test array properties
spacing=Array.ElementSpacing
L=(Array.NumElements+1)*spacing
span=Array.NumElements*spacing
%%
testelemnt=64;
wvtool(hamming(testelemnt)); %plot hamming window
wvtool(chebwin(testelemnt)); %plot chebyshev 

%%
Array.Taper' %vector with taper coefficients

%% example plot
% Create an isotropic antenna element
close all
% Create a Uniform Linear Array Object
Array = phased.ULA('NumElements',6,...
'ArrayAxis','x');
Array.ElementSpacing = 0.5;
Array.Taper = ones(1,6).';
Elem = phased.IsotropicAntennaElement;
Elem.FrequencyRange = [0 f];
Array.Element = Elem;
SteeringAngles = [0;0];
% Assign Phase shift quantization bits
PhaseShiftBits = 0;
PropagationSpeed=physconst('LightSpeed');
% Plot Array Geometry
subfigure(2,2,3)
viewArray(Array,'ShowIndex','None','Title','Geometria array');
w = ones(getNumElements(Array), length(f));
% Plot 3d graph
format = 'polar';
subfigure(2,2,4);
pattern(Array, f , 'PropagationSpeed', PropagationSpeed,'Type','directivity', 'CoordinateSystem', format,'weights', w(:,1));

%% example plot
close all
subfigure(2,2,3)
cutAngle = 0;
pattern(Array, f, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
subfigure(2,2,4);
pattern(Array, f, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
%pattern(object,frequency,azimuth,elevation)

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
subfigure(3,3,i+6);
pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
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
%% puntamento array lineare antenne cosine
% Assign Frequencies and Propagation Speed
close all
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
subfigure(3,4,i+8);
pattern(Array, Frequency, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
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
subfigure(3,5,i+5);
pattern(Array, Frequency, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
subfigure(3,5,i+10);
plotGratingLobeDiagram(Array,Frequency(1),SteeringAngles(:,1),PropagationSpeed);
pause(0.2)

end


%% 2D antennas
% Create a uniform rectangular array
% cosine antenna
%% create 2D antenna
% creiamo antenna 2D variando il numero di elementi per riga
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

%% variazione elementi
% antenna 2D variando il numero di elementi 
% sia sulle righe che sulle colonne
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
% variazione dell'angolo di puntamento

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

%% simulazione scanner elettronico
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



