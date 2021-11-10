# Antenna Array Training

During the lessons of Wireless Electromagnetics Technologies I prepared an exercise on antenna arrays in Matlab.

[📡 more info..](https://alessandromastrofini.it/array-esercitazione-matlab)

<br>

```matlab
sensorArrayAnalyzer
```

<img src="https://alessandromastrofini.it/wp-content/uploads/2021/11/Senza-nome-2-1536x1155.png" alt="pattern" style="width:500px;"/>
 
```matlab
% Phased Array System Toolbox
f=3e8 %3 MHz
lambda = physconst('LightSpeed')/f
%%
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
%%
close all
subfigure(2,2,3)
cutAngle = 0;
pattern(Array, f, cutAngle, -90:90, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
subfigure(2,2,4);
pattern(Array, f, -180:180, cutAngle, 'PropagationSpeed', PropagationSpeed,...
 'Type', 'directivity', 'CoordinateSystem', format ,'weights', w);
%pattern(object,frequency,azimuth,elevation)
```
