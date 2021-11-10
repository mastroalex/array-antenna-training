# Antenna Array Training

During the lessons of Wireless Electromagnetics Technologies I prepared an exercise on antenna arrays in Matlab.

[📡 more info..](https://alessandromastrofini.it/array-esercitazione-matlab)

<br>


<img src="(https://alessandromastrofini.it/wp-content/uploads/2021/11/Senza-nome-2-1536x1155.png" alt="pattern" style="width:500px;"/>
 
```matlab
f=3e8 %3 MHz
lambda = physconst('LightSpeed')/f
array=ula;
L=(array.NumElements+1)*array.ElementSpacing
% to visualize windows
testelemnt=64;
wvtool(hamming(testelemnt)); %plot hamming window
wvtool(chebwin(testelemnt)); %plot chebyshev 
ula.Taper %vector with taper coefficients
```
