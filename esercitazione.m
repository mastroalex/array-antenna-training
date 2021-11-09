f=3e8 %3 MHz
lambda = physconst('LightSpeed')/f
array=ula;
L=(array.NumElements+1)*array.ElementSpacing
testelemnt=64;
wvtool(hamming(testelemnt)); %plot hamming window
wvtool(chebwin(testelemnt)); %plot chebyshev 
ula.Taper %vector with taper coefficients

