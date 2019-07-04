function string = latex_pretify(eq)
%Pretify equagtions for LateX
%   Author: Taeke de Haan
%   Date: 27-03-2017
string = latex(eq);
string = strrep(string,'alphadd','\ddot{\alpha}');
string = strrep(string,'alphad','\dot{\alpha}');

string = strrep(string,'betadd','\ddot{beta}');
string = strrep(string,'betad','\dot{beta}');
string = strrep(string,'beta','\beta');

string = strrep(string,'gamma','\gamma');
end

