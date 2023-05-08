function [X,Y] = DrawEllipse(a,b,c,d,f,g,Center)

%%DrawEllipse: Draw the ellipse from the general equation  
%              a x^2 + 2 b xy + c y^2 + 2 d x + 2 f y + g = 0
%
%
% Input arguments
%    a,b,c,d,e,f,g  : the ellipse equation parameters
%    
% Output arguments:
%     X,Y           : points to draw  
%                     e.g.:  [p(:,1),p(:,2)] = DrawEllipse(a,b,c,d,f,g);
%                            plot(p(:,1), p(:,2), '.-')
%                            axis equal
% ----------------------------------------------------------------------
% File.....: DrawEllipse.m
% Date.....: 29-FEB-2012
% Modified.: dd-MMM-yyyy, by X. Surname
% Author...: Gabriele Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------

%% Calculate parameters for drawing
aux   = b^2-a*c;
x_0   = Center(1); %(c*d - b*f)/(aux); % Center, x-coordinate
y_0   = Center(2); %(a*f - b*d)/(aux); % Center, y-coordinate
Semi1 =  sqrt(2*(a*f^2+c*d^2+g*b^2-2*b*d*f-a*c*g)/(aux*(sqrt((a-c)^2+4*b^2)-(a+c))));   % First semi-major axis
Semi2 =  sqrt(2*(a*f^2+c*d^2+g*b^2-2*b*d*f-a*c*g)/(aux*(-sqrt((a-c)^2+4*b^2)-(a+c))));  % Second semi-major axis
% Rotation angle
if b==0 && a <= c
    phi = 0;
elseif b==0 && a > c
    phi = pi/2;
elseif b~=0 && a <= c
    phi = 0.5*acot((a-c)/(2*b));
elseif b~=0 && a > c
    phi = pi/2 + 0.5*acot((a-c)/(2*b));    
end


% Draw ellipse
steps = 360;
beta = -phi;
sinbeta = sin(beta);
cosbeta = cos(beta);

alpha = linspace(0, 360, steps)' .* (pi / 180);
sinalpha = sin(alpha);
cosalpha = cos(alpha);

X = x_0 + (Semi1 * cosalpha * cosbeta - Semi2 * sinalpha * sinbeta);
Y = y_0 + (Semi1 * cosalpha * sinbeta + Semi2 * sinalpha * cosbeta);

%plot(X, Y, '.-'), axis equal
    
    