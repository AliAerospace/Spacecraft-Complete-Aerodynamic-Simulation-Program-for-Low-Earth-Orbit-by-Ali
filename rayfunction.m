%rayfunction.m
function intersect = rayfunction(O, D, V1, V2, V3)

%Möller–Trumbore algorithm

%Initialise the outputs
intersect = false;

%Floating point precision
fp = 1e-6;

%Compute the edges of the triangle
e1 = V2 - V1;
e2 = V3 - V1;

%Compute the determinant
H = cross(D, e2);
Delta = dot(e1, H);

%If determinant is near zero, the ray is parallel to the triangle
if abs(Delta) < fp
    return;
end

S = O - V1;
u = dot(S, H)/Delta;

%Check if intersection is outside the triangle
if u < 0.0 || u > 1.0
    return; %No intersection
end

Q = cross(S, e1);
v = dot(D, Q)/Delta;

%Check if intersection is outside the triangle
if v < 0.0 || (u + v) > 1.0
    return; %No intersection
end

t = dot(e2, Q)/Delta;

%Check if intersection occurs in the positive ray direction
if t > fp
    intersect = true;
end
end
