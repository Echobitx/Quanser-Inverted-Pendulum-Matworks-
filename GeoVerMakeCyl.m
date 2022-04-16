function VertexData = GeoVerMakeCyl(Location,Orientation,Radius,Height)

r = Location;
R = Orientation;
n_side = 20;

% Vertices
% n_side = SideCount;

for i_ver=1:n_side
    VertexData_0(i_ver,:) = [Radius*cos(2*pi/n_side*i_ver),0, Radius*sin(2*pi/n_side*i_ver)];
    VertexData_0(n_side+i_ver,:) = [Radius*cos(2*pi/n_side*i_ver),Height,Radius*sin(2*pi/n_side*i_ver)];
end

n_ver = 2*n_side;

for i_ver=1:n_ver
    VertexData(i_ver,:) = r + VertexData_0(i_ver,:)*R';
end