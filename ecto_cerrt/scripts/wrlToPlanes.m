% Requires bla.wrl to only have box translations and dimensions, and copy initial trnaslation 
% to t0 below
% NOTE It does not include the "back" faces of the boxes (l25)

f = fopen('bla.xml', 'w');
fprintf(f, '<main>\n');
fprintf(f, '\t<planes>\n');

t0 = [0.84999996 0 -0.69999998];

clf;
data = load('bla.wrl');
counter = 0;
for i = 1 : 8

	% Center of the box
	t = data(2*i-1,:) + t0;
	dims = data(2*i,:) / 2;

	dirs = [[1 0 0]; [-1 0 0]; [0 1 0]; [0 -1 0]; [0 0 1]; [0 0 -1]];
	dirs2 = [[0 1 0]; [0 -1 0]; [0 0 1]; [0 0 -1]; [1 0 0]; [-1 0 0]];
	for j = 1 : 6

		% Remove the back faces
		if(j == 1), continue; end;

		fprintf(f, '\t\t<plane%d>\n', counter);
		fprintf(f, '\t\t\t<id> %d </id>\n', counter);

		% Compute the center of the face
		dir1 = dirs(j,:);
		dimIdx = ceil(j/2.0);
		tc = t + dir1 * dims(dimIdx);

		% Compute the four points
		dir2 = dirs2(j,:);
		dir3 = cross(dir1, dir2);
		dimIdx2 = dimIdx - 2;
		if(dimIdx2 < 1), dimIdx2 = dimIdx2 + 3; end;
		dimIdx3 = dimIdx - 1;
		if(dimIdx3 < 1), dimIdx3 = dimIdx3 + 3; end;
		p0 = tc + dir2 * dims(dimIdx2) + dir3 * dims(dimIdx3);
		p1 = tc - dir2 * dims(dimIdx2) + dir3 * dims(dimIdx3);
		p2 = tc - dir2 * dims(dimIdx2) - dir3 * dims(dimIdx3);
		p3 = tc + dir2 * dims(dimIdx2) - dir3 * dims(dimIdx3);
		ps = [p0; p1; p2; p3; p0];
		plot3(ps(:,1), ps(:,2), ps(:,3), '-o'); hold on;
		plot3(tc(1), tc(2), tc(3), 'ro'); hold on;

		% Visualize plane normal
		pn = tc + dir1 * 0.1;
		temp = [tc; pn];
		plot3(temp(:,1), temp(:,2), temp(:,3), 'ko-', 'linewidth', 2); hold on;
		
		% Compute the offset 
		offset = -dot(tc, dir1);
		fprintf(f, '\t\t\t<params> %.3f %.3f %.3f %f </params>\n', dir1(1), dir1(2), dir1(3), offset);

		% Print the center of the polygon
		fprintf(f, '\t\t\t<mean_point_world> %f %f %f </mean_point_world>\n', tc(1), tc(2), tc(3));

		% Print the points in the local frame
		fprintf(f, '\t\t\t<poly_points_local>\n');
		vs = ps - tc;
		for v = 1 : 4
			fprintf(f, '\t\t\t\t<v%d> %f %f %f </v%d>\n', v-1, vs(v, 1), vs(v, 2), vs(v, 3), v-1); 
		end
		fprintf(f, '\t\t\t</poly_points_local>\n');
		
		fprintf(f, '\t\t</plane%d>\n', counter);
		

		counter = counter+1;
	end
	
end

fprintf(f, '\t</planes>\n');
fprintf(f, '</main>\n');
axis equal
fclose(f);
