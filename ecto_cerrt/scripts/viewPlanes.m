%% 
% @author Can Erdogan
% @date 2017-05-19
% @brief Visualizes the planes that are received from vision code.
%%

% Read the data
bla = load('../data/points3');
bla(1,:) = [];

% Create polygon structures
temp = find(sum(bla, 2) == 0);
lastTemp = 1;
polys = [];
for i = 1 : numel(temp)
	polys{end+1} = bla(lastTemp:(temp(i)-1), :);
	lastTemp = temp(i) + 1;
end

% Visualize the polygons
for i = 1 : numel(polys)
	plot3(polys{i}(:,1), polys{i}(:,2), polys{i}(:,3), '-o');
	for j = 1 : size(polys{i})
	%	plot3(polys{i}(j,1), polys{i}(j,2), polys{i}(j,3), '-o');
		text(polys{i}(j,1), polys{i}(j,2), polys{i}(j,3), [num2str(i-1), char('a' + j -1)]);
	end

	hold on;
end

% Visualize the normals
for i = 1 : numel(polys)
	n = bla(end-numel(polys)+i, :);
	n = n / norm(n);
	mp = mean(polys{i});
	mp2 = mp + 0.2 * n;
	plot3(mp(1), mp(2), mp(3), 'ro');
	text(mp(1), mp(2), mp(3), num2str(i-1));
	plot3([mp(1); mp2(1)], [mp(2); mp2(2)], [mp(3); mp2(3)], 'r-');
end

