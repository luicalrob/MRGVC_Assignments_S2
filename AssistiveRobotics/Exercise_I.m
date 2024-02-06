
%% Exercise 1
% a
w_T_a = transl(10,0,0) * troty(90,'deg');
figure(1); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w'); trplot(w_T_a, 'frame', 'A');
% b
w_T_b = troty(90, 'deg') * transl(10,0,0);
figure(2); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w'); trplot(w_T_b, 'frame', 'B');

pause;
%% Exercise 2

% a
w_T_a = transl(10,0,0) * troty(90,'deg');
figure(1); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w'); trplot(w_T_a, 'frame', 'A');
% b
w_T_b = transl(10,0,0) * troty(90, 'deg');
figure(2); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w'); trplot(w_T_b, 'frame', 'B');

pause;