
%% Exercise 1

% a
w_T_a = transl(10,0,0) * troty(90,'deg');
%b
w_T_b = troty(90, 'deg') * transl(10,0,0);

figure(1); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w', 'color' ,'r'); trplot(w_T_a, 'frame', 'A'); trplot(w_T_b, 'frame', 'B');

%% Exercise 2

% a
w_T_a = transl(10,0,0) * troty(90, 'deg');
% b
w_T_b = transl(10,0,0) * troty(90, 'deg');
figure(2); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w', 'color' ,'r'); trplot(w_T_a, 'frame', 'A'); trplot(w_T_b, 'frame', 'B');

%% Exercise 3
w_T_a = trotz(45 ,'deg') * troty(0,'deg') * trotz(15,'deg');
w_T_b = trotz(60 ,'deg') * troty(0,'deg') * trotz(0,'deg');
figure(3); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w', 'color' ,'r'); trplot(w_T_a, 'frame', 'A', 'color', 'b'); trplot(w_T_b, 'frame', 'B', 'color', 'g');
% w_T_a == w_T_b
%sin(a+b) = sin(a)cos(b) + cos(a)sin(b)
%cos(a+b) = cos(a)cos(b) - sin(a)sin(b) ---> a=45, b=15
rpy_a = tr2rpy(w_T_a);
rpy_b = tr2rpy(w_T_b);

%% Exercise 4

% a
w_T_a = troty(90,'deg') * transl(0,0,-3) * transl(2,0,0);
%b
w_T_b = transl(0,0,-3) * transl(2,0,0) * troty(90, 'deg');
figure(4); clf; grid; hold on; 
trplot(eye(4), 'frame', 'w', 'color' ,'r'); trplot(w_T_a, 'frame', 'A'); trplot(w_T_b, 'frame', 'B');