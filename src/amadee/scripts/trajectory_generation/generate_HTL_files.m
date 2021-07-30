close all;
clear all;

%% CONFIG

height = 5;   % [m] Maximum height
iters = 30;   % [1] Iterations to perform HTLs
radius = 2;   % [m] distance to cover with flight

%% WP GENERATION

att = rand(1,iters) * 2*pi;
wps = [radius*cos(att); radius*sin(att); height*ones(1,iters)];

%% CSV GENERATION

csvHeader = "x,y,z,yaw,holdtime";
for k=1:iters
  wp_mat = [0, 0, height, 0, 3;
    wps(:,k)', 0, 3;
    0, 0, height, 0, 3];
  
  fname = sprintf("out/T5_%02d.csv", k);
  fid = fopen(fname, 'w');
  fprintf(fid,"%s\n",csvHeader);
  fclose(fid);
  
  dlmwrite(fname,wp_mat,'-append');
end