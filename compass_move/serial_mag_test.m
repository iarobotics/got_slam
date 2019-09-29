%serial magnetometer test
delete(instrfindall);

s = serial('COM7');
fopen(s);
figure(1)
axis([-1 1 -1 1])
hold on

while(1==1)
    a=fscanf(s);
    pause(0.1);
    C=strsplit(a,',');
    if(length(C)==2)
      C2=strsplit(C{2},char(13));
      t1=str2num(C{1})
      t2=str2num(C2{1})
      plot(t1,t2,'.')
    end
end

