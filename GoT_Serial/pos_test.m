%script visualizing position estimates.
clf
plot3(0,0,0)
axis([-1000 1000 -1000 1000 -100 1000]);
% plot(0,0)
% axis([-1000 1000 -1000 1000]);
hold on
delete(instrfindall);
s = serial('COM11');
fopen(s)
%fprintf(s,'*IDN?')
i=0;
while(1==1)
out = fscanf(s)
C=strsplit(out,':');
if(length(C)==3)
    C=C{2};
    C=strsplit(C,',');
    if(length(C)==3);
        x=str2num(C{1})
        y=str2num(C{2})
        z=str2num(C{3})
        plot3(x,y,z,'*')
        %plot(x,y,'*')
    end
    i=i+1;
    %pts1(:,i)=[x;y];
end

pause(0.6)
end

