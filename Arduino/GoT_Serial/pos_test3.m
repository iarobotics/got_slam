%script visualizing position estimates.
clf
 %plot3(0,0,0)
 %axis([0 12000 -6000 6000 -100 7000]);
plot(0,0)
axis([-1000 12000 -6000 6000]);
%hold on


PosList=[[42929,1645,-462,4000]',[42498,1591,4193,3999]',[42867,6195,0,5499]',[42928,10157,4204,5499]',[42497,10568,-2389,5499]'];
%plot3(PosList(2,:),PosList(3,:),PosList(4,:),'*')
plot(PosList(2,:),PosList(3,:),'*')
%axis([0 12000 -6000 6000 ]);
%axis([0 1000 0 20000]);
% plot(0,0)
% axis([-1000 1000 -1000 1000]);

delete(instrfindall);
s = serial('COM6');
fopen(s)
%fprintf(s,'*IDN?')
i=1;
while(1==1)

out = fscanf(s)
C=strsplit(out,':')
if(length(C)==3)
C=strsplit(C{2},',')
    if(length(C)==6)
        x=str2num(C{4})
        y=str2num(C{5})
        z=str2num(C{6})
        plot(x,y,'.b')
        %plot(i,str2num(C{4}),'.');
        data(:,i)=[str2num(C{1}) str2num(C{2}) str2num(C{3}) str2num(C{4}) str2num(C{5})]';
      
    end
    i=i+1;
    %pts1(:,i)=[x;y];
plot(PosList(2,:),PosList(3,:),'*',x,y,'ob')
axis([0 12000 -6000 6000 ]);

pause(0.001)
end
end
% pause
% clf
% plot(1:1000,data(5,:),'.')

