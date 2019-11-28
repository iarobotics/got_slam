%script visualizing position estimates.
% clf
% plot3(0,0,0)
% axis([-1000 1000 -1000 1000 -100 1000]);
% plot(0,0)
% axis([-1000 1000 -1000 1000]);
hold on
delete(instrfindall);
s = serial('COM11');
fopen(s)
%fprintf(s,'*IDN?')
i=1;
while(i<500)
out = fscanf(s)
C=strsplit(out,':');
%if(length(C)==3)
    C=C{2};
    C=strsplit(C,',');
    for j=1:length(C)
        pts(i,j)=str2num(C{j});
    end
    %if(length(C)==3)
        
        
    i=i+1
%end

pause(0.1)
end
 clf
% [H I]=find(pts(:,5)==0);
% plot(pts(H,7),pts(H,1:6))
% pause
% [H I]=find(pts(:,5)==1);
% plot(pts(H,7),pts(H,1:6))
% pause
% [H I]=find(pts(:,5)==2);
% plot(pts(H,7),pts(H,1:6)) 
% pause
% [H I]=find(pts(:,5)==3);
% plot(pts(H,7),pts(H,1:6))
% pause
% [H I]=find(pts(:,5)==4);
% plot(pts(H,7),pts(H,1:6))
plot(pts(:,7),pts(:,2:3))
hold on
plot(pts(:,7),pts(:,5)*1000,'*')
pause
% clf
% plot(pts(:,7),pts(:,1))
% hold on
% plot(pts(:,7),pts(:,5)*1000,'*')
% pause
clf
[H I]=find(pts(:,5)==0);
plot(pts(H,7),pts(H,1),'b.')
hold on
[H I]=find(pts(:,5)==1);
plot(pts(H,7),pts(H,1),'r.')
hold on
[H I]=find(pts(:,5)==2);
plot(pts(H,7),pts(H,1),'g.')
hold on
[H I]=find(pts(:,5)==3);
plot(pts(H,7),pts(H,1),'b*')
hold on
[H I]=find(pts(:,5)==4);
plot(pts(H,7),pts(H,1),'r*')
hold on




