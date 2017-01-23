
FILENAME='result-4lane.csv';
SELF_RATIO=[0:0.02:0.98];
AVG_CAR_PER_SEC=[2:0.2:4.8];
PER_PLOT_DATA_VOLUME=length(SELF_RATIO)*length(AVG_CAR_PER_SEC);
%constants

firstRowInfo=csvread(FILENAME,1,0,[1,0,1,7]);
LANE_NUM=firstRowInfo(1);
%get the first row info.To set lane information

dataMatrix=csvread(FILENAME,1,0);
dataMatrixSize=size(dataMatrix);
dataMatrixHeight=dataMatrixSize(1);
rowNumOfPlot=dataMatrixHeight/PER_PLOT_DATA_VOLUME;
%import whole data into dataMatrix,get its height

%Begin plotting.In each row, the left is total QNum,right is separated.
%Green for self-driving car, Red for human car.

for k=1:rowNumOfPlot
    
    figure();
  % subplot(rowNumOfPlot,2,2*k-1);
   vec=dataMatrix(PER_PLOT_DATA_VOLUME*(k-1)+1:PER_PLOT_DATA_VOLUME*k,8)+dataMatrix(PER_PLOT_DATA_VOLUME*(k-1)+1:PER_PLOT_DATA_VOLUME*k,8);
   plotMatrix=vec2mat(vec,length(AVG_CAR_PER_SEC));
   x=SELF_RATIO;
   y=AVG_CAR_PER_SEC;
   [X,Y]=meshgrid(x,y);
   Z=plotMatrix';
   map=[1,0,0;0,1,0];
   handle=surf(X,Y,Z);
   xlabel('Percentage of autocars');
   ylabel('Avg num of cars per sec');
   zlabel('queueLength');
   str=sprintf('Total queue length:\n(%i lanes in total. %i lane(s) dedicated to autocars)',LANE_NUM,k-1);
   title(str);
   str=sprintf('L=%i,Total QL,%i lane(s) dedicated to autocars.jpg',LANE_NUM,k-1);
   saveas(handle,str);
   
   handle1=figure();
   %subplot(rowNumOfPlot,2,2*k);
   vec=dataMatrix(PER_PLOT_DATA_VOLUME*(k-1)+1:PER_PLOT_DATA_VOLUME*k,8);
   plotMatrix=vec2mat(vec,length(AVG_CAR_PER_SEC));
   x=SELF_RATIO;
   y=AVG_CAR_PER_SEC;
   [X,Y]=meshgrid(x,y);
   Z1=plotMatrix';
   s=size(Z1);
   c=ones(s);
   map=[1,0,0;0,1,0];
   surf(X,Y,Z1,c);
   xlabel('Percentage of autocars');
   ylabel('Avg num of cars per sec');
   zlabel('queueLength');


   hold on;

   vec=dataMatrix(PER_PLOT_DATA_VOLUME*(k-1)+1:PER_PLOT_DATA_VOLUME*k,7);
   plotMatrix=vec2mat(vec,length(AVG_CAR_PER_SEC));
   x=SELF_RATIO;
   y=AVG_CAR_PER_SEC;
   [X,Y]=meshgrid(x,y);
   Z2=plotMatrix';
   s=size(Z2);
   c=2*ones(s);
   map=[1,0,0;0,1,0];
   surf(X,Y,Z2,c);
   xlabel('Percentage of autocars');
   ylabel('Avg num of cars per sec');
   zlabel('queueLength');
   colormap(map);
   alpha(0.8);
   str=sprintf('Autocar queue(green),usual car queue(red):\n(%i lanes in total. %i lane(s) dedicated to autocars.)',LANE_NUM,k-1);
   title(str);
   str=sprintf('L=%i,Separate QL.%i lane(s) dedicated to autocars.jpg',LANE_NUM,k-1);
   saveas(handle1,str);
    
end












