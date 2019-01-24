clear;clc;close all
%% temperory
NN=2;
%% set some Constants
global K1 K2 K3 K4 L1 L2 m1 m2 dt
%%Constants 
% https://hypertextbook.com/facts/2006/bodyproportions.shtml (size of human)
L1 = 0.26; %link 1 length (upper arm)
L2 = 0.43; %link 2 length (fore arm +hand)

m1 = 1.4; %link 1 mass (1.4kg)
m2 = 1.1; %link 2 mass (1.1kg)
dt = .1; %Simulation time step 
%%Compute non changing constants 
K1 = (1/3*m1+m2)*L1^2 + 1/3*m2*L2^2; %for ddTheta1 
K2 = m2*L1*L2; %for dd01 and others 
K3 = 1/3*m2*L2^2; %for ddTheta1 and ddTheta2 in t1 and t2 
K4 = 1/2*m2*L1*L2; %for ddTheta1 and ddTheta2 

%% plot the set-up (e.g., the arm and targets)
% initial arm endEffector (The home zone)

Theta10 = pi/3.162; %Shoulder joint angle 
Theta20 = pi/1.93; %Elbow joint angle  
[x10,y10,x20,y20]=endEffector(Theta10,Theta20,L1,L2);


halfChunk=0.23;
homeZoneToChest=0.43;

%% the task spec given by the experimenter (S.S.)
% give the target locations
t1=[0,0.10];t2=[0.025,0.05];
t0=[0,0];
t4=[0,-0.10];t3=[0.025,-0.05];
% a sequence of these locations
targets=[t1;t0;t3;t0;t2;t0;t4;t0];



% the active area for the pointing;
xLeft=-halfChunk-0.05;
xRight=xLeft+0.13;

yTop=homeZoneToChest+0.15;
yBottom=homeZoneToChest-0.15;

    

targets(:,1)=targets(:,1)-halfChunk;

targets(:,2)=targets(:,2)+homeZoneToChest;
nTargets=size(targets,1);
figure('position', [200 200 600 600])


%% plot the set up
plotSetup

%%
% initial state

a1=[-1:0.2:1];
actions= combvec(a1,a1);
nActions=size(actions,2);


listPtr = 1; 
QTableEnterMap = containers.Map;
QT=zeros(1e6, nActions);


gamma = 1;    % <- take this is an undiscounted task 
alpha = 1e-1;
epsilon=0.1;
maxSteps=200;
min_cumReward=1000;
min_xxyy=NaN(2,maxSteps);
min_whichTar=NaN(1,maxSteps);
min_Steps=maxSteps;
while 1
% run the program untril ctrl+C

% Starting a new episode
% Initial conditions 
Theta1t = Theta10;  %Shoulder joint angle 
Theta2t = Theta20;  %Elbow joint angle 
dTheta1t = 0;       %Shoulder angular velocity 
dTheta2t = 0;       %Elbow angular velocity 


targetIndex=1;
tarx=targets(targetIndex,1);
tary=targets(targetIndex,2);

nSteps=0;
terminal=0;


reach=0;

tau1=0; %torque on shoulder 
tau2=0; %torque on elbow 

stateIndex=1;
state= [Theta1t,Theta2t,dTheta1t,dTheta2t,targetIndex];
stateStr=num2str(state);
QTableEnterMap(stateStr) = stateIndex;

[Theta10,Theta20,dTheta1,dTheta2]=armDynamics(Theta1t,Theta2t,dTheta1t,dTheta2t,tau1,tau2);


xxyy=NaN(2,maxSteps);

whichTar=NaN(1,maxSteps);
cumReward=0;
hReach=0;
while ~terminal
    % choose an action
    max_inx=find(QT(stateIndex,:)==max(QT(stateIndex,:)));
    actionChosenIndex=max_inx(1,randi(length(max_inx)));   
    
    if( rand<epsilon )         % explore ... with a random action
        actionChosenIndex=randi(nActions);
    end
  
    nSteps=nSteps+1;
    
    if nSteps>maxSteps
        terminal=1;
    end

    % take the action 
    tau1=actions(1,actionChosenIndex);
    tau2=actions(2,actionChosenIndex);

    % enter new state
    [Theta1,Theta2,dTheta1,dTheta2]=armDynamics(Theta1t,Theta2t,dTheta1t,dTheta2t,tau1,tau2);
    

    
    %%Transfer to next time step 
    dTheta1t = dTheta1; 
    dTheta2t = dTheta2; 
    Theta1t = Theta1; 
    Theta2t = Theta2; 
    
    state= [Theta1,Theta2,dTheta1,dTheta2,targetIndex];
    [x1,y1,x2,y2]=endEffector(Theta1,Theta2,L1,L2);
    

    % decide the immediateReward
    cumReward=cumReward+(tau1.*10)^2+(tau2.*10)^2+1;
    immediateReward=-((tau1.*10)^2+(tau2.*10)^2+1);%-sqrt((x2-tarx)^2+(y2-tary)^2);%
    

    
    if x2>xRight
        immediateReward=-1000;
        terminal=1;
    elseif x2<xLeft
        immediateReward=-1000;
        terminal=1;
    elseif y2>yTop
        immediateReward=-1000;
        terminal=1;
    elseif y2<yBottom
        immediateReward=-1000;
        terminal=1;
    else
    end
        
     
%     % the shouder/elbow angle is too big or too small
%     % it gives a big penalty
%     % shoulder
%     if Theta1>80*(pi/180) 
%         immediateReward=-100;
%         terminal=1;
%         tarminalCause=TC_ShoulderOver;
%     elseif Theta1<pi*20/180
%         immediateReward=-100;
%         terminal=1;
%         tarminalCause=TC_ShoulderUnder;
%         
%     %Elbow  
%     elseif Theta2>pi*150/180 
%         immediateReward=-100;
%         terminal=1;
%         tarminalCause=TC_ElbowOver;
%     elseif Theta2<pi*40/180
%         immediateReward=-100;
%         terminal=1;
%         tarminalCause=TC_ShoulderUnder;
%     else
%     end
    
    % if the intermidate target is reached
    if sqrt((x2-tarx)^2+(y2-tary)^2)<0.005
        immediateReward=0;
        
        
        % the if the last target is reached
        if targetIndex==NN%;nTargets
            immediateReward=1000;
            terminal=1;
            reach=1;
        else % go the next target
            % go the the next target
            targetIndex=targetIndex+1;
            tarx=targets(targetIndex,1);
            tary=targets(targetIndex,2);
        end
    end
    
    

    % new state
    stateStr=num2str(state);
    
    if QTableEnterMap.isKey(stateStr)
        stateIndexNew= QTableEnterMap(stateStr);        
    else
        listPtr=listPtr+1;
        QTableEnterMap(stateStr) = listPtr;
        stateIndexNew=listPtr;
    end
    
    
    xxyy(:,nSteps)=[Theta1;Theta2];
    whichTar(:,nSteps)=targetIndex;

    
    
    % Q-table update
    if(terminal==0) % not the terminal state
        QT(stateIndex,actionChosenIndex) =  QT(stateIndex,actionChosenIndex)  + alpha*( immediateReward + ...
            gamma*max(QT(stateIndexNew,:)) - QT(stateIndex,actionChosenIndex) ); 

        stateIndex=stateIndexNew;
    else % IS the terminal state ... there is no Q_qlearn(b';a') term in the qlearn update
        QT(stateIndex,actionChosenIndex) =  QT(stateIndex,actionChosenIndex)  + alpha*( immediateReward - QT(stateIndex,actionChosenIndex) ); 
    end
    
    
   
end






% if the last target is reached
if reach==1
    if min_cumReward>cumReward %min_Steps>nSteps
        min_cumReward=cumReward;
        min_xxyy=xxyy;
        min_whichTar=whichTar;
        min_Steps =nSteps;
    end
    clear cm
    cm=colormap(winter(min_Steps));
    
    clf
    plotSetup

    set(gca,'FontSize',12)
%     set(gca,'xTickLabels',get(gca,'xTick').*100)
%     set(gca,'yTickLabels',get(gca,'yTick').*100)
    xlabel('meter');
    ylabel('meter');
    wt0=0;
    for s=1:min_Steps     
        Theta1=min_xxyy(1,s);
        Theta2=min_xxyy(2,s);
        wt=min_whichTar(1,s);
        if wt>wt0
            wt0=wt;
            if wt0>1
                hReach=plot(targets(wt0-1,1),targets(wt0-1,2),'gs','MarkerSize',15,'MarkerFaceColor','g');
            end
            hNext=plot(targets(wt0,1),targets(wt0,2),'rs','MarkerSize',15,'MarkerFaceColor','r');
        end
        
         
        
        [x1,y1,x2,y2]=endEffector(Theta1,Theta2,L1,L2);
        plot([0 x1],[0 y1],'linewidth',1,'color','b'); hold on%plot shoulder to elbow 
        plot([x1 x2],[y1,y2],'linewidth',1,'color','b') %plot elbow to hand 

        plot(x2,y2,'r.','markerSize',23,'color',cm(s,:));  
        pause(0.2)

        hold on
    
        if hReach~=0
            legend([hNext,hReach],{'Next Target';'Reached'})
        else
            legend(hNext,'Next Target')
        end
    end
    % the final target
    if reach==1
    hReach=plot(targets(NN,1),targets(NN,2),'gs','MarkerSize',15,'MarkerFaceColor','g');
    pause(0.1)
    end
end
end