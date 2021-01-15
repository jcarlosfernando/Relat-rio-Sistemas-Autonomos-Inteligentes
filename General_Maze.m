
%% Disciplina Sistemas Autônomos Inteligentes - UTFPR 2020/2
% Autor: Carlos Joventino

%% Programa p/ envio dos "goals" c/ navegação pelo navigation stack
% Projeto final disciplina Sistemas Autônomos Inteligentes
% Robocup Maze

clc;
clearvars;
close all;

% Tolerância de correção
toler_dist = 0.1; % Tolerância de distância [m]

din_goal1 = rosmessage('nav_msgs/Odometry'); % Define o tipo de mensagem
goal1 = rossubscriber('/goal1'); % Cria o subscriber

din_goal2 = rosmessage('nav_msgs/Odometry'); % Define o tipo de mensagem
goal2 = rossubscriber('/goal2'); % Cria o subscriber

din_goal3 = rosmessage('nav_msgs/Odometry'); % Define o tipo de mensagem
goal3 = rossubscriber('/goal3'); % Cria o subscriber

din_goal4 = rosmessage('nav_msgs/Odometry'); % Define o tipo de mensagem
goal4 = rossubscriber('/goal4'); % Cria o subscriber

din_goal5 = rosmessage('nav_msgs/Odometry'); % Define o tipo de mensagem
goal5 = rossubscriber('/goal5'); % Cria o subscriber

pos = rosmessage('geometry_msgs/PoseStamped'); % Define o tipo de mensagem
pub = rospublisher('/move_base_simple/goal'); % Cria o publisher

tftree = rostf;
pause(1);
tftree.AvailableFrames;

% Recebe a mensagem do goal dinâmico
goal_pos1 = receive (goal1, 10);
goal_pos2 = receive (goal2, 10);
goal_pos3 = receive (goal3, 10);
goal_pos4 = receive (goal4, 10);
goal_pos5 = receive (goal5, 10);


x1 = goal_pos1.Pose.Pose.Position.X;
y1 = goal_pos1.Pose.Pose.Position.Y;
Qz1 = goal_pos1.Pose.Pose.Orientation.Z;
w1 = goal_pos1.Pose.Pose.Orientation.W;

x2 = goal_pos2.Pose.Pose.Position.X;
y2 = goal_pos2.Pose.Pose.Position.Y;
Qz2 = goal_pos2.Pose.Pose.Orientation.Z;
w2 = goal_pos2.Pose.Pose.Orientation.W;

x3 = goal_pos3.Pose.Pose.Position.X;
y3 = goal_pos3.Pose.Pose.Position.Y;
Qz3 = goal_pos3.Pose.Pose.Orientation.Z;
w3 = goal_pos3.Pose.Pose.Orientation.W;

x4 = goal_pos4.Pose.Pose.Position.X;
y4 = goal_pos4.Pose.Pose.Position.Y;
Qz4 = goal_pos4.Pose.Pose.Orientation.Z;
w4 = goal_pos4.Pose.Pose.Orientation.W;

x5 = goal_pos5.Pose.Pose.Position.X;
y5 = goal_pos5.Pose.Pose.Position.Y;
Qz5 = goal_pos5.Pose.Pose.Orientation.Z;
w5 = goal_pos5.Pose.Pose.Orientation.W;

desired_pos = [x1,y1; x2,y2; x3,y3; x4,y4; x5,y5]; % Coordenadas (x,y) da posição desejada (Goal Dinâmico)
desired_orie = [Qz1,w1; Qz2,w2; Qz3,w3; Qz4,w4; Qz5,w5];

for i = 1:length(desired_pos)    
    
    pos.Header.FrameId = 'map';
    pos.Pose.Position.X = desired_pos(i,1);
    pos.Pose.Position.Y = desired_pos(i,2);    
    pos.Pose.Orientation.Z = desired_orie(i,1);
    pos.Pose.Orientation.W = desired_orie(i,2);
    send(pub,pos);        

    dist = inf;
    while dist > toler_dist
        [dist,erro_ang] = Pos_Control(desired_pos(i,1), desired_pos(i,2), tftree);
    end    
    disp ("****Posição Atingida****");
    pause(5);
end

disp ("****Simulação Encerrada****");
