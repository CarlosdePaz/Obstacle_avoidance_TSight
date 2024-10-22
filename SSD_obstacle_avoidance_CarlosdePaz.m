%% GRASPING OBJECTS WITH A VIBROTACTILE SENSORY SUBSTITUTION GLOVE

% Program developed by Carlos de Paz (carlos.paz@uam.es, carlosdepazrios94@gmail.com).

clc; clear all; close all;
cd('D:\SSD vs Cane\Programs')
load('ModalityOrder.mat')

% 1. Get the participant's number and make his/her directory

pp = input('Please, introduce participant number: \n');
fprintf('Participant number = %i \n', pp) 
Pnumber=['PP',num2str(pp)];

DirID=Pnumber;
[s, mess, messid] = mkdir(DirID);
cd(DirID); MainCD = cd;

%shoulder_width = input('Enter the shoulder width in cm \n');
fprintf('Press a key to start the Experiment \n')
pause()

% 2. Open the port with Qualisys. Get the devices. Set some parameters
Data = QMC('D:\SSD vs Cane\Programs\QMC_Conf.txt'); % Open the port with Qualisys
vectorLength = 2;%1.5; %vector module
n_vectors = 8; %number of vectors
D = 0.05; %distance between vectors 
VoltageT = zeros(3,16); 
index_modality = 1;
slope = -40; %linear function (Vibration = slope*distance + intercept)
intercept = 240;

% 3.1 Define the Corridor
width_corridor = 2.5;
height_corridor = 10;
center_corridor = [1.25, 4];

rect_corridor = [center_corridor(1) - width_corridor/2, center_corridor(2) - height_corridor/2,...        
    center_corridor(1) + width_corridor/2, center_corridor(2) - height_corridor/2,...        
    center_corridor(1) + width_corridor/2, center_corridor(2) + height_corridor/2,...        
    center_corridor(1) - width_corridor/2, center_corridor(2) + height_corridor/2];

%3.2 Define the first obstacle
width_1_Obj =  .4;
height_1_Obj = .3;
center_1_Obj = [1.25, 2 + height_1_Obj/2];

rect_1_Obj = [center_1_Obj(1) - width_1_Obj/2, center_1_Obj(2) - height_1_Obj/2,...        
    center_1_Obj(1) + width_1_Obj/2, center_1_Obj(2) - height_1_Obj/2,...        
    center_1_Obj(1) + width_1_Obj/2, center_1_Obj(2) + height_1_Obj/2,...        
    center_1_Obj(1) - width_1_Obj/2, center_1_Obj(2) + height_1_Obj/2];

%4. Start the experiment 
for trial = 1:12 % 4 conditions per 3 modalities 
     
    % 5. Display Trial number, get the condition (position of the "mobile" obstacles) and display the trial condition
    [conf, rect_corner_obstacles] = GetObjNavig(pp,trial);
    rect_corner = [rect_1_Obj; rect_corner_obstacles; rect_corridor];
    conf.pp = pp; conf.trial = trial; %conf.shoulder_width = shoulder_width;
    conf.modality = ModalityOrder(pp, index_modality);
       
    if trial == 1 
        conf.modality = ModalityOrder(pp, index_modality);
        if conf.modality == 1
            modality_disp = '------------ SSD ------------';
        elseif conf.modality == 2
            modality_disp = '------------ Cane -----------';
        else
            modality_disp = '------------ Cross-Modal ----';
        end    
        fprintf(['The modality is ', modality_disp,'\n'])
    elseif trial == 5 || trial == 9
        index_modality = index_modality + 1;
        conf.modality = ModalityOrder(pp, index_modality);
        if conf.modality == 1
            modality_disp = '------------ SSD ------------';
        elseif conf.modality == 2
            modality_disp = '------------ Cane -----------';
        else
            modality_disp = '------------ Cross-Modal ----';
        end
        fprintf(['Change modality to ', modality_disp,'\n'])
    end   
    fprintf('Trial = %i \n', trial);
    
    % Plot the obstacles location
    f_condition = figure(1);
    hold on
    box on
    axis([-2.5,4.5,1,7])
    plot([rect_corner(4,1) rect_corner(4,3) rect_corner(4,5) rect_corner(4,7) rect_corner(4,1)],...
                [rect_corner(4,2) rect_corner(4,4) rect_corner(4,6) rect_corner(4,8) rect_corner(4,2)], 'k', 'LineWidth', 3);
    fill(rect_corner(1,1:2:end), rect_corner(1,2:2:end),'k')
    fill(rect_corner(2,1:2:end), rect_corner(2,2:2:end),'k')
    fill(rect_corner(3,1:2:end), rect_corner(3,2:2:end),'k')
    H1 = uicontrol('Style', 'PushButton', 'String', 'Close', 'Callback', 'close all'); %button to stop the trial (loop)
    hold off
    
    fprintf('Press a key to start the trial \n')
    pause()
    close all
    % 6. Get data from Qualisys, calculate the final points and the intercepts
    rb_data = QMC(Data);
    
    %Stop the script anytime the participant is not correctly detected
    Error = isnan(rb_data(1));
    flag_print=0;
      while Error ~= 0
        rb_data = QMC(Data);
        if flag_print == 0
           fprintf('Waiting for data \n')
           flag_print = 1;
        end
        Error = isnan(rb_data(1));
      end
     clear Error
    
    rb_data(1:3) = rb_data(1:3)./1000; %work in meters
    rb_data(1) = rb_data(1);
    rb_data(2) = rb_data(2) + 0.2;

    origin = rb_data(1:2); %displace the data according to the L-translation (Qualisys)
    angle = -rb_data(6);
    vectors_8 = calculate_vectors_n(vectorLength, D, origin, angle, n_vectors); % calculate the final 8 vectors (its initial and final position in X-Y)
    [x_intercept,y_intercept,distances] = intercept_vectors_n_obstacles(rect_corner, vectors_8); % calculate the intercepts between the vectors and the obstacles.
    
    %7. Calculate the voltage
    VoltageT(:,1:2:end) = [slope*min(distances') + intercept; slope*min(distances') + intercept; slope*min(distances') + intercept];%linear function [Vmax = 8, then use V = -1.6667*min(distances') + 8]
    VoltageT(:,2:2:end) = VoltageT(:,1:2:end);
    
    if conf.modality ~= 2
        send_array(uint8(VoltageT))        
    end
    
    % 8. Plot the data (it will be continuously updated)
    f = figure(1);
    hold on
    box on
    axis([-2.5,4.5,-1,7])
    H = uicontrol('Style', 'PushButton', 'String', 'Stop', 'Callback', 'delete(gcbf)'); %button to stop the trial (loop)
    % 8.1 Plot the obstacles
    plot([rect_corner(4,1) rect_corner(4,3) rect_corner(4,5) rect_corner(4,7) rect_corner(4,1)],...
                [rect_corner(4,2) rect_corner(4,4) rect_corner(4,6) rect_corner(4,8) rect_corner(4,2)], 'k', 'LineWidth', 3);
    fill(rect_corner(1,1:2:end), rect_corner(1,2:2:end),'k')
    fill(rect_corner(2,1:2:end), rect_corner(2,2:2:end),'k')
    fill(rect_corner(3,1:2:end), rect_corner(3,2:2:end),'k')
    
    % 8.1 Plot the vectors
    ini_plot = plot(vectors_8(:,1), vectors_8(:,2), 'ro');
    final_plot = plot(vectors_8(:,3), vectors_8(:,4), 'bo');

    % 8.1 Plot the line between the initial and end coordinates of the vectors
    line_plot_1 = plot([vectors_8(1,1), vectors_8(1,3)],[vectors_8(1,2),vectors_8(1,4)], 'k--');
    line_plot_2 = plot([vectors_8(2,1), vectors_8(2,3)],[vectors_8(2,2),vectors_8(2,4)], 'k--');
    line_plot_3 = plot([vectors_8(3,1), vectors_8(3,3)],[vectors_8(3,2),vectors_8(3,4)], 'k--');
    line_plot_4 = plot([vectors_8(4,1), vectors_8(4,3)],[vectors_8(4,2),vectors_8(4,4)], 'k--');
    line_plot_5 = plot([vectors_8(5,1), vectors_8(5,3)],[vectors_8(5,2),vectors_8(5,4)], 'k--');
    line_plot_6 = plot([vectors_8(6,1), vectors_8(6,3)],[vectors_8(6,2),vectors_8(6,4)], 'k--');
    line_plot_7 = plot([vectors_8(7,1), vectors_8(7,3)],[vectors_8(7,2),vectors_8(7,4)], 'k--');
    line_plot_8 = plot([vectors_8(8,1), vectors_8(8,3)],[vectors_8(8,2),vectors_8(8,4)], 'k--');
    
    % 8.3 Plot the intercepts
    int_plot_1 = plot(x_intercept(1,:), y_intercept(1,:), 'ro');
    int_plot_2 = plot(x_intercept(2,:), y_intercept(2,:), 'ro');
    int_plot_3 = plot(x_intercept(3,:) , y_intercept(3,:), 'ro');
    int_plot_4 = plot(x_intercept(4,:), y_intercept(4,:), 'ro');
    int_plot_5 = plot(x_intercept(5,:), y_intercept(5,:), 'ro');
    int_plot_6 = plot(x_intercept(6,:), y_intercept(6,:), 'ro');
    int_plot_7 = plot(x_intercept(7,:) , y_intercept(7,:), 'ro');
    int_plot_8 = plot(x_intercept(8,:), y_intercept(8,:), 'ro');

    % 9. Prepare the empty matrix to store the variables    
    MatrixData(1,:) = zeros(1,4); 
    n = 1; % variable to store 
    tic; %measure the movement time
   
    % 10. Start the trial loop
    while ishandle(H) % The trial (loop) runs until the stop button of the figure is pressed
   
    % 11. Get the data, intercepts and the voltage (stream)
    rb_data = QMC(Data);    
    
    rb_data(1:3) = rb_data(1:3) ./1000;
    rb_data(1) = rb_data(1);
    rb_data(2) = rb_data(2) + 0.2;
    origin = rb_data(1:2);
    angle = -rb_data(6);
    vectors_8 = calculate_vectors_n(vectorLength, D, origin, angle, n_vectors);
    
    [x_intercept,y_intercept,distances] = intercept_vectors_n_obstacles(rect_corner, vectors_8);
    
    VoltageT(:,1:2:end) = [slope*min(distances') + intercept; slope*min(distances') + intercept; slope*min(distances') + intercept];
    VoltageT(:,2:2:end) = VoltageT(:,1:2:end);
    VoltageT(isnan(VoltageT)) = 0; % in case some of the vectors are not intercepting the obstacles change the NaN for zeros
    VoltageT(VoltageT < 0) = 0;
    VoltageT(VoltageT > 255) = 255;
    
    if conf.modality ~= 2
        send_array(uint8(VoltageT));
    end
    
    % 12. Store the values 
    time = toc;
    MatrixData(n,:) = [rb_data(1:2)', rb_data(6), time]; % store the X-Y coordinates of the rigid-body, the yaw rotation and the movement time
    n = n+1;
    
    %11. Update the figure
    %11. 1. Initial and final position of the vectors 
    set(ini_plot, 'Xdata', vectors_8(:,1), 'Ydata', vectors_8(:,2))
    set(final_plot, 'Xdata', vectors_8(:,3), 'Ydata', vectors_8(:,4))
    
    %11. 2. Line between the initial and final position of the vectors 
    set(line_plot_1, 'Xdata', [vectors_8(1,1), vectors_8(1,3)], 'Ydata', [vectors_8(1,2), vectors_8(1,4)])
    set(line_plot_2, 'Xdata', [vectors_8(2,1), vectors_8(2,3)], 'Ydata', [vectors_8(2,2), vectors_8(2,4)])
    set(line_plot_3, 'Xdata', [vectors_8(3,1), vectors_8(3,3)], 'Ydata', [vectors_8(3,2), vectors_8(3,4)])
    set(line_plot_4, 'Xdata', [vectors_8(4,1), vectors_8(4,3)], 'Ydata', [vectors_8(4,2), vectors_8(4,4)])
    set(line_plot_5, 'Xdata', [vectors_8(5,1), vectors_8(5,3)], 'Ydata', [vectors_8(5,2), vectors_8(5,4)])
    set(line_plot_6, 'Xdata', [vectors_8(6,1), vectors_8(6,3)], 'Ydata', [vectors_8(6,2), vectors_8(6,4)])
    set(line_plot_7, 'Xdata', [vectors_8(7,1), vectors_8(7,3)], 'Ydata', [vectors_8(7,2), vectors_8(7,4)])
    set(line_plot_8, 'Xdata', [vectors_8(8,1), vectors_8(8,3)], 'Ydata', [vectors_8(8,2), vectors_8(8,4)])

    %11. 3. Intercepts between the vectors and the obstacles
    set(int_plot_1, 'Xdata', x_intercept(1,:), 'Ydata', y_intercept(1,:))
    set(int_plot_2, 'Xdata', x_intercept(2,:), 'Ydata', y_intercept(2,:))
    set(int_plot_3, 'Xdata', x_intercept(3,:), 'Ydata', y_intercept(3,:))
    set(int_plot_4, 'Xdata', x_intercept(4,:), 'Ydata', y_intercept(4,:))
    set(int_plot_5, 'Xdata', x_intercept(5,:), 'Ydata', y_intercept(5,:))
    set(int_plot_6, 'Xdata', x_intercept(6,:), 'Ydata', y_intercept(6,:))
    set(int_plot_7, 'Xdata', x_intercept(7,:), 'Ydata', y_intercept(7,:))
    set(int_plot_8, 'Xdata', x_intercept(8,:), 'Ydata', y_intercept(8,:))

    drawnow
       
    end
    hold off
    close all
    clc
    
    % 12. Turn off the motors
    VoltageT = zeros(3,16); 
    send_array(uint8(VoltageT));
    % 13. Create a table and save the data
    conf.hits = input('Enter the number of hits \n');
    
    if conf.modality ~= 1
        conf.bastonazos = input('Enter the number of bastonazos \n');
    end
    
    MatrixDataTable = array2table(MatrixData, 'VariableNames', {'X', 'Y', 'YAW', 'Time'});
    Tnumber = ['TR', num2str(trial)];
    FileID = [Pnumber,Tnumber];
    save(FileID, 'conf', 'MatrixData', 'MatrixDataTable')

    clearvars -except pp Pnumber rect_corridor rect_1_Obj vectorLength n_vectors D Data VoltageT ModalityOrder index_modality conf MatrixDataTable FileID slope intercept
end

QMC(Data, 'quit') %once the participant finish the experiment, close the port with Qualisys

%% GetObjNavig

% This function generates the obstacles positions regarding the participant
% and the trial.

function [conf,rect_corner_obstacles] = GetObjNavig(pp,trial)
    
        load NavigOrder.mat
        condition = NavigOrder(pp,trial);
        
        width_Corridor = 2.5;
        width_Obj = 0.43;
        height_Obj = 0.37;
        adjustment = 0.00;

        %Left position --> "x0" + width_Obj /2
        %Center --> "x0"+ width_Corridor/2 
        %Rigth position --> 2 - width_Obj /2 + "x0"
                
        switch condition
            case 1 %2nd --> left; 3th --> center
            
                center_2_Obj = [width_Obj/2, 3.5 + height_Obj/2];
                rect_2_Obj = [center_2_Obj(1) - width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) + height_Obj/2,...        
                    center_2_Obj(1) - width_Obj/2, center_2_Obj(2) + height_Obj/2];
                
                center_3_Obj = [width_Corridor/2, 5 + height_Obj/2];
                rect_3_Obj = [center_3_Obj(1) - width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) + height_Obj/2,...        
                    center_3_Obj(1) - width_Obj/2, center_3_Obj(2) + height_Obj/2];
                
                rect_corner_obstacles = [rect_2_Obj; rect_3_Obj];
                conf.condition_obstacles = 1;
                
            case 2 %2nd --> left; 3th --> rigth
                
                center_2_Obj = [width_Obj/2, 3.5 + height_Obj/2];
                rect_2_Obj = [center_2_Obj(1) - width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) + height_Obj/2,...        
                    center_2_Obj(1) - width_Obj/2, center_2_Obj(2) + height_Obj/2];
                
                center_3_Obj = [width_Corridor - width_Obj/2 - adjustment, 5 + height_Obj/2];
                rect_3_Obj = [center_3_Obj(1) - width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) + height_Obj/2,...        
                    center_3_Obj(1) - width_Obj/2, center_3_Obj(2) + height_Obj/2];
                
                rect_corner_obstacles = [rect_2_Obj; rect_3_Obj];
                conf.condition_obstacles = 2;

            case 3 %2nd --> rigth; 3th --> center
                
                center_2_Obj = [width_Corridor - width_Obj/2 - adjustment, 3.5 + height_Obj/2];
                rect_2_Obj = [center_2_Obj(1) - width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) + height_Obj/2,...        
                    center_2_Obj(1) - width_Obj/2, center_2_Obj(2) + height_Obj/2];
                
                center_3_Obj = [width_Corridor/2, 5 + height_Obj/2];
                rect_3_Obj = [center_3_Obj(1) - width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) + height_Obj/2,...        
                    center_3_Obj(1) - width_Obj/2, center_3_Obj(2) + height_Obj/2];
                
                rect_corner_obstacles = [rect_2_Obj; rect_3_Obj];
                conf.condition_obstacles = 3;
                
            case 4 %2nd --> rigth; 3th --> left
                
                center_2_Obj = [width_Corridor - width_Obj/2 - adjustment, 3.5 + height_Obj/2];
                rect_2_Obj = [center_2_Obj(1) - width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) - height_Obj/2,...        
                    center_2_Obj(1) + width_Obj/2, center_2_Obj(2) + height_Obj/2,...        
                    center_2_Obj(1) - width_Obj/2, center_2_Obj(2) + height_Obj/2];

                center_3_Obj = [width_Obj/2, 5 + height_Obj/2];
                rect_3_Obj = [center_3_Obj(1) - width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) - height_Obj/2,...        
                    center_3_Obj(1) + width_Obj/2, center_3_Obj(2) + height_Obj/2,...        
                    center_3_Obj(1) - width_Obj/2, center_3_Obj(2) + height_Obj/2];
                
                rect_corner_obstacles = [rect_2_Obj; rect_3_Obj];
                conf.condition_obstacles = 4;
        end
end

%% send_array

%This function opens a port between Matlab and the device. We send the
%voltage to each motor (0-255)

function send_array(msg, ip, port, dofeedback)
    if nargin < 4, port = 12000; end
    if nargin < 3, ip = '192.168.4.1'; end
    if nargin < 2, dofeedback = false; end
    if nargin < 1, msg = randi([1 255], 3, 16, 'uint8'); end

    if dofeedback
        disp(msg)
    end
    udp_conn = udp(ip, port);
    fopen(udp_conn);
    fwrite(udp_conn , matlab2python(msg), 'uint8')
    fclose(udp_conn);
    delete(udp_conn);

end
%% intercept_vectors_n_obstacles

%This function calculates whenever the user is facing a obstacle. In that
%case, we calculate the intercept coordinates and the corresponding
%distance

function [x_intercept,y_intercept,distances] = intercept_vectors_n_obstacles(rect_corner, vectors_n)
    x_intercept = NaN(length(vectors_n(:,1)),length(rect_corner(:,1)));
    y_intercept = NaN(length(vectors_n(:,1)),length(rect_corner(:,1)));
    distances = NaN(length(vectors_n(:,1)),length(rect_corner(:,1)));
        for vector = 1:length(vectors_n(:,1))
            for obstacle = 1:length(rect_corner(:,1))
                    [xi,yi] = polyxpoly([vectors_n(vector,1) vectors_n(vector,3)], [vectors_n(vector,2) vectors_n(vector,4)],...
                         [rect_corner(obstacle,1) rect_corner(obstacle,3) rect_corner(obstacle,5) rect_corner(obstacle,7) rect_corner(obstacle,1)],...
                         [rect_corner(obstacle,2) rect_corner(obstacle,4) rect_corner(obstacle,6) rect_corner(obstacle,8) rect_corner(obstacle,2)]);
                     
                     if length(xi) > 1
                         xi_a = xi(1); xi_b = xi(2);
                         yi_a = yi(1); yi_b = yi(2);
                         
                         d_a = sqrt((xi_a - vectors_n(vector,1)).^2 + (yi_a - vectors_n(vector,2)).^2);
                         d_b = sqrt((xi_b - vectors_n(vector,1)).^2 + (yi_b - vectors_n(vector,2)).^2);
                         
                        xi = []; yi = [];
                        
                         if d_a > d_b
                             xi = xi_b; yi = yi_b; 
                             d = d_b;
                         else
                             xi = xi_a; yi = yi_a; 
                              d = d_a;
                         end
                         
                     else
                         d = sqrt((xi - vectors_n(vector,1)).^2 + (yi - vectors_n(vector,2)).^2);
                     end
                     
                     
                    try 
                        x_intercept(vector,obstacle) = xi;
                        y_intercept(vector,obstacle) = yi;
                        distances(vector, obstacle) = d;
                    catch
                        x_intercept(vector,obstacle) = NaN;
                        y_intercept(vector,obstacle) = NaN;
                        distances(vector, obstacle) = NaN;
                    end

                clear d xi yi
            end
        end
end