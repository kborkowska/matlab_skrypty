%% analysis script for finding limit acc anc vel for IRPOS

max_inc_to_max_vel(200, 0)


% %% prepare file opening specs
% time = '08:05:15';
% date = '08-05-2018';
% path = '~/IRPOS_results/trapezoid_generator_results/';
% 
% tolerances = [0.01 0.01 0.01 0.01 0.01 0.01];
% numberOfJoints = 6;
% 
% fullPath = [path date '--' time '/'];
% 
% formatSpec = '';
% for i=1:numberOfJoints
%     formatSpec = [formatSpec '%f ']; 
% end
% 
% %% get setpoint data
% file = fopen([fullPath 'setpoints.txt'], 'r');
% setpoints = fscanf(file, formatSpec, [numberOfJoints Inf] );
% setpoints = setpoints';
% 
% %% get result data
% file = fopen([fullPath 'results.txt'], 'r');
% results = fscanf(file, formatSpec, [numberOfJoints Inf] );
% results = results';
% 
% %% calculate differences
% problems = zeros(size(results,1),numberOfJoints);
% for i=1:numberOfJoints
%     for j=1:size(results,1)
%         if (setpoints(j,i)-results(j,i))^2 > tolerances(i)^2
%             problems(j,i) = 1;
%         end
%     end
% end
% 
% %% create graphs
% for i=1:numberOfJoints
%     figure('Name', ['Joint no.' num2str(i) ' results']);
%     plot(setpoints(:,i),'b');
%     hold on;
%     plot(results(:,i),'g');
%     x = [];
%     y = [];
%     for j=1:size(problems,1)
%        if problems(j, i)>0
%             x = [x j];
%             y = [y results(j,i)];
%        end
%     end
%     if size(x)>0
%         plot(x,y,'r*');
%     end
%     ylabel('position')
%     xlabel('time')
% end


