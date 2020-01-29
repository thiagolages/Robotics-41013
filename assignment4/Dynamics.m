classdef Dynamics < handle
    properties
        
           
    end
    
    methods(Static)
        function self = Dynamics()
            % do nothing
        end 
        
        function [tau, qdd, qd, q] = solveAndPlot(p560, qMatrix, time, dt)
             tau_max = [97.6 186.4 89.4 24.2 20.1 21.3]';                                % Maximum joint torque of the Puma560

            % Variables to change
                                                                           % Set control frequency at 100Hz
            %steps = size(qMatrix,1);                                                            % No. of steps along trajectory
            steps = time/dt;
            s = lspb(0,1,steps);                                                      % Generate trapezoidal velocity profile
            q_size = size(qMatrix,1);
            for i = 1:steps
                q(i,:) = (1-s(i))*qMatrix(1,:) + s(i)*qMatrix(q_size,:); % take first and last
            end
            %q = qMatrix;
            qd = zeros(steps,6);                                                        % Array of joint velocities
            qdd = nan(steps,6);                                                         % Array of joint accelerations
            tau = nan(steps,6);                                                         % Array of joint torques
            
            for i = 1:steps-1
                qdd(i,:) = (1/dt)^2 * (q(i+1,:) - q(i,:) - dt*qd(i,:));                 % Calculate joint acceleration to get to next set of joint angles
                M = p560.inertia(q(i,:));                                               % Calculate inertia matrix at this pose
                C = p560.coriolis(q(i,:),qd(i,:));                                      % Calculate coriolis matrix at this pose
                g = p560.gravload(q(i,:));                                              % Calculate gravity vector at this pose
                
                tau(i,:) = (M*qdd(i,:)' + C*qd(i,:)' + g')';                            % Calculate the joint torque needed
                
                % NOT capping it. we wanna see if it exceeds
%                 for j = 1:6
%                     if abs(tau(i,j)) > tau_max(j)                                       % Check if torque exceeds limits
%                         tau(i,j) = sign(tau(i,j))*tau_max(j);                           % Cap joint torque if above limits
%                     end
%                 end
                disp(i);
                qdd(i,:) = (inv(M)*(tau(i,:)' - C*qd(i,:)' - g'))';                     % Re-calculate acceleration based on actual torque
                q(i+1,:) = q(i,:) + dt*qd(i,:) + dt^2*qdd(i,:);                         % Update joint angles based on actual acceleration
                qd(i+1,:) = qd(i,:) + dt*qdd(i,:);                                      % Update the velocity for the next pose
            end

            t = 0:dt:(steps-1)*dt;                                                      % Generate time vector

            % Visulalisation and plotting of results

            % Plot joint angles
            figure(7)
            for j = 1:6
                subplot(3,2,j)
                plot(t,q(:,j)','k','LineWidth',1);
                refline(0,p560.qlim(j,1));
                refline(0,p560.qlim(j,2));
                ylabel('Angle (rad)');
                box off
            end

            % Plot joint velocities
            figure(8)
            for j = 1:6
                subplot(3,2,j)
                plot(t,qd(:,j)*30/pi,'k','LineWidth',1);
                refline(0,0);
                ylabel('Velocity (RPM)');
                box off
            end

            % Plot joint acceleration
            figure(9)
            for j = 1:6
                subplot(3,2,j)
                plot(t,qdd(:,j),'k','LineWidth',1);
                ylabel('rad/s/s');
                refline(0,0)
                box off
            end

            % Plot joint torques
            figure(10)
            for j = 1:6
                subplot(3,2,j)
                plot(t,tau(:,j),'k','LineWidth',1);
                refline(0,tau_max(j));
                refline(0,-tau_max(j));
                ylim([-tau_max(j)*1.1 tau_max(j)*1.1]);
                ylabel('Nm');
                box off
            end

            figure(1)
            p560.plot(q,'fps',steps)

            
        end
        
    end
end