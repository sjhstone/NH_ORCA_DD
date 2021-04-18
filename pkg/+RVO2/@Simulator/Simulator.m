% AUTHOR: Jiahe Shi, 2020

classdef Simulator < handle
    
    properties
        TimeStep
        TimeHorizon % tau = TimeHorizon*TimeStep
        
        Field
        Agents
        Goals
        
        NeighborDist
        MaxNeighbors
        
        Visualization
        
        MODEL
    end
    
    properties (Access=private)
        GlobalTime
    end
    
    methods
        function obj = Simulator(dt,N,minDist,maxNN, MODEL)
            arguments
                dt
                N
                minDist
                maxNN
                MODEL = 'effective'
            end
            
            obj.TimeStep = dt;
            obj.TimeHorizon = N;
            obj.NeighborDist = minDist;
            obj.MaxNeighbors = maxNN;
            obj.MODEL = MODEL;
        end
        
        function setAgentList(obj, AA)
            obj.Agents = AA;
        end
        
        function setAgentGoalList(obj, goals)
            obj.Goals = goals;
        end
        
        function flag_reachedGoal = reachedGoal(obj)
            flag_reachedGoal = false;
            for ii = 1:numel(obj.Agents)
                if norm( obj.Agents(ii).q - obj.goals(ii,:) ) > 0.2
                    return
                end
            end
            flag_reachedGoal = true;
        end
        
        function doStep(obj)
%             buildAgentTree();
%             for ii = 1:numel(obj.Agents)
%                 obj.Agents(ii).computeNeighbors();
%                 obj.Agents(ii).computeNewVelocity();
%             end
            prefVel = NaN(numel(obj.Agents),2);
            
            DIST_EPS = obj.Agents(1).R;
            
            
            
            for ii = 1:numel(obj.Agents)
                roadToGoal = obj.Goals(ii,:) - obj.Agents(ii).q;
                distToGoal = norm(roadToGoal);
                disp([distToGoal;DIST_EPS]);
                if distToGoal > DIST_EPS
                    prefVel(ii,:) = obj.Goals(ii,:) - obj.Agents(ii).q;
                    if norm(prefVel(ii,:)) > obj.Agents(ii).vmax
                        prefVel(ii,:) = obj.Agents(ii).vmax*prefVel(ii,:)/norm(prefVel(ii,:));
                    end
                else
                    prefVel(ii,:) = [0 0];
                end
            end
            
            obj.Agents(1).computeNewVelocity(obj.Agents(2), prefVel(1,:), obj.TimeStep, obj.TimeHorizon, obj.Visualization, obj.MODEL);
            obj.Agents(2).computeNewVelocity(obj.Agents(1), prefVel(2,:), obj.TimeStep, obj.TimeHorizon, obj.Visualization, obj.MODEL);
            
            
            
            for ii = 1:numel(obj.Agents)
                obj.Agents(ii).doStep(obj.TimeStep);
                obj.Agents(ii).updatePlot(obj.Visualization.Plt(ii));
            end
            
            obj.GlobalTime = obj.GlobalTime + obj.TimeStep;
        end
        
        function defineField(obj, w, h)
            obj.Field.w = w;
            obj.Field.h = h;
            obj.Field.x = [-w/2 w/2];
            obj.Field.y = [-h/2 h/2];
        end
        
        function initVisualization(obj)
            simVisFig = figure;
            subplts = struct('Pref',[],'Plt',[]);
            
            for ii = numel(obj.Agents):-1:1
                subplts(ii) = obj.Agents(ii).plot(simVisFig, 'Effective', true, 'LinearVelocity', true, 'WheelVelocity', true);
            end
            
            obj.Visualization.Fig = simVisFig;
            obj.Visualization.Plt = subplts;

            groundAxis = [obj.Field.x, obj.Field.y];
            axis(groundAxis)
            axis square
            xticks([obj.Field.x(1) 0 obj.Field.x(2)])
            yticks([obj.Field.y(1) 0 obj.Field.y(2)])
            xlabel('px [m]')
            ylabel('py [m]')
        end
        
        function visualizeORCA(obj, idxA)
            obj.Visualization.ORCA(idxA) = struct('self', [], 'global', []);
            A = obj.Agents(idxA);
            
            figure(obj.Visualization.Fig)
            obj.Visualization.ORCA(idxA).global = quiver(NaN, NaN, NaN, NaN, [A.color '-'], 'Marker', '.', 'AutoScale', 'off', 'MaxHeadSize', 2);
        end
        
        
        function visualizeVO(obj, idxA, idxB, mode)
            arguments
                obj
                idxA
                idxB
                mode = 0
            end
            
            [A, ~, O, R, voEdgeX1, voEdgeX2, k1, k2] = VO_quantities(obj, idxA, idxB);
            
            obj.Visualization.VO(idxA) = struct('self', [], 'global', []);

            if mode > 0
                if isempty(A.VOFig)
                    A.VOFig = figure;
                    plot_circle([0 0], A.vmax, 'eColor', 'k');
                    hold on
                else
                    figure(A.VOFig)
                end
                
                obj.Visualization.VO(idxA).self.O = plot_circle(O, R, 50, A.color);
                obj.Visualization.VO(idxA).self.A = plot(voEdgeX1(A.vmax), k1*voEdgeX1(A.vmax), [A.color '--']);
                obj.Visualization.VO(idxA).self.B = plot(voEdgeX2(A.vmax), k2*voEdgeX2(A.vmax), [A.color '--']);

                axis([-1 1 -1 1]*A.vmax)
                axis square

                xticks([-1 0 1]*A.vmax)
                yticks([-1 0 1]*A.vmax)
                xlabel('vx [m]')
                ylabel('vy [m]')
            else
                figure(obj.Visualization.Fig)
                obj.Visualization.VO(idxA).global.O = plot_circle(O+A.q, R, 50, A.color);
                obj.Visualization.VO(idxA).global.A = plot(A.q(1)+voEdgeX1(obj.Field.w), A.q(2)+k1*voEdgeX1(obj.Field.w), [A.color '--']);
                obj.Visualization.VO(idxA).global.B = plot(A.q(1)+voEdgeX2(obj.Field.w), A.q(2)+k2*voEdgeX2(obj.Field.w), [A.color '--']);
            end
        end
        
        function updateVisualVO(obj, idxA, idxB, mode)
            [A, ~, O, R, voEdgeX1, voEdgeX2, k1, k2] = VO_quantities(obj, idxA, idxB);

            if mode > 0
                update_circle(obj.Visualization.VO(idxA).self.O, O, R, 50);
                set(obj.Visualization.VO(idxA).self.A, 'XData', voEdgeX1(A.vmax), 'YData', k1*voEdgeX1(A.vmax));
                set(obj.Visualization.VO(idxA).self.B, 'XData', voEdgeX2(A.vmax), 'YData', k2*voEdgeX2(A.vmax));
            else
                update_circle(obj.Visualization.VO(idxA).global.O, O+A.q, R, 50);
                if isreal(k1) && isreal(k2)
                    set(obj.Visualization.VO(idxA).global.A, 'XData', A.q(1)+voEdgeX1(obj.Field.w), 'YData', A.q(2)+k1*voEdgeX1(obj.Field.w));
                    set(obj.Visualization.VO(idxA).global.B, 'XData', A.q(1)+voEdgeX2(obj.Field.w), 'YData', A.q(2)+k2*voEdgeX2(obj.Field.w));
                else
                    % once the center of robot enters VO
                    set(obj.Visualization.VO(idxA).global.A, 'XData', [], 'YData', []);
                    set(obj.Visualization.VO(idxA).global.B, 'XData', [], 'YData', []);
                end
            end
        end
        
        function [A, B, O, R, voEdgeX1, voEdgeX2, k1, k2] = VO_quantities(obj, idxA, idxB)
            arguments
                obj
                idxA
                idxB
            end
            
            A = obj.Agents(idxA);
            B = obj.Agents(idxB);
            tau = obj.TimeStep*obj.TimeHorizon;
            
            switch obj.MODEL
                case 'effective'
                    p_dif = B.p - A.p;
                    R_sum = A.R + B.R;
                case 'holonomic'
                    p_dif = B.q - A.q;
                    R_sum = A.r + B.r;
            end

            O = p_dif/tau;
            R = R_sum/tau;
            
            a = O(1); b = O(2); distO = norm(O);

            angA1 = atan(b/a); angB1 = asin(R/distO);
            k1 = tan(angA1-angB1);
            angA2 = atan(a/b); angB2 = asin(R/distO);
            k2 = tan(pi/2-angA2+angB2);
            
            k1 = max(min(realmax,k1),-realmax);
            k2 = max(min(realmax,k2),-realmax);

            vxDir = sign(a);
            
            p1 = [(k1^2+1) -2*(a+b*k1) distO-R^2];
            p1 = max(min(realmax,p1),-realmax);
            x_i1 = real(roots(p1));
            x_i1 = x_i1(1);
            
            p2 = [(k2^2+1) -2*(a+b*k2) distO-R^2];
            p2 = max(min(realmax,p2),-realmax);
            x_i2 = real(roots(p2));
            x_i2 = x_i2(1);
            
            if vxDir > 0
                voEdgeX1 = @(x) [x_i1 x];
                voEdgeX2 = @(x) [x_i2 x];
            else
                voEdgeX1 = @(x) [-x x_i1];
                voEdgeX2 = @(x) [-x x_i2];
            end
        end

    end
end


function plt = plot_circle(O, R, N, c, s)
arguments
    O % center, row vector, [x y]
    R % radius
    N = 50 % # of edges
    c = 'w'
    s.fAlpha = 0.05;
    s.eColor = 'none';
end
baseVertexAngs = linspace(0, 2*pi, N+1);
x = O(1) + R * cos(baseVertexAngs);
y = O(2) + R * sin(baseVertexAngs);
plt = fill(x, y, c);
plt.FaceAlpha = s.fAlpha;
plt.EdgeColor = s.eColor;
end

function update_circle(plt, O, R, N)
baseVertexAngs = linspace(0, 2*pi, N+1);
x = O(1) + R * cos(baseVertexAngs);
y = O(2) + R * sin(baseVertexAngs);
set(plt, 'XData', x, 'YData', y);
end