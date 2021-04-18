classdef TurtleBot2 < handle
    % Velocity root: TwistLin, TwistAng
    % vX, vY
    % vx, vy
    
    properties
        ID
        vPref
        q
        theta
        color
        
        VOFig
        ORCALines
    end
    
    properties (Constant)%, Access=private)
        r = 0.175;    % radius
        L = 0.23;     % distance between wheel pairs
        ThWh = 0.024; % thickness of wheel
        RWh = 0.0176; % wheel surface radius
        
        vmax = 0.65;
        wmax = pi;
    end

    properties (Access=private)
        TwistLin % TwistLinY is useless
        TwistAng
        Trj_q
        Trj_v
    end
    
    properties (Dependent)
        p
        R
        vEffective
    end
    
    properties (Dependent, Access=private)
        Patch
        EffectivePatch
        RotMat
        v
        vL
        vR
    end
    
    methods
        function obj = TurtleBot2(id, q, theta, twist_lin, twist_ang, colorSpec)
            arguments
                id = NaN
                q(:,2) double = [0 0]
                theta double = 0
                twist_lin double = 0
                twist_ang double = 0
                colorSpec = 'k'
            end
            
            if nargin > 0 % enable array initialization
                assert(max(abs(twist_lin)) <= obj.vmax, 'Max linear velocity must not > %f', obj.vmax);
                assert(max(abs(twist_ang)) <= obj.wmax);
                N = numel(id);
                for ii = N:-1:1
                    obj(ii).ID = id(ii);
                    obj(ii).q = q(ii,:);
                    obj(ii).Trj_q = q(ii,:);
                    obj(ii).theta = theta(ii);
                    obj(ii).TwistLin = twist_lin(ii);
                    obj(ii).TwistAng = twist_ang(ii);
                    obj(ii).color = colorSpec{ii};
                end
                
                for ii = 1:N
                    fprintf('TurtleBot2:%d initialized\t(x=%f,y=%f), θ=%f, v=%f, ω=%f.\n', id(ii), q(ii,1), q(ii,2), theta(ii), twist_lin(ii), twist_ang(ii,:));
                end
            end
        end
        
        function p = get.p(obj)
            p = obj.q + [obj.r*cos(obj.theta), obj.r*sin(obj.theta)];
            if any(isnan(p))
                warning(obj.r);
                warning(obj.theta)
            end
        end
        
        function R = get.R(obj)
            R = 2*obj.r;
        end
        
        function RotMat = get.RotMat(obj)
            RotMat = [cos(obj.theta) -sin(obj.theta);
                      sin(obj.theta)  cos(obj.theta)];
        end
        
        function Patch = get.Patch(obj)
            nEdges = 24;
            
            Patch = struct('Base',struct('x',[],'y',[]), ...
                'WheelL',struct('x',[],'y',[]), ...
                'WheelR',struct('x',[],'y',[]));
            
            baseVertexAngs = linspace(0, 2*pi, nEdges+1);
            qBase_x = obj.r * cos(baseVertexAngs);
            qBase_y = obj.r * sin(baseVertexAngs);
            qBase = obj.RotMat * [qBase_x; qBase_y];
            
            Patch.Base.x = obj.q(1) + qBase(1,:);
            Patch.Base.y = obj.q(2) + qBase(2,:);
            
            qWheel_x = [1 1 -1 -1]*obj.RWh;
            qWheel_y = 0.5*[1 -1 -1 1]*obj.ThWh;
            
            qWheelL_y = qWheel_y + obj.L/2;
            qWheelR_y = qWheel_y - obj.L/2;
            
            qWheelL = obj.RotMat * [qWheel_x; qWheelL_y];
            qWheelR = obj.RotMat * [qWheel_x; qWheelR_y];
            
            Patch.WheelL.x = obj.q(1) + qWheelL(1,:);
            Patch.WheelL.y = obj.q(2) + qWheelL(2,:);
            Patch.WheelL.O = obj.q + (obj.RotMat * [0; obj.L/2])';
            
            Patch.WheelR.x = obj.q(1) + qWheelR(1,:);
            Patch.WheelR.y = obj.q(2) + qWheelR(2,:);
            Patch.WheelR.O = obj.q + (obj.RotMat * [0;-obj.L/2])';
        end
        
        function Patch = get.EffectivePatch(obj)
            nEdges = 48;
            
            Patch = struct('Base',struct('x',[],'y',[]));
            
            baseVertexAngs = linspace(0, 2*pi, nEdges+1);
            qBase_x = obj.R * cos(baseVertexAngs);
            qBase_y = obj.R * sin(baseVertexAngs);
            qBase = obj.RotMat * [qBase_x; qBase_y];
            
            Patch.Base.x = obj.p(1) + qBase(1,:);
            Patch.Base.y = obj.p(2) + qBase(2,:);
        end
        
        function v = get.v(obj)
            v = [cos(obj.theta) sin(obj.theta)] .* obj.TwistLin;
        end
        
        function vL = get.vL(obj)
            vL = obj.diffWheelVelocity(1);
        end
        
        function vR = get.vR(obj)
            vR = obj.diffWheelVelocity(2);
        end
        
        function vEffective = get.vEffective(obj)
            o = obj.theta;
            M_aff = [cos(o) cos(o); sin(o) sin(o)]/2 + obj.r/obj.L*[sin(o) -sin(o);-cos(o) cos(o)];
            vEffective = M_aff*obj.diffWheelVelocity(1:2);
        end
        
        function vx = diffWheelVelocity(obj, ii)
            vWheel = [1/2 1/2; -1/obj.L 1/obj.L]\[obj.TwistLin; obj.TwistAng];
            vx = vWheel(ii);
        end

        
        function doStep(obj, dt)
            obj.q = obj.q + dt*obj.v;
            
            if isnan(obj.TwistAng)
                obj.TwistAng = 0;
                warning('Inf TwistAng');
            end
            obj.theta = mod(obj.theta + dt*obj.TwistAng, 2*pi);
            
            obj.Trj_q = [obj.Trj_q; obj.q];
        end
        
        computeNewVelocity(obj, B, prefVel, dt, N, vis_handle, MODEL)
        plts = plot(obj, fig_h, NameValPairs)
        updatePlot(obj, plts)
        
    end
    
end
