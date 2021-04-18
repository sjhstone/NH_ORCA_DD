function computeNewVelocity(obj, B, prefVel, dt, N, vis_handle, MODEL)

A = obj;

if all(prefVel == 0)
    A.TwistLin = 0;
    A.TwistAng = 0;
else
    switch MODEL
        case 'effective'
            relPos = B.p - A.p;
            relVel = A.vEffective - B.vEffective;
            relVel = relVel';
            sumRadius = A.R + B.R;
            prefVel = prefVel + [A.r B.r].*[-sin(A.theta)*A.TwistAng cos(B.theta)*B.TwistAng];
        case 'holonomic'
            relPos = B.q - A.q;
            relVel = A.v - B.v;
            sumRadius = A.r + B.r;
            prefVel = 1*prefVel;
            disp(prefVel);
    end

    line = struct;

    if norm(relPos)^2 > sumRadius^2
        w = relVel - relPos/N;
        wLenSq = dot(w,w);
        dotProd1 = dot(w,relPos);

        if dotProd1 < 0 && dotProd1 > sumRadius^2 * wLenSq
            % project on cut-off circle
            unitW = w/norm(w);
            line.direction = [unitW(2) -unitW(1)];
            u = (sumRadius/N - norm(w))*unitW;
        else
            % project on legs
            leg = sqrt(norm(relPos)^2 - sumRadius^2);
            if det([relPos;w]) > 0
                % project on left leg
                line.direction = [relPos(1)*leg-relPos(2)*sumRadius, relPos(1)*sumRadius+relPos(2)*leg]/norm(relPos)^2;
            else
                % project on right leg
                line.direction = -[relPos(1)*leg+relPos(2)*sumRadius, -relPos(1)*sumRadius+relPos(2)*leg]/norm(relPos)^2;
            end
            dotProd2 = dot(relVel, line.direction);
            u = dotProd2 * line.direction - relVel;
        end
    else
        disp('Collision');
        w = relVel - relPos/dt;
        unitW = w/norm(w);

        line.direction = [unitW(2) -unitW(1)];
        u = (sumRadius/dt - norm(w))*unitW;
    end

    line.point = A.v + u/2;
    
    ORCApt = A.RotMat * line.point' + A.p;
    ORCAdr = A.RotMat * line.direction';
    
    set(vis_handle.ORCA(A.ID).global, ...
        'XData', ORCApt(1), 'YData', ORCApt(2),...
        'UData', ORCAdr(1), 'VData', ORCAdr(2));
    
    
    [newVel, flag_success] = linearProgram2(line, A.R, prefVel, false);

    directionDeviation = atan(newVel(2)/newVel(1))-A.theta;

    if ~flag_success
        warning('Using lp-3 to compute');
        newVel = linearProgram3(line, A.vmax, newVel);
    else
        disp(newVel);
    end
    
    switch MODEL
        case 'effective'
            o = obj.theta;
            M_aff = [cos(o) cos(o); sin(o) sin(o)]/2 + obj.r/obj.L*[sin(o) -sin(o);-cos(o) cos(o)];
            ctrlCmd = [1/2 1/2; -1/obj.L 1/obj.L] * (M_aff\newVel');
            A.TwistLin = min(ctrlCmd(1),A.vmax);
            A.TwistAng = ctrlCmd(2)/norm(ctrlCmd(2))*A.wmax;
        case 'holonomic'
            A.TwistLin = min(norm(newVel),A.vmax);
            A.TwistAng = directionDeviation/norm(directionDeviation)*A.wmax;
    end
end

end


function result = linearProgram3(ORCAline, radius, result)

if det([ORCAline.direction; ORCAline.point-result]) > 0
    warning('Result does not satisfy constraint of line');
end

end


function [result, flag_success] = linearProgram2(ORCAline, radius, optVelocity, flag_directionOpt)

if flag_directionOpt
    % Optimize direction
    result = optVelocity*radius;
elseif norm(optVelocity)^2 > radius^2
    % Optimize closest point and outside circle
    result = optVelocity/norm(optVelocity) * radius;
else
    % Optimize closest point and inside circle
    result = optVelocity;
end

% assume only 2 robots, only 1 ORCA line
if det([ORCAline.direction;ORCAline.point-result]) > 0
    tmpResult = result;
    
    [result, flag_success] = linearProgram1(ORCAline, radius, optVelocity, flag_directionOpt);
    if ~flag_success
        result = tmpResult;
    end
else
    flag_success = false;
end

end


function [result, flag_success] = linearProgram1(ORCAline, radius, optVelocity, flag_directionOpt)

result = [];

dotProd = dot(ORCAline.point, ORCAline.direction);
discriminant = dotProd^2 + radius^2 - norm(ORCAline.point)^2;

if discriminant < 0
    % max speed circle fully invalidates line
    flag_success = false;
    return
end

tL = -dotProd - sqrt(discriminant);
tR = -dotProd + sqrt(discriminant);

if flag_directionOpt
    if optVelocity*ORCAline.direction > 0
        % take right extreme
        result = ORCAline.point + tRight*ORCAline.direction;
    else
        result = ORCAline.point + tLeft*ORCAline.direction;
    end
else
    % optimize closest point
    t = dot(ORCAline.direction, optVelocity-ORCAline.point);
    if t < tL
        result = ORCAline.point + tL*ORCAline.direction;
    elseif t > tR
        result = ORCAline.point + tR*ORCAline.direction;
    else
        result = ORCAline.point + t*ORCAline.direction;
    end
end

flag_success = true;
end
