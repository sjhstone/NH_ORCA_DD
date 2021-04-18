function plts = plot(obj, fig_h, NameValPairs)
    arguments
        obj RVO2.robotModel.TurtleBot2
        fig_h matlab.ui.Figure
        NameValPairs.EffectiveBase logical = false
        NameValPairs.LinearVelocity logical = false
        NameValPairs.WheelVelocity logical = false
        NameValPairs.qTrajectory logical = true
    end
    plts.Pref = NameValPairs;

    patch = obj.Patch;

    figure(fig_h);
    hold on

    plts.Plt.Base = plot(patch.Base.x, patch.Base.y, [obj.color '-']);
    plts.Plt.WheelL = fill(patch.WheelL.x, patch.WheelL.y, obj.color, 'LineStyle', 'none');
    plts.Plt.WheelR = fill(patch.WheelR.x, patch.WheelR.y, obj.color, 'LineStyle', 'none');

    if NameValPairs.EffectiveBase
        patch_effective = obj.EffectivePatch;
        plts.Plt.EffectiveBase = plot(patch_effective.Base.x, patch_effective.Base.y, [obj.color ':']);
    end

    if NameValPairs.LinearVelocity
        plts.Plt.LinearVelocity = quiver(obj.q(1), obj.q(2), obj.v(1), obj.v(2), 1.0, [obj.color '-'], 'Marker', '.', 'AutoScale', 'off', 'MaxHeadSize', 2);
    end

    if NameValPairs.EffectiveBase && NameValPairs.LinearVelocity
        plts.Plt.LinearEffectiveVelocity = quiver(obj.p(1), obj.p(2), obj.vEffective(1), obj.vEffective(2), 1.0, [obj.color ':'], 'Marker', '.', 'AutoScale', 'off', 'MaxHeadSize', 2);
    end

    if NameValPairs.WheelVelocity
        plts.Plt.WheelVelocityL = quiver(patch.WheelL.O(1), patch.WheelL.O(2), obj.vL*cos(obj.theta), obj.vL*sin(obj.theta), [obj.color '-']);
        plts.Plt.WheelVelocityR = quiver(patch.WheelR.O(1), patch.WheelR.O(2), obj.vR*cos(obj.theta), obj.vR*sin(obj.theta), [obj.color '-']);
    end

    if NameValPairs.qTrajectory
        plts.Plt.qTrajectory = plot(obj.Trj_q(:,1), obj.Trj_q(:,2), [obj.color '-.']);
    end
end