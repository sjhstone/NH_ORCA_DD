% AUTHOR: Jiahe Shi, 2020

function updatePlot(obj, plts)
    pref = plts.Pref;
    plt = plts.Plt;

    patch = obj.Patch;

    set(plt.Base, 'XData', patch.Base.x, 'YData', patch.Base.y);
    set(plt.WheelL, 'XData', patch.WheelL.x, 'YData', patch.WheelL.y);
    set(plt.WheelR, 'XData', patch.WheelR.x, 'YData', patch.WheelR.y);

    if pref.EffectiveBase
        patch_effective = obj.EffectivePatch;
        set(plt.EffectiveBase, 'XData', patch_effective.Base.x, 'YData', patch_effective.Base.y);
    end

    if pref.LinearVelocity
        set(plt.LinearVelocity, 'XData', obj.q(1), 'YData', obj.q(2), 'UData', obj.v(1), 'VData', obj.v(2));
    end

    if pref.EffectiveBase && pref.LinearVelocity
        set(plt.LinearEffectiveVelocity, 'XData', obj.p(1), 'YData', obj.p(2), 'UData', obj.vEffective(1), 'VData', obj.vEffective(2));
    end

    if pref.WheelVelocity
        set(plt.WheelVelocityL, 'XData', patch.WheelL.O(1), 'YData', patch.WheelL.O(2), 'UData', obj.vL*cos(obj.theta), 'VData', obj.vL*sin(obj.theta));
        set(plt.WheelVelocityR, 'XData', patch.WheelR.O(1), 'YData', patch.WheelR.O(2), 'UData', obj.vR*cos(obj.theta), 'VData', obj.vR*sin(obj.theta));
    end

    if pref.qTrajectory
        set(plt.qTrajectory, 'XData', obj.Trj_q(:,1), 'YData', obj.Trj_q(:,2));
    end
end