package org.firstinspires.ftc.teamcode.actions.objects;

import java.util.function.DoubleConsumer;

public class LoopForTimeAction extends TimedAction {
    private final double duration;
    private final DoubleConsumer loopFn;

    public LoopForTimeAction(
            double durationSeconds,
            DoubleConsumer loopFn
    ) {
        this.duration = Math.max(0, durationSeconds);
        this.loopFn = loopFn;
    }

    @Override
    public boolean update() {
        double t = timer.seconds();

        if (t < duration) {
            loopFn.accept(t);
            return false;
        }

        return true;
    }

    @Override
    public void end() {}
}

