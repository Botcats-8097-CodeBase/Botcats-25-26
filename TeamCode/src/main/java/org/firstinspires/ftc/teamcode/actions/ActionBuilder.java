package org.firstinspires.ftc.teamcode.actions;

import org.firstinspires.ftc.teamcode.actions.objects.InstantAction;
import org.firstinspires.ftc.teamcode.actions.objects.LoopForTimeAction;
import org.firstinspires.ftc.teamcode.actions.objects.LoopUntilAction;
import org.firstinspires.ftc.teamcode.actions.objects.SequenceAction;
import org.firstinspires.ftc.teamcode.actions.objects.UntilAction;
import org.firstinspires.ftc.teamcode.actions.objects.WaitAction;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

public class ActionBuilder {
    private final List<Action> steps = new ArrayList<>();

    public ActionBuilder doNow(Runnable r) {
        steps.add(new InstantAction(r));
        return this;
    }

    public ActionBuilder wait(double seconds) {
        steps.add(new WaitAction(seconds));
        return this;
    }

    public ActionBuilder waitUntil(BooleanSupplier condition) {
        steps.add(new UntilAction(condition));
        return this;
    }

    public ActionBuilder loopFor(DoubleConsumer r, double durationSeconds) {
        steps.add(new LoopForTimeAction(durationSeconds, r));
        return this;
    }

    public ActionBuilder loopUntil(Runnable r, BooleanSupplier condition) {
        steps.add(new LoopUntilAction(r, condition));
        return this;
    }

    public ActionBuilder run(Action action) {
        steps.add(action);
        return this;
    }

    public Action build() {
        return new SequenceAction(steps);
    }
}
