package org.firstinspires.ftc.teamcode.actions.objects;

import java.util.function.BooleanSupplier;

public class LoopUntilAction extends TimedAction {
    private final Runnable loop;
    private final BooleanSupplier done;

    public LoopUntilAction(Runnable loop, BooleanSupplier done) {
        this.loop = loop;
        this.done = done;
    }

    @Override public void start() {}

    @Override
    public boolean update() {
        loop.run();
        return done.getAsBoolean();
    }

    @Override public void end() {}
}
