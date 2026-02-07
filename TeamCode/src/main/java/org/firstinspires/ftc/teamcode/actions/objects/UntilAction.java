package org.firstinspires.ftc.teamcode.actions.objects;

import org.firstinspires.ftc.teamcode.actions.Action;

import java.util.function.BooleanSupplier;

public class UntilAction implements Action {
    private final BooleanSupplier condition;

    public UntilAction(BooleanSupplier condition) { this.condition = condition; }

    @Override public void start() {}

    @Override
    public boolean update() {
        return condition.getAsBoolean();
    }

    @Override public void end() {}
}

