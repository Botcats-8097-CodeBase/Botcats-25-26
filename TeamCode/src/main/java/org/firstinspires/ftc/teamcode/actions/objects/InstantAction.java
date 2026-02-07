package org.firstinspires.ftc.teamcode.actions.objects;

import org.firstinspires.ftc.teamcode.actions.Action;

public class InstantAction implements Action {
    private final Runnable r;
    private boolean ran = false;

    public InstantAction(Runnable r) { this.r = r; }

    @Override public void start() { ran = false; }

    @Override
    public boolean update() {
        if (!ran) {
            ran = true;
            r.run();
        }
        return true;
    }

    @Override public void end() {}
}

