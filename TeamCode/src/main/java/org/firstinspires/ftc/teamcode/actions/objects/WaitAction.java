package org.firstinspires.ftc.teamcode.actions.objects;

/**
 * Waits for a fixed amount of time without blocking.
 */
public class WaitAction extends TimedAction {
    private final double seconds;

    public WaitAction(double seconds) {
        this.seconds = Math.max(0, seconds);
    }

    @Override
    public boolean update() {
        return timer.seconds() >= seconds;
    }

    @Override public void end() {}
}

