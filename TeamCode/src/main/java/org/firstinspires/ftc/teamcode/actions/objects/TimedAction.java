package org.firstinspires.ftc.teamcode.actions.objects;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.Action;

public abstract class TimedAction implements Action {
    protected final ElapsedTime timer = new ElapsedTime();

    @Override
    public void start() {
        timer.reset();
        onStart();
    }

    protected void onStart() {}
}