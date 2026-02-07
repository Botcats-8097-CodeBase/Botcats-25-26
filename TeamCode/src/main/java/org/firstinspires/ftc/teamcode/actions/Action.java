package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public interface Action {
    void start();

    boolean update();

    void end();
}

