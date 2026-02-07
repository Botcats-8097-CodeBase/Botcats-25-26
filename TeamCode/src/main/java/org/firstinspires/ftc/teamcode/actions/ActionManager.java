package org.firstinspires.ftc.teamcode.actions;

import java.util.ArrayList;
import java.util.List;

public class ActionManager {
    private static final List<Action> active = new ArrayList<>();

    public static void schedule(Action action) {
        action.start();
        active.add(action);
    }

    public static void schedule(ActionBuilder builder) {
        schedule(builder.build());
    }

    public static void update() {
        for (int i = 0; i < active.size(); ) {
            Action a = active.get(i);

            if (a.update()) {
                a.end();
                active.remove(i);
            } else {
                i++;
            }
        }
    }

    public static void cancelAll() {
        for (Action a : active) a.end();
        active.clear();
    }

    public static boolean isIdle() {
        return active.isEmpty();
    }
}

