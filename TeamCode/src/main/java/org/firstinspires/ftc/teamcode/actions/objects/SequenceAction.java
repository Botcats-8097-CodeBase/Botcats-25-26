package org.firstinspires.ftc.teamcode.actions.objects;

import org.firstinspires.ftc.teamcode.actions.Action;

import java.util.ArrayList;
import java.util.List;

public class SequenceAction implements Action {
    private final List<Action> steps;
    private int index = 0;

    public SequenceAction(List<Action> steps) {
        this.steps = new ArrayList<>(steps);
    }

    @Override
    public void start() {
        index = 0;
        if (!steps.isEmpty()) steps.get(0).start();
    }

    @Override
    public boolean update() {
        if (index >= steps.size()) return true;

        Action current = steps.get(index);
        if (current.update()) {
            current.end();
            index++;
            if (index < steps.size()) steps.get(index).start();
        }
        return index >= steps.size();
    }

    @Override
    public void end() {
        if (index < steps.size()) steps.get(index).end();
    }
}