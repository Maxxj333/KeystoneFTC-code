package org.firstinspires.ftc.teamcode.commands;

import java.util.Set;

public class ConditionalVision extends AutoCommand {

    public static volatile String lastDecisionLog = "(none)";
    public AutoCommand getThenCmd() { return thenCmd; }
    public AutoCommand getElseCmd() { return elseCmd; }

    private enum Phase { WAITING, RUNNING_CHILD, DONE }

    private final TagFilter filter;
    private final double timeoutSec;
    private final AutoCommand thenCmd;
    private final AutoCommand elseCmd;
    private final VisionSource vision;
    private final long stalenessMs;

    private Phase phase = Phase.WAITING;
    private double elapsed = 0;
    private AutoCommand chosen = null;
    private boolean choseThen = false;

    public ConditionalVision(TagFilter filter, double timeoutSec,
                             AutoCommand thenCmd, AutoCommand elseCmd,
                             VisionSource vision, long stalenessMs) {
        this.filter = filter;
        this.timeoutSec = timeoutSec;
        this.thenCmd = thenCmd;
        this.elseCmd = elseCmd;
        this.vision = vision;
        this.stalenessMs = stalenessMs;
    }

    public boolean choseThenBranch() { return choseThen; }

    @Override public void start() {
        phase = Phase.WAITING;
        elapsed = 0;
        chosen = null;
        choseThen = false;
    }

    @Override public void execute(double dt) {
        switch (phase) {
            case WAITING: {
                elapsed += dt;
                Set<Integer> visible = vision.getVisibleTagIds(stalenessMs);
                boolean matches = filter.anyMatch(visible);
                if (matches) {
                    choseThen = true;
                    chosen = thenCmd;
                    lastDecisionLog = filter + " -> THEN";
                    chosen.start();
                    phase = Phase.RUNNING_CHILD;
                } else if (elapsed >= timeoutSec) {
                    choseThen = false;
                    chosen = elseCmd;
                    lastDecisionLog = filter + " -> ELSE";
                    chosen.start();
                    phase = Phase.RUNNING_CHILD;
                }
                break;
            }
            case RUNNING_CHILD: {
                if (chosen != null) {
                    chosen.execute(dt);
                    if (chosen.isFinished()) {
                        chosen.end(false);
                        phase = Phase.DONE;
                    }
                }
                break;
            }
            case DONE:
                break;
        }
    }

    @Override public boolean isFinished() { return phase == Phase.DONE; }

    @Override public void end(boolean interrupted) {
        if (interrupted && chosen != null && !chosen.isFinished()) {
            chosen.cancel();
        }
    }
}