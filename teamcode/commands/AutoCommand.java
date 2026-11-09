package org.firstinspires.ftc.teamcode.commands;

public abstract class AutoCommand {
    public abstract void start();
    public abstract void execute(double dt);
    public abstract boolean isFinished();
    public void end(boolean interrupted) {}
    public void cancel() { end(true); }
}