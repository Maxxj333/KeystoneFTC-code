package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class OtherCommands {

    // ---------------------------------------------------------------
    // WAIT <seconds>
    // ---------------------------------------------------------------
    public static class Wait extends AutoCommand {
        private final double duration;
        private double elapsed = 0;
        public Wait(double duration) { this.duration = duration; }
        @Override public void start() { elapsed = 0; }
        @Override public void execute(double dt) { elapsed += dt; }
        @Override public boolean isFinished() { return elapsed >= duration; }
    }

    // ---------------------------------------------------------------
    // SHOOT <seconds>
    // ---------------------------------------------------------------
    public static class Shoot extends AutoCommand {
        private final ShooterSubsystem shooter;
        private final IntakeSubsystem  intake;
        private final double feedDuration;
        private double feedElapsed = 0;
        private Phase phase = Phase.IDLE;

        private enum Phase { IDLE, SPINUP, FEEDING, SPINDOWN, DONE }

        public Shoot(ShooterSubsystem shooter, IntakeSubsystem intake, double feedDuration) {
            this.shooter = shooter;
            this.intake = intake;
            this.feedDuration = feedDuration;
        }

        @Override public void start() {
            phase = Phase.SPINUP;
            feedElapsed = 0;
            shooter.requestSpinUp();
        }

        @Override public void execute(double dt) {
            shooter.update();
            switch (phase) {
                case SPINUP:
                    if (shooter.isReady()) {
                        phase = Phase.FEEDING;
                        shooter.requestFeed();
                    }
                    break;
                case FEEDING:
                    intake.fireAssist();
                    feedElapsed += dt;
                    if (feedElapsed >= feedDuration) {
                        phase = Phase.SPINDOWN;
                        shooter.requestStop();
                    }
                    break;
                case SPINDOWN:
                    if (shooter.getState() == ShooterSubsystem.State.IDLE) {
                        phase = Phase.DONE;
                    }
                    break;
                default: break;
            }
        }

        @Override public boolean isFinished() { return phase == Phase.DONE; }

        @Override public void end(boolean interrupted) {
            if (interrupted) shooter.requestImmediateStop();
            intake.off();
        }
    }

    // ---------------------------------------------------------------
    // INTAKE <seconds> [power]
    // Runs intake for a fixed duration, then stops.
    // ---------------------------------------------------------------
    public static class Intake extends AutoCommand {
        private final IntakeSubsystem intake;
        private final double duration;
        private final double power;
        private double elapsed = 0;

        public Intake(IntakeSubsystem intake, double duration, double power) {
            this.intake = intake;
            this.duration = duration;
            this.power = power;
        }

        @Override public void start() {
            elapsed = 0;
            intake.setPower(power);
        }
        @Override public void execute(double dt) { elapsed += dt; }
        @Override public boolean isFinished() { return elapsed >= duration; }
        @Override public void end(boolean interrupted) { intake.off(); }
    }

    // ---------------------------------------------------------------
    // INTAKE / INTAKE_HOLD [power]  (NEW)
    // Runs intake indefinitely. Stops only when the parent group
    // cancels it (typically RACE finishing its other child, or PAR
    // being cancelled). isFinished() always returns false.
    // ---------------------------------------------------------------
    public static class IntakeHold extends AutoCommand {
        private final IntakeSubsystem intake;
        private final double power;

        public IntakeHold(IntakeSubsystem intake, double power) {
            this.intake = intake;
            this.power = power;
        }

        @Override public void start() { intake.setPower(power); }
        @Override public void execute(double dt) { /* nothing — runs forever */ }
        @Override public boolean isFinished() { return false; }   // KEY
        @Override public void end(boolean interrupted) { intake.off(); }
    }

    // ---------------------------------------------------------------
    // SERVO_ACTIVE / SERVO_DEFAULT (instant)
    // ---------------------------------------------------------------
    public static class ServoActive extends AutoCommand {
        private final ServoSubsystem servos;
        private boolean done = false;
        public ServoActive(ServoSubsystem servos) { this.servos = servos; }
        @Override public void start() { servos.toActive(); done = true; }
        @Override public void execute(double dt) {}
        @Override public boolean isFinished() { return done; }
    }

    public static class ServoDefault extends AutoCommand {
        private final ServoSubsystem servos;
        private boolean done = false;
        public ServoDefault(ServoSubsystem servos) { this.servos = servos; }
        @Override public void start() { servos.toDefault(); done = true; }
        @Override public void execute(double dt) {}
        @Override public boolean isFinished() { return done; }
    }
}