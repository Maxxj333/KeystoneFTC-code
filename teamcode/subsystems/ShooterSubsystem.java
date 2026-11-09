package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class ShooterSubsystem {

    public enum State { IDLE, SPINNING, FEEDING, SPIN_DOWN }

    private final DcMotorEx shooter;
    private final DcMotorEx shooter2;
    private final DcMotor feeder;
    private final double ticksPerRev;
    private final double targetTicksPerSec;

    private State state = State.IDLE;
    private double  spinDownStartSec = 0;
    private boolean ready = false;

    public ShooterSubsystem(HardwareMap hw) {
        shooter  = hw.get(DcMotorEx.class, RobotConfig.SHOOTER_NAME);
        shooter2 = hw.get(DcMotorEx.class, RobotConfig.SHOOTER2_NAME);
        feeder   = hw.get(DcMotor.class,   RobotConfig.FEEDER_NAME);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(
                RobotConfig.SHOOTER_kP, RobotConfig.SHOOTER_kI,
                RobotConfig.SHOOTER_kD, RobotConfig.SHOOTER_kF);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setVelocityPIDFCoefficients(
                RobotConfig.SHOOTER_kP, RobotConfig.SHOOTER_kI,
                RobotConfig.SHOOTER_kD, RobotConfig.SHOOTER_kF);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setPower(0);

        double tpr = shooter.getMotorType().getTicksPerRev();
        if (tpr <= 0) tpr = RobotConfig.SHOOTER_TICKS_FALLBACK;
        ticksPerRev = tpr;
        targetTicksPerSec = RobotConfig.SHOOTER_TARGET_RPM / 60.0 * ticksPerRev;
    }

    public void requestSpinUp() { state = State.SPINNING; }

    public void requestFeed() { state = State.FEEDING; }

    public void requestStopFeed() {
        if (state == State.FEEDING) state = State.SPINNING;
    }

    public void requestStop() {
        if (state == State.IDLE || state == State.SPIN_DOWN) return;
        state = State.SPIN_DOWN;
        spinDownStartSec = nowSec();
    }

    public void requestImmediateStop() {
        state = State.IDLE;
        shooter.setPower(0);
        shooter2.setPower(0);
        feeder.setPower(0);
        ready = false;
    }

    public void update() {
        switch (state) {
            case IDLE:
                shooter.setPower(0);
                shooter2.setPower(0);
                feeder.setPower(0);
                ready = false;
                break;

            case SPINNING:
            case FEEDING:
                shooter.setVelocity(targetTicksPerSec);
                shooter2.setVelocity(targetTicksPerSec);
                updateReadyFlag();
                if (state == State.FEEDING && ready) {
                    feeder.setPower(RobotConfig.FEEDER_MAX_POWER);
                } else {
                    feeder.setPower(0);
                }
                break;

            case SPIN_DOWN:
                shooter.setVelocity(targetTicksPerSec);
                shooter2.setVelocity(targetTicksPerSec);
                feeder.setPower(0);
                if (nowSec() - spinDownStartSec >= RobotConfig.SHOOTER_SPINDOWN_S) {
                    state = State.IDLE;
                    shooter.setPower(0);
                    shooter2.setPower(0);
                    ready = false;
                }
                break;
        }
    }

    private void updateReadyFlag() {
        double rpm = getDrumRpm();
        if (rpm >= RobotConfig.SHOOTER_FIRE_RPM)          ready = true;
        else if (rpm < RobotConfig.SHOOTER_FIRE_RPM_LOW)  ready = false;
    }

    public State   getState()      { return state; }
    public boolean isReady()       { return ready; }
    public double  getShooterRpm() { return shooter.getVelocity()  / ticksPerRev * 60.0; }
    public double  getShooter2Rpm(){ return shooter2.getVelocity() / ticksPerRev * 60.0; }
    public double  getDrumRpm()    { return Math.min(getShooterRpm(), getShooter2Rpm()); }
    public double  getFeederPower(){ return feeder.getPower(); }

    private static double nowSec() { return System.nanoTime() / 1.0e9; }
}