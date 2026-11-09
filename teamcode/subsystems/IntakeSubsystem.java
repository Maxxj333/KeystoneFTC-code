package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class IntakeSubsystem {

    private final DcMotor intake;
    private double currentPower = 0;

    public IntakeSubsystem(HardwareMap hw) {
        intake = hw.get(DcMotor.class, RobotConfig.INTAKE_NAME);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setPower(0);
    }

    public void on()          { setPowerInternal(RobotConfig.INTAKE_MAX_POWER); }
    public void off()         { setPowerInternal(0); }
    public void fireAssist()  { setPowerInternal(RobotConfig.INTAKE_FIRE_POWER); }
    public void setPower(double p) { setPowerInternal(p); }

    /**
     * Teleop ramp. Call each loop.
     * @param dt                seconds since last call
     * @param buttonPressed     manual intake button state
     * @param fireAssistOverride true -> force fire-assist power, ignoring button
     */
    public void updateRamp(double dt, boolean buttonPressed, boolean fireAssistOverride) {
        if (fireAssistOverride) {
            currentPower = RobotConfig.INTAKE_FIRE_POWER;
        } else if (buttonPressed) {
            currentPower += RobotConfig.INTAKE_RAMP_RATE * dt;
            if (currentPower > RobotConfig.INTAKE_MAX_POWER) currentPower = RobotConfig.INTAKE_MAX_POWER;
        } else {
            currentPower -= RobotConfig.INTAKE_RAMP_RATE * dt;
            if (currentPower < 0) currentPower = 0;
        }
        intake.setPower(currentPower);
    }

    public double getPower() { return currentPower; }

    private void setPowerInternal(double p) {
        currentPower = p;
        intake.setPower(p);
    }
}