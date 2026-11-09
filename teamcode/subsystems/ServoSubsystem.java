package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConfig;

public class ServoSubsystem {

    public enum State { DEFAULT, ACTIVE }

    private final Servo left, right;
    private State state = State.DEFAULT;

    public ServoSubsystem(HardwareMap hw) {
        left  = hw.get(Servo.class, RobotConfig.LEFT_SERVO_NAME);
        right = hw.get(Servo.class, RobotConfig.RIGHT_SERVO_NAME);
        left.setDirection(Servo.Direction.REVERSE);
        right.setDirection(Servo.Direction.FORWARD);
        toDefault();
    }

    public void toDefault() {
        state = State.DEFAULT;
        left.setPosition(RobotConfig.LEFT_SERVO_DEFAULT);
        right.setPosition(RobotConfig.RIGHT_SERVO_DEFAULT);
    }

    public void toActive() {
        state = State.ACTIVE;
        left.setPosition(RobotConfig.LEFT_SERVO_ACTIVE);
        right.setPosition(RobotConfig.RIGHT_SERVO_ACTIVE);
    }

    public State getState() { return state; }
}