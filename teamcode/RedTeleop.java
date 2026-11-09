package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;

import java.util.List;

@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleop extends LinearOpMode {

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private Limelight3A limelight;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private ServoSubsystem servos;

    private double headingOffsetRad = 0.0;
    private boolean isFieldCentric = false;
    private boolean prevX = false, prevY = false, prevA = false, prevB = false;

    private int lastFlPos = 0, lastFrPos = 0, lastBlPos = 0, lastBrPos = 0;
    private double odoY = 0;
    private double odoYAtLastAccept = 0;
    private double lastAcceptedY = Double.NaN;
    private long   lastAcceptedMs = 0;

    private double lastRawVisX, lastRawVisY, lastRawVisTh;
    private double lastSelVisX, lastSelVisY, lastSelVisTh;
    private String lastCandidateLabel = "raw";
    private double lastHiveTiltDeg = Double.NaN;
    private double lastScoreRaw = 0, lastScoreFlipped = 0;
    private double lastExpectedY = Double.NaN;
    private boolean lastWasFlipped = false;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, RobotConfig.FL_NAME);
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, RobotConfig.BL_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, RobotConfig.FR_NAME);
        backRightMotor  = hardwareMap.get(DcMotorEx.class, RobotConfig.BR_NAME);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(RobotConfig.DRIVE_P, RobotConfig.DRIVE_I,
                            RobotConfig.DRIVE_D, RobotConfig.DRIVE_F));
        }

        shooter = new ShooterSubsystem(hardwareMap);
        intake  = new IntakeSubsystem(hardwareMap);
        servos  = new ServoSubsystem(hardwareMap);

        imu = hardwareMap.get(IMU.class, RobotConfig.IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        limelight = hardwareMap.get(Limelight3A.class, RobotConfig.LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(RobotConfig.LIMELIGHT_PIPELINE);
        limelight.start();

        SlewRateLimiter limitY = new SlewRateLimiter(RobotConfig.ACCEL_LIMIT_MPS2);
        SlewRateLimiter limitX = new SlewRateLimiter(RobotConfig.ACCEL_LIMIT_MPS2);
        SlewRateLimiter limitOmega = new SlewRateLimiter(RobotConfig.ACCEL_LIMIT_MPS2 / RobotConfig.K);

        lastFlPos = frontLeftMotor.getCurrentPosition();
        lastFrPos = frontRightMotor.getCurrentPosition();
        lastBlPos = backLeftMotor.getCurrentPosition();
        lastBrPos = backRightMotor.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        double lastTime = getRuntime();

        while (opModeIsActive()) {

            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            // Odometry Y (pure encoder)
            int fl = frontLeftMotor.getCurrentPosition();
            int fr = frontRightMotor.getCurrentPosition();
            int bl = backLeftMotor.getCurrentPosition();
            int br = backRightMotor.getCurrentPosition();
            double dFl = (fl - lastFlPos) / RobotConfig.TICKS_PER_METER;
            double dFr = (fr - lastFrPos) / RobotConfig.TICKS_PER_METER;
            double dBl = (bl - lastBlPos) / RobotConfig.TICKS_PER_METER;
            double dBr = (br - lastBrPos) / RobotConfig.TICKS_PER_METER;
            lastFlPos = fl; lastFrPos = fr; lastBlPos = bl; lastBrPos = br;
            double dVy = (dFl + dFr + dBl + dBr) / 4.0;
            double dVx = (dFl - dFr - dBl + dBr) / 4.0;
            double heading = getRawHeadingRad() - headingOffsetRad;
            double dYglobal = -dVx * Math.cos(heading) + dVy * Math.sin(heading);
            odoY += dYglobal;

            // DRIVE
            boolean cX = gamepad1.x, cY = gamepad1.y;
            if (cX && !prevX) { isFieldCentric = true;  headingOffsetRad = getRawHeadingRad(); }
            if (cY && !prevY) { isFieldCentric = false; }
            prevX = cX; prevY = cY;

            double rawFieldVy = -gamepad1.left_stick_y * RobotConfig.MAX_LINEAR_VELOCITY_MPS;
            double rawFieldVx =  gamepad1.left_stick_x * RobotConfig.MAX_LINEAR_VELOCITY_MPS;
            double rawOmega   = -gamepad1.right_stick_x * (RobotConfig.MAX_LINEAR_VELOCITY_MPS / RobotConfig.K);

            double targetRobotVy, targetRobotVx;
            if (isFieldCentric) {
                double h = getRawHeadingRad() - headingOffsetRad;
                targetRobotVy = rawFieldVy * Math.cos(h) - rawFieldVx * Math.sin(h);
                targetRobotVx = rawFieldVy * Math.sin(h) + rawFieldVx * Math.cos(h);
            } else {
                targetRobotVy = rawFieldVy;
                targetRobotVx = rawFieldVx;
            }

            limitY.setRateLimit(RobotConfig.ACCEL_LIMIT_MPS2);
            limitX.setRateLimit(RobotConfig.ACCEL_LIMIT_MPS2);
            limitOmega.setRateLimit(RobotConfig.ACCEL_LIMIT_MPS2 / RobotConfig.K);

            double targetVy    = limitY.calculate(targetRobotVy, dt);
            double targetVx    = limitX.calculate(targetRobotVx, dt);
            double targetOmega = limitOmega.calculate(rawOmega, dt);

            double vFL = targetVy + targetVx - (RobotConfig.K * targetOmega);
            double vFR = targetVy - targetVx + (RobotConfig.K * targetOmega);
            double vRL = targetVy - targetVx - (RobotConfig.K * targetOmega);
            double vRR = targetVy + targetVx + (RobotConfig.K * targetOmega);
            double maxMag = Math.max(Math.abs(vFL), Math.max(Math.abs(vFR), Math.max(Math.abs(vRL), Math.abs(vRR))));
            if (maxMag > RobotConfig.MAX_LINEAR_VELOCITY_MPS) {
                vFL = (vFL / maxMag) * RobotConfig.MAX_LINEAR_VELOCITY_MPS;
                vFR = (vFR / maxMag) * RobotConfig.MAX_LINEAR_VELOCITY_MPS;
                vRL = (vRL / maxMag) * RobotConfig.MAX_LINEAR_VELOCITY_MPS;
                vRR = (vRR / maxMag) * RobotConfig.MAX_LINEAR_VELOCITY_MPS;
            }
            frontLeftMotor.setVelocity(metersPerSecToTicks(vFL));
            frontRightMotor.setVelocity(metersPerSecToTicks(vFR));
            backLeftMotor.setVelocity(metersPerSecToTicks(vRL));
            backRightMotor.setVelocity(metersPerSecToTicks(vRR));

            // SERVOS
            boolean cA = gamepad1.a, cB = gamepad1.b;
            if (cA && !prevA) servos.toActive();
            if (cB && !prevB && servos.getState() == ServoSubsystem.State.ACTIVE) servos.toDefault();
            prevA = cA; prevB = cB;

            // SHOOTER
            if (gamepad1.right_trigger > RobotConfig.TRIGGER_THRESH) shooter.requestFeed();
            else shooter.requestStop();
            shooter.update();

            // INTAKE
            boolean fireAssist = (shooter.getState() == ShooterSubsystem.State.FEEDING) && shooter.isReady();
            intake.updateRamp(dt, gamepad1.right_bumper, fireAssist);

            // LIMELIGHT (traj frame + Y flip fix)
            LLResult ll = limelight.getLatestResult();
            boolean llValid = (ll != null && ll.isValid());
            int llTags = 0; long llStale = -1;
            StringBuilder tagIds = new StringBuilder();

            if (ll != null) {
                llStale = ll.getStaleness();
                List<LLResultTypes.FiducialResult> fid = ll.getFiducialResults();
                if (fid != null) {
                    llTags = fid.size();
                    for (LLResultTypes.FiducialResult f : fid) {
                        if (tagIds.length() > 0) tagIds.append(",");
                        tagIds.append(f.getFiducialId());
                    }
                }

                if (llValid) {
                    Pose3D bp = ll.getBotpose();
                    if (bp != null) {
                        double rawX = bp.getPosition().x;
                        double rawY = bp.getPosition().y;
                        double rawZ = bp.getPosition().z;
                        double rawYaw = wrapAngle(bp.getOrientation().getYaw(AngleUnit.RADIANS));
                        double rawPitch = bp.getOrientation().getPitch(AngleUnit.RADIANS);

                        double phi = solveHiveTilt(rawX, rawZ, rawYaw, rawPitch);
                        if (!Double.isNaN(phi)) {
                            double cp = Math.cos(phi), sp = Math.sin(phi), h = RobotConfig.HIVE_PIVOT_HEIGHT_M;
                            double correctedX = rawX * cp - (rawZ - h) * sp;
                            double r00 = Math.cos(rawYaw) * Math.cos(rawPitch);
                            double r10 = Math.sin(rawYaw) * Math.cos(rawPitch);
                            double r20 = -Math.sin(rawPitch);
                            double correctedYaw = Math.atan2(r10, cp * r00 + sp * r20);

                            double c = Math.cos(RobotConfig.FMAP_TO_FIELD_THETA);
                            double s = Math.sin(RobotConfig.FMAP_TO_FIELD_THETA);
                            double llX = c * correctedX - s * rawY + RobotConfig.FMAP_TO_FIELD_TX;
                            double llY = RobotConfig.FLIP_FIELD_Y ? (rawY + RobotConfig.FMAP_TO_FIELD_TY)
                                                                  : (s * correctedX + c * rawY + RobotConfig.FMAP_TO_FIELD_TY);
                            double llTh = wrapAngle(correctedYaw + RobotConfig.FMAP_TO_FIELD_THETA);

                            double trajX = llY;
                            double trajY = llX;
                            double trajTh = RobotConfig.FLIP_VISION_TH_SIGN
                                    ? wrapAngle(Math.PI/2.0 + llTh)
                                    : wrapAngle(Math.PI/2.0 - llTh);
                            trajTh = wrapAngle(trajTh + RobotConfig.VISION_TH_OFFSET);

                            lastRawVisX = trajX;
                            lastRawVisY = trajY;
                            lastRawVisTh = trajTh;

                            double expectedY;
                            if (Double.isNaN(lastAcceptedY)) {
                                expectedY = Double.NaN;
                            } else {
                                expectedY = lastAcceptedY + (odoY - odoYAtLastAccept);
                            }
                            lastExpectedY = expectedY;

                            double L = RobotConfig.FIELD_LENGTH_M;
                            double trajY_flip = L - trajY;

                            double scoreRaw, scoreFlipped;
                            if (Double.isNaN(expectedY)) {
                                scoreRaw = 0; scoreFlipped = 1e6;
                            } else {
                                scoreRaw     = Math.abs(trajY - expectedY);
                                scoreFlipped = Math.abs(trajY_flip - expectedY);
                            }

                            double MARGIN = RobotConfig.Y_FLIP_HYSTERESIS_M;
                            boolean useFlipped;
                            if (lastWasFlipped) {
                                useFlipped = !(scoreRaw + MARGIN < scoreFlipped);
                            } else {
                                useFlipped = scoreFlipped + MARGIN < scoreRaw;
                            }
                            lastWasFlipped = useFlipped;

                            double selectedY = useFlipped ? trajY_flip : trajY;
                            double selectedTiltDeg = useFlipped ? Math.toDegrees(phi) : Math.toDegrees(-phi);

                            lastSelVisX = trajX;
                            lastSelVisY = selectedY;
                            lastSelVisTh = trajTh;
                            lastHiveTiltDeg = selectedTiltDeg;
                            lastCandidateLabel = useFlipped ? "FLIP" : "raw";
                            lastScoreRaw = scoreRaw;
                            lastScoreFlipped = scoreFlipped;

                            lastAcceptedY = selectedY;
                            odoYAtLastAccept = odoY;
                            lastAcceptedMs = System.currentTimeMillis();
                        }
                    }
                }
            }

            // TELEMETRY
            telemetry.addData("> Drive Mode", isFieldCentric ? "FIELD (X)" : "ROBOT (Y)");
            telemetry.addData("> Shooter",   "%.1f / %.1f RPM  state=%s  ready=%s",
                    shooter.getShooterRpm(), shooter.getShooter2Rpm(),
                    shooter.getState(), shooter.isReady());
            telemetry.addData("> Feeder",    "%.2f", shooter.getFeederPower());
            telemetry.addData("> Intake",    "%.2f%s", intake.getPower(), fireAssist ? " [FIRE]" : "");
            telemetry.addData("> Servos",    "%s", servos.getState());

            telemetry.addLine("--- LIMELIGHT (traj frame) ---");
            telemetry.addData("> Valid",   llValid);
            telemetry.addData("> Tags",    llTags);
            telemetry.addData("> Tag IDs", tagIds.length() > 0 ? tagIds.toString() : "(none)");
            telemetry.addData("> Raw Y",   "%.3f  (flip=%.3f)",
                    lastRawVisY, RobotConfig.FIELD_LENGTH_M - lastRawVisY);
            telemetry.addData("> Expected", "%.3f  (prev+odo)", lastExpectedY);
            telemetry.addData("> Scores",  "raw=%.3f flip=%.3f", lastScoreRaw, lastScoreFlipped);
            telemetry.addData("> Selected", "[%s] Y=%.3f  tilt=%.1f",
                    lastCandidateLabel, lastSelVisY, lastHiveTiltDeg);
            telemetry.addData("> odoY",     "%.2f", odoY);
            telemetry.update();
        }

        frontLeftMotor.setPower(0); frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);  backRightMotor.setPower(0);
        shooter.requestImmediateStop();
        intake.off();
        servos.toDefault();
        if (limelight != null) limelight.stop();
    }

    private double solveHiveTilt(double x, double z, double yaw, double pitch) {
        double h = RobotConfig.HIVE_PIVOT_HEIGHT_M;
        double a = x, b = z - h, c = -h;
        double radius = Math.hypot(a, b);
        if (radius < 1e-9 || Math.abs(c) > radius) return Double.NaN;
        double phi = solvePhi(a, b, c, Math.toRadians(RobotConfig.MAX_HIVE_TILT_DEG));
        if (Double.isNaN(phi)) return Double.NaN;
        double sinPred = Math.sin(phi) * Math.cos(yaw);
        double predPitch = Math.asin(Math.max(-1.0, Math.min(1.0, sinPred)));
        double errDeg = Math.toDegrees(Math.abs(wrapAngle(pitch - predPitch)));
        if (errDeg > RobotConfig.PITCH_CHECK_TOL_DEG) return Double.NaN;
        return phi;
    }
    private static double solvePhi(double a, double b, double c, double maxTilt) {
        double r = Math.hypot(a, b);
        double base = Math.atan2(b, a);
        double alpha = Math.asin(c / r);
        double p1 = wrapAngle(alpha - base), p2 = wrapAngle(Math.PI - alpha - base);
        boolean in1 = Math.abs(p1) <= maxTilt, in2 = Math.abs(p2) <= maxTilt;
        if (in1 && in2) return Math.abs(p1) <= Math.abs(p2) ? p1 : p2;
        if (in1) return p1;
        if (in2) return p2;
        return Double.NaN;
    }

    private double getRawHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    private static double wrapAngle(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
    private double metersPerSecToTicks(double mps) {
        return (mps / (2.0 * Math.PI * RobotConfig.WHEEL_RADIUS_M)) * RobotConfig.TICKS_PER_REV;
    }
    private static class SlewRateLimiter {
        private double rateLimit, lastValue = 0.0;
        SlewRateLimiter(double r) { rateLimit = r; }
        void setRateLimit(double r) { rateLimit = Math.abs(r); }
        double calculate(double target, double dt) {
            double maxChange = rateLimit * dt;
            if (target > lastValue + maxChange) lastValue += maxChange;
            else if (target < lastValue - maxChange) lastValue -= maxChange;
            else lastValue = target;
            return lastValue;
        }
    }
}