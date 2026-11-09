package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.commands.AutoCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups;
import org.firstinspires.ftc.teamcode.commands.CommandParser;
import org.firstinspires.ftc.teamcode.commands.ConditionalVision;
import org.firstinspires.ftc.teamcode.commands.PathCommand;
import org.firstinspires.ftc.teamcode.commands.VisionSource;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Scanner;
import java.util.Set;

@Autonomous(name = "Red Auto", group = "Auto")
public class RedAuto extends LinearOpMode implements PathCommand.TargetSink {

    public static final String DEFAULT_PATH_BATCH = "RED_2";
    public static final String TEAMCODE_DIR       = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/";
    public static final String EVENTS_FILENAME    = "events.txt";
    public static final String PATH_FILE_PREFIX   = "path_";
    public static final String PATH_FILE_SUFFIX   = ".txt";

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private Limelight3A limelight;
    private ShooterSubsystem shooter;
    private IntakeSubsystem  intake;
    private ServoSubsystem   servos;

    private double ekfX, ekfY, ekfTheta;
    private double pXX, pYY, pTT;
    private double headingOffsetRad = 0.0;
    private int lastFlPos = 0, lastFrPos = 0, lastBlPos = 0, lastBrPos = 0;

    // Pure odometry (no vision pollution) — used as flip reference
    private double odoX = 0, odoY = 0;

    // Vision state
    private boolean lastVisionValid = false;
    private int     lastTagCount = 0;
    private long    lastStaleMs  = -1;
    private double  lastRawVisX, lastRawVisY, lastRawVisTh;
    private double  lastSelVisX, lastSelVisY, lastSelVisTh;
    private String  lastCandidateLabel = "raw";
    private double  lastHiveTiltDeg = Double.NaN;
    private double  lastScoreRaw = 0, lastScoreFlipped = 0;
    private boolean lastWasFlipped = false;
    private double  lastAcceptedX = Double.NaN;
    private double  lastAcceptedY = Double.NaN;
    private long    lastAcceptedMs = 0;
    private double  emaVisX = Double.NaN, emaVisY = Double.NaN, emaVisTh = Double.NaN;

    private double[] pathTargetThisFrame = null;

    private class LimelightVisionSource implements VisionSource {
        @Override public Set<Integer> getVisibleTagIds(long maxStalenessMs) {
            Set<Integer> ids = new HashSet<>();
            LLResult ll = limelight.getLatestResult();
            if (ll == null || !ll.isValid()) return ids;
            if (ll.getStaleness() > maxStalenessMs) return ids;
            List<LLResultTypes.FiducialResult> fid = ll.getFiducialResults();
            if (fid == null) return ids;
            for (LLResultTypes.FiducialResult f : fid) ids.add(f.getFiducialId());
            return ids;
        }
    }

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

        List<String[]> batches = listAvailableBatches();
        if (batches.isEmpty()) {
            telemetry.addData("ERROR", "No batch folders with events.txt or path_*.txt");
            telemetry.update(); sleep(5000); return;
        }
        int selectedIdx = 0;
        for (int i = 0; i < batches.size(); i++) {
            if (batches.get(i)[0].equals(DEFAULT_PATH_BATCH)) { selectedIdx = i; break; }
        }
        boolean dUpPrev = false, dDownPrev = false;
        while (!isStarted() && !isStopRequested()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            if (up && !dUpPrev)     selectedIdx = (selectedIdx - 1 + batches.size()) % batches.size();
            if (down && !dDownPrev) selectedIdx = (selectedIdx + 1) % batches.size();
            dUpPrev = up; dDownPrev = down;
            telemetry.addLine("=== SELECT BATCH ===");
            for (int i = 0; i < batches.size(); i++) {
                String marker = (i == selectedIdx) ? ">>> " : "    ";
                telemetry.addData(marker + batches.get(i)[0], batches.get(i)[1]);
            }
            telemetry.addData("D-pad", "up/down | START");
            telemetry.update(); sleep(20);
        }
        String selectedBatch = batches.get(selectedIdx)[0];
        String batchDisplay  = selectedBatch.isEmpty() ? "(teamcode root)" : selectedBatch;
        String batchDir = selectedBatch.isEmpty() ? TEAMCODE_DIR : TEAMCODE_DIR + selectedBatch + "/";

        VisionSource visionSource = new LimelightVisionSource();
        CommandParser parser = new CommandParser(
                shooter, intake, servos,
                name -> loadPathFile(batchDir, name),
                visionSource, this);

        AutoCommand rootCmd;
        try {
            File evFile = new File(batchDir + EVENTS_FILENAME);
            if (evFile.exists()) {
                rootCmd = parser.parse(evFile);
                telemetry.addData("Routine", "Parsed " + EVENTS_FILENAME);
            } else {
                List<File> pathFiles = listPathFiles(batchDir);
                if (pathFiles.isEmpty()) {
                    telemetry.addData("ERROR", "No events.txt AND no path files");
                    telemetry.update(); sleep(5000); return;
                }
                List<AutoCommand> auto = new ArrayList<>();
                for (File pf : pathFiles) {
                    auto.add(new PathCommand(loadPathFile(batchDir, pf.getName()), this));
                }
                rootCmd = new CommandGroups.Sequential(auto);
                telemetry.addData("Routine", "No events.txt — auto SEQ of paths");
            }
        } catch (Exception e) {
            telemetry.addData("PARSE ERROR", e.getMessage());
            telemetry.update(); sleep(5000); return;
        }

        double[] startPose = findFirstPathStart(rootCmd);
        if (startPose == null) {
            startPose = new double[]{0, 0, 0, 0};
            telemetry.addData("WARN", "No PATH — EKF start at (0,0,0)");
        }
        ekfX = startPose[1]; ekfY = startPose[2]; ekfTheta = startPose[3];
        odoX = startPose[1]; odoY = startPose[2];
        headingOffsetRad = getRawHeadingRad() - ekfTheta;
        pXX = 0.1; pYY = 0.1; pTT = 0.05;
        lastFlPos = frontLeftMotor.getCurrentPosition();
        lastFrPos = frontRightMotor.getCurrentPosition();
        lastBlPos = backLeftMotor.getCurrentPosition();
        lastBrPos = backRightMotor.getCurrentPosition();

        telemetry.addData("Batch", batchDisplay);
        telemetry.addData("Start Pose", "X=%.2f Y=%.2f Th=%.1f",
                ekfX, ekfY, Math.toDegrees(ekfTheta));
        telemetry.addData("Status", "Ready. START to run.");
        telemetry.update();

        waitForStart();

        rootCmd.start();
        double lastTime = getRuntime();

        while (opModeIsActive() && !rootCmd.isFinished()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            predictEKF();
            resetEMAIfStale();
            updateEKFWithVision();

            pathTargetThisFrame = null;
            rootCmd.execute(dt);

            if (pathTargetThisFrame != null) driveControl(pathTargetThisFrame);
            else stopDrive();

            telemetry.addData("Batch", batchDisplay);
            telemetry.addData("Routine", rootCmd.isFinished() ? "DONE" : "RUNNING");
            telemetry.addData("Conditional", ConditionalVision.lastDecisionLog);
            if (pathTargetThisFrame != null) {
                telemetry.addData("Path Target", "X=%.2f Y=%.2f Th=%.1f",
                        pathTargetThisFrame[1], pathTargetThisFrame[2],
                        Math.toDegrees(pathTargetThisFrame[3]));
            }
            telemetry.addData("EKF",  "X=%.2f Y=%.2f Th=%.1f",
                    ekfX, ekfY, Math.toDegrees(ekfTheta));
            telemetry.addData("odoY", "%.2f  (pure encoder)", odoY);
            telemetry.addData("Shooter", "%s ready=%s %.0f/%.0f rpm",
                    shooter.getState(), shooter.isReady(),
                    shooter.getShooterRpm(), shooter.getShooter2Rpm());
            telemetry.addData("Intake/Servo", "intake=%.2f servo=%s",
                    intake.getPower(), servos.getState());

            telemetry.addLine("--- VISION (traj frame) ---");
            telemetry.addData("> Valid",   "%s tags=%d stale=%dms",
                    lastVisionValid ? "OK" : "STALE", lastTagCount, lastStaleMs);
            telemetry.addData("> Raw Y",   "%.3f  (flip would be %.3f)",
                    lastRawVisY, RobotConfig.FIELD_LENGTH_M - lastRawVisY);
            telemetry.addData("> Scores",  "raw=%.3f flip=%.3f", lastScoreRaw, lastScoreFlipped);
            telemetry.addData("> Selected", "[%s] Y=%.3f  tilt=%.1f",
                    lastCandidateLabel, lastSelVisY, lastHiveTiltDeg);
            telemetry.update();
        }

        if (!rootCmd.isFinished()) rootCmd.cancel();
        stopDrive();
        shooter.requestImmediateStop();
        intake.off();
        servos.toDefault();
        limelight.stop();
    }

    @Override public void submitPathTarget(double[] target) { pathTargetThisFrame = target; }

    private void driveControl(double[] target) {
        double errorX = target[1] - ekfX;
        double errorY = target[2] - ekfY;
        double errorTheta = wrapAngle(target[3] - ekfTheta);
        double robotErrY = errorX * Math.cos(ekfTheta) + errorY * Math.sin(ekfTheta);
        double robotErrX = errorX * Math.sin(ekfTheta) - errorY * Math.cos(ekfTheta);
        double correctionVy = RobotConfig.KP_Y * robotErrY;
        double correctionVx = RobotConfig.KP_X * robotErrX;
        double correctionOmega = RobotConfig.KP_THETA * errorTheta;
        double ffLocalVy = target[4] * Math.cos(ekfTheta) + target[5] * Math.sin(ekfTheta);
        double ffLocalVx = target[4] * Math.sin(ekfTheta) - target[5] * Math.cos(ekfTheta);
        double targetVy = ffLocalVy + correctionVy;
        double targetVx = ffLocalVx + correctionVx;
        double targetOmega = target[6] + correctionOmega;
        double vFL = targetVy + targetVx - (RobotConfig.K * targetOmega);
        double vFR = targetVy - targetVx + (RobotConfig.K * targetOmega);
        double vRL = targetVy - targetVx - (RobotConfig.K * targetOmega);
        double vRR = targetVy + targetVx + (RobotConfig.K * targetOmega);
        double maxMag = Math.max(Math.abs(vFL),
                Math.max(Math.abs(vFR), Math.max(Math.abs(vRL), Math.abs(vRR))));
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
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0); frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);  backRightMotor.setPower(0);
    }

    private double[] findFirstPathStart(AutoCommand cmd) {
        if (cmd instanceof PathCommand) return ((PathCommand) cmd).getFirstPoint();
        if (cmd instanceof CommandGroups.Sequential)
            for (AutoCommand c : ((CommandGroups.Sequential) cmd).getChildren()) {
                double[] r = findFirstPathStart(c); if (r != null) return r; }
        if (cmd instanceof CommandGroups.Parallel)
            for (AutoCommand c : ((CommandGroups.Parallel) cmd).getChildren()) {
                double[] r = findFirstPathStart(c); if (r != null) return r; }
        if (cmd instanceof CommandGroups.Race)
            for (AutoCommand c : ((CommandGroups.Race) cmd).getChildren()) {
                double[] r = findFirstPathStart(c); if (r != null) return r; }
        if (cmd instanceof ConditionalVision) {
            // Prefer THEN's path — the robot's physical start pose must match
            // whatever path runs first in the expected branch.
            ConditionalVision cv = (ConditionalVision) cmd;
            double[] r = findFirstPathStart(cv.getThenCmd());
            if (r != null) return r;
            return findFirstPathStart(cv.getElseCmd());
        }
        return null;
    }

    private void predictEKF() {
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
        ekfTheta = wrapAngle(getRawHeadingRad() - headingOffsetRad);
        double dx = dVx * Math.sin(ekfTheta) + dVy * Math.cos(ekfTheta);
        double dy = -dVx * Math.cos(ekfTheta) + dVy * Math.sin(ekfTheta);
        ekfX += dx; ekfY += dy;
        odoX += dx; odoY += dy;   // pure odometry — NO vision correction
        pXX += RobotConfig.ODOMETRY_LINEAR_NOISE * RobotConfig.ODOMETRY_LINEAR_NOISE;
        pYY += RobotConfig.ODOMETRY_LINEAR_NOISE * RobotConfig.ODOMETRY_LINEAR_NOISE;
        pTT += RobotConfig.ODOMETRY_ANGULAR_NOISE * RobotConfig.ODOMETRY_ANGULAR_NOISE;
    }

    private void updateEKFWithVision() {
        lastVisionValid = false;

        LLResult ll = limelight.getLatestResult();
        if (ll == null) return;
        lastStaleMs = ll.getStaleness();
        List<LLResultTypes.FiducialResult> fid = ll.getFiducialResults();
        lastTagCount = (fid != null) ? fid.size() : 0;
        if (!ll.isValid()) return;
        if (fid == null || fid.size() < RobotConfig.MIN_TAGS_FOR_UPDATE) return;
        if (lastStaleMs > RobotConfig.MAX_VISION_STALENESS_MS) return;

        Pose3D bp = ll.getBotpose();
        if (bp == null) return;

        double rawX = bp.getPosition().x, rawY = bp.getPosition().y, rawZ = bp.getPosition().z;
        double rawYaw = wrapAngle(bp.getOrientation().getYaw(AngleUnit.RADIANS));
        double rawPitch = bp.getOrientation().getPitch(AngleUnit.RADIANS);

        double phi = solveHiveTilt(rawX, rawZ, rawYaw, rawPitch);
        if (Double.isNaN(phi)) return;

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

        lastRawVisX = trajX; lastRawVisY = trajY; lastRawVisTh = trajTh;

        // ===== Y FLIP DECISION (with hysteresis) =====
        double L = RobotConfig.FIELD_LENGTH_M;
        double trajY_flip = L - trajY;

        // Reference 1: path target Y (unpolluted by vision; NaN if no path active)
        double refTraj = (pathTargetThisFrame != null) ? pathTargetThisFrame[2] : Double.NaN;

        // Reference 2: EKF Y (may be polluted by prior vision)
        double refEKF = ekfY;

        // Reference 3: last accepted vision Y (decays with staleness)
        double refPrev = lastAcceptedY;
        double prevWeightScale = 1.0;
        if (!Double.isNaN(refPrev)) {
            long ageMs = System.currentTimeMillis() - lastAcceptedMs;
            if (ageMs > 0) {
                prevWeightScale = Math.exp(-ageMs / RobotConfig.Y_FLIP_PREV_DECAY_MS);
                if (prevWeightScale < 0.05) prevWeightScale = 0;  // effectively gone
            }
        } else {
            prevWeightScale = 0;
        }

        // Weighted score
        double scoreRaw = 0, scoreFlipped = 0, totalWeight = 0;
        if (!Double.isNaN(refTraj)) {
            double w = RobotConfig.Y_FLIP_REF_WEIGHT_TRAJ;
            scoreRaw     += w * Math.abs(trajY - refTraj);
            scoreFlipped += w * Math.abs(trajY_flip - refTraj);
            totalWeight  += w;
        }
        if (!Double.isNaN(refEKF)) {
            double w = RobotConfig.Y_FLIP_REF_WEIGHT_EKF;
            scoreRaw     += w * Math.abs(trajY - refEKF);
            scoreFlipped += w * Math.abs(trajY_flip - refEKF);
            totalWeight  += w;
        }
        if (prevWeightScale > 0) {
            double w = RobotConfig.Y_FLIP_REF_WEIGHT_PREV * prevWeightScale;
            scoreRaw     += w * Math.abs(trajY - refPrev);
            scoreFlipped += w * Math.abs(trajY_flip - refPrev);
            totalWeight  += w;
        }
        if (totalWeight > 0) {
            scoreRaw     /= totalWeight;
            scoreFlipped /= totalWeight;
        }

        // Hysteresis
        double MARGIN = RobotConfig.Y_FLIP_HYSTERESIS_M;
        boolean useFlipped;
        if (lastWasFlipped) {
            useFlipped = !(scoreRaw + MARGIN < scoreFlipped);
        } else {
            useFlipped = scoreFlipped + MARGIN < scoreRaw;
        }
        lastWasFlipped = useFlipped;

        double selectedY = useFlipped ? trajY_flip : trajY;
        // Tilt sign follows flip: if Y was mirrored, phi was mirrored too
        double selectedTiltDeg = useFlipped ? Math.toDegrees(phi) : Math.toDegrees(-phi);

        lastSelVisX = trajX;
        lastSelVisY = selectedY;
        lastSelVisTh = trajTh;
        lastHiveTiltDeg = selectedTiltDeg;
        lastCandidateLabel = useFlipped ? "FLIP" : "raw";
        lastScoreRaw = scoreRaw;
        lastScoreFlipped = scoreFlipped;
        lastVisionValid = true;

        // Update accepted state
        lastAcceptedX = trajX;
        lastAcceptedY = selectedY;
        lastAcceptedMs = System.currentTimeMillis();

        // EMA
        if (Double.isNaN(emaVisX)) {
            emaVisX = trajX; emaVisY = selectedY; emaVisTh = trajTh;
        } else {
            double a = RobotConfig.VISION_EMA_ALPHA;
            emaVisX = a * trajX + (1.0 - a) * emaVisX;
            emaVisY = a * selectedY + (1.0 - a) * emaVisY;
            emaVisTh = wrapAngle(emaVisTh + a * wrapAngle(trajTh - emaVisTh));
        }

        // Optional outlier gate (disabled by default)
        if (RobotConfig.ENABLE_VISION_OUTLIER_GATE) {
            double innovationDist = Math.hypot(emaVisX - ekfX, emaVisY - ekfY);
            if (innovationDist > RobotConfig.MAX_VISION_JUMP_M) return;
        }

        // Kalman update
        double kX = pXX / (pXX + RobotConfig.VISION_LINEAR_NOISE * RobotConfig.VISION_LINEAR_NOISE)
                * RobotConfig.VISION_TRUST_FACTOR;
        double kY = pYY / (pYY + RobotConfig.VISION_LINEAR_NOISE * RobotConfig.VISION_LINEAR_NOISE)
                * RobotConfig.VISION_TRUST_FACTOR;
        if (Math.abs(emaVisX - ekfX) > RobotConfig.VISION_XY_DEADBAND_M) {
            ekfX += kX * (emaVisX - ekfX); pXX *= (1 - kX);
        }
        if (Math.abs(emaVisY - ekfY) > RobotConfig.VISION_XY_DEADBAND_M) {
            ekfY += kY * (emaVisY - ekfY); pYY *= (1 - kY);
        }
        if (RobotConfig.FUSE_VISION_THETA) {
            double kT = pTT / (pTT + RobotConfig.VISION_ANGULAR_NOISE * RobotConfig.VISION_ANGULAR_NOISE)
                    * RobotConfig.VISION_TRUST_FACTOR;
            ekfTheta = wrapAngle(ekfTheta + kT * wrapAngle(emaVisTh - ekfTheta));
            pTT *= (1 - kT);
        }
    }

    private void resetEMAIfStale() {
        if (!lastVisionValid && lastStaleMs > RobotConfig.MAX_VISION_STALENESS_MS) {
            emaVisX = Double.NaN; emaVisY = Double.NaN; emaVisTh = Double.NaN;
        }
    }

    private List<String[]> listAvailableBatches() {
        List<String[]> out = new ArrayList<>();
        File root = new File(TEAMCODE_DIR);
        if (!root.exists() || !root.isDirectory()) return out;
        if (new File(TEAMCODE_DIR + EVENTS_FILENAME).exists() || !listPathFiles(TEAMCODE_DIR).isEmpty()) {
            out.add(new String[]{"", "root"});
        }
        File[] subs = root.listFiles(File::isDirectory);
        if (subs != null) {
            Arrays.sort(subs, (a, b) -> a.getName().compareTo(b.getName()));
            for (File d : subs) {
                String dirPath = d.getAbsolutePath() + "/";
                boolean hasEv = new File(dirPath + EVENTS_FILENAME).exists();
                int n = listPathFiles(dirPath).size();
                if (hasEv || n > 0) {
                    String label = (hasEv ? EVENTS_FILENAME : "")
                            + (hasEv && n > 0 ? " + " : "")
                            + (n > 0 ? n + " paths" : "");
                    out.add(new String[]{d.getName(), label});
                }
            }
        }
        return out;
    }

    private List<File> listPathFiles(String dirPath) {
        File d = new File(dirPath);
        File[] files = d.listFiles((dir, name) ->
                name.startsWith(PATH_FILE_PREFIX) && name.endsWith(PATH_FILE_SUFFIX));
        if (files == null) return new ArrayList<>();
        List<File> list = new ArrayList<>(Arrays.asList(files));
        list.sort((a, b) -> a.getName().compareTo(b.getName()));
        return list;
    }

    private List<double[]> loadPathFile(String dirPath, String name) throws FileNotFoundException {
        String fn = name.endsWith(PATH_FILE_SUFFIX) ? name : name + PATH_FILE_SUFFIX;
        File f = new File(dirPath + fn);
        if (!f.exists()) throw new FileNotFoundException("Path not found: " + f.getAbsolutePath());
        List<double[]> path = new ArrayList<>();
        try (Scanner sc = new Scanner(f)) {
            while (sc.hasNextLine()) {
                String line = sc.nextLine().trim();
                if (line.isEmpty() || line.startsWith("Time_s")) continue;
                String[] parts = line.split(",");
                if (parts.length >= 13) {
                    double[] pt = new double[13];
                    for (int j = 0; j < 13; j++) pt[j] = Double.parseDouble(parts[j].trim());
                    path.add(pt);
                }
            }
        }
        if (path.isEmpty()) throw new FileNotFoundException("Empty path: " + fn);
        return path;
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
}