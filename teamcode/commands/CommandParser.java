package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class CommandParser {

    public interface PathLoader {
        List<double[]> loadPath(String name) throws FileNotFoundException;
    }

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem  intake;
    private final ServoSubsystem   servos;
    private final PathLoader       pathLoader;
    private final VisionSource     visionSource;
    private final PathCommand.TargetSink sink;

    public CommandParser(ShooterSubsystem shooter, IntakeSubsystem intake,
                         ServoSubsystem servos, PathLoader pathLoader,
                         VisionSource visionSource, PathCommand.TargetSink sink) {
        this.shooter = shooter;
        this.intake = intake;
        this.servos = servos;
        this.pathLoader = pathLoader;
        this.visionSource = visionSource;
        this.sink = sink;
    }

    public AutoCommand parse(File eventsFile) throws Exception {
        List<Line> lines = new ArrayList<>();
        try (Scanner sc = new Scanner(eventsFile)) {
            int lineNo = 0;
            while (sc.hasNextLine()) {
                lineNo++;
                String raw = sc.nextLine();
                String stripped = raw.trim();
                if (stripped.isEmpty() || stripped.startsWith("#") || stripped.startsWith("//")) continue;
                int indent = 0, i = 0;
                while (i < raw.length()) {
                    char c = raw.charAt(i);
                    if (c == ' ') { indent++; i++; }
                    else if (c == '\t') { indent += 4; i++; }
                    else break;
                }
                lines.add(new Line(lineNo, indent, stripped));
            }
        }
        int[] pos = {0};
        List<AutoCommand> topLevel = parseBlock(lines, pos, -1);
        if (pos[0] < lines.size()) {
            Line bad = lines.get(pos[0]);
            throw new IllegalArgumentException("Line " + bad.lineNo + ": unexpected indent");
        }
        return new CommandGroups.Sequential(topLevel);
    }

    private List<AutoCommand> parseBlock(List<Line> lines, int[] pos, int parentIndent) throws Exception {
        List<AutoCommand> cmds = new ArrayList<>();
        Integer childIndent = null;
        while (pos[0] < lines.size()) {
            Line l = lines.get(pos[0]);
            if (l.indent <= parentIndent) break;
            if (childIndent == null) childIndent = l.indent;
            if (l.indent != childIndent) {
                throw new IllegalArgumentException("Line " + l.lineNo
                        + ": inconsistent indent (expected " + childIndent
                        + ", got " + l.indent + ")");
            }
            pos[0]++;
            String[] parts = l.text.split("\\s+");
            String kw = parts[0].toUpperCase();
            switch (kw) {
                case "SEQ":
                    cmds.add(new CommandGroups.Sequential(parseBlock(lines, pos, childIndent)));
                    break;
                case "PAR":
                    cmds.add(new CommandGroups.Parallel(parseBlock(lines, pos, childIndent)));
                    break;
                case "RACE":
                    cmds.add(new CommandGroups.Race(parseBlock(lines, pos, childIndent)));
                    break;
                case "IF_VISION": {
                    if (parts.length < 2) {
                        throw new IllegalArgumentException("Line " + l.lineNo
                                + ": IF_VISION requires tag spec (e.g. 34-37)");
                    }
                    TagFilter filter = TagFilter.parse(parts[1]);
                    double timeout = (parts.length >= 3)
                            ? Double.parseDouble(parts[2])
                            : RobotConfig.VISION_CONDITIONAL_DEFAULT_TIMEOUT_S;

                    if (pos[0] >= lines.size()) {
                        throw new IllegalArgumentException("Line " + l.lineNo
                                + ": IF_VISION missing THEN block");
                    }
                    Line thenLine = lines.get(pos[0]);
                    if (thenLine.indent <= childIndent) {
                        throw new IllegalArgumentException("Line " + thenLine.lineNo
                                + ": IF_VISION expects THEN at deeper indent");
                    }
                    if (!thenLine.text.toUpperCase().equals("THEN")) {
                        throw new IllegalArgumentException("Line " + thenLine.lineNo
                                + ": expected THEN, got: " + thenLine.text);
                    }
                    int branchIndent = thenLine.indent;
                    pos[0]++;
                    List<AutoCommand> thenCmds = parseBlock(lines, pos, branchIndent);

                    List<AutoCommand> elseCmds = new ArrayList<>();
                    if (pos[0] < lines.size()) {
                        Line elseLine = lines.get(pos[0]);
                        if (elseLine.indent == branchIndent
                                && elseLine.text.toUpperCase().equals("ELSE")) {
                            pos[0]++;
                            elseCmds = parseBlock(lines, pos, branchIndent);
                        }
                    }

                    cmds.add(new ConditionalVision(
                            filter, timeout,
                            new CommandGroups.Sequential(thenCmds),
                            new CommandGroups.Sequential(elseCmds),
                            visionSource,
                            RobotConfig.MAX_VISION_STALENESS_MS));
                    break;
                }
                default:
                    cmds.add(parseLeaf(parts, l.lineNo));
            }
        }
        return cmds;
    }

    private AutoCommand parseLeaf(String[] parts, int lineNo) throws Exception {
        String kw = parts[0].toUpperCase();
        switch (kw) {
            case "PATH": {
                if (parts.length < 2)
                    throw new IllegalArgumentException("Line " + lineNo + ": PATH requires a name");
                return new PathCommand(pathLoader.loadPath(parts[1]), sink);
            }
            case "WAIT": {
                if (parts.length < 2)
                    throw new IllegalArgumentException("Line " + lineNo + ": WAIT requires seconds");
                return new OtherCommands.Wait(Double.parseDouble(parts[1]));
            }
            case "SHOOT": {
                double dur = (parts.length >= 2)
                        ? Double.parseDouble(parts[1])
                        : RobotConfig.SHOOTER_BLOCKING_FEED_DURATION_S;
                return new OtherCommands.Shoot(shooter, intake, dur);
            }
            case "INTAKE": {
                if (parts.length < 2) {
                    // No duration = hold until cancelled by parent
                    return new OtherCommands.IntakeHold(intake, RobotConfig.INTAKE_MAX_POWER);
                }
                double dur = Double.parseDouble(parts[1]);
                double power = (parts.length >= 3)
                        ? Double.parseDouble(parts[2])
                        : RobotConfig.INTAKE_MAX_POWER;
                return new OtherCommands.Intake(intake, dur, power);
            }
            case "INTAKE_HOLD": {
                double power = (parts.length >= 2)
                        ? Double.parseDouble(parts[1])
                        : RobotConfig.INTAKE_MAX_POWER;
                return new OtherCommands.IntakeHold(intake, power);
            }
            case "SERVO_ACTIVE":
                return new OtherCommands.ServoActive(servos);
            case "SERVO_DEFAULT":
                return new OtherCommands.ServoDefault(servos);
            default:
                throw new IllegalArgumentException("Line " + lineNo
                        + ": unknown command '" + kw + "'");
        }
    }

    private static class Line {
        final int lineNo, indent;
        final String text;
        Line(int lineNo, int indent, String text) {
            this.lineNo = lineNo; this.indent = indent; this.text = text;
        }
    }
}