package org.firstinspires.ftc.teamcode.commands;

import java.util.List;

public class PathCommand extends AutoCommand {

    public interface TargetSink {
        void submitPathTarget(double[] target);
    }

    private final List<double[]> path;
    private final double duration;
    private final TargetSink sink;
    private double localTime = 0;

    public PathCommand(List<double[]> path, TargetSink sink) {
        if (path == null || path.isEmpty())
            throw new IllegalArgumentException("Empty path");
        this.path = path;
        this.duration = path.get(path.size() - 1)[0] - path.get(0)[0];
        this.sink = sink;
    }

    public double[] getFirstPoint() { return path.get(0); }

    @Override public void start() { localTime = 0; }

    @Override public void execute(double dt) {
        localTime += dt;
        sink.submitPathTarget(interpolate(localTime));
    }

    @Override public boolean isFinished() { return localTime >= duration; }

    private double[] interpolate(double localT) {
        double tAbs = path.get(0)[0] + localT;
        for (int i = 0; i < path.size() - 1; i++) {
            double[] p1 = path.get(i), p2 = path.get(i + 1);
            if (tAbs >= p1[0] && tAbs <= p2[0]) {
                double dt = p2[0] - p1[0];
                if (dt == 0) return p1;
                double a = (tAbs - p1[0]) / dt;
                double[] out = new double[13];
                out[0] = tAbs;
                for (int j = 1; j < 13; j++) {
                    if (j == 3) {
                        double diff = p2[3] - p1[3];
                        while (diff > Math.PI)  diff -= 2 * Math.PI;
                        while (diff < -Math.PI) diff += 2 * Math.PI;
                        double v = p1[3] + diff * a;
                        while (v > Math.PI)  v -= 2 * Math.PI;
                        while (v < -Math.PI) v += 2 * Math.PI;
                        out[3] = v;
                    } else {
                        out[j] = p1[j] + (p2[j] - p1[j]) * a;
                    }
                }
                return out;
            }
        }
        return path.get(path.size() - 1);
    }
}