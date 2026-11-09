package org.firstinspires.ftc.teamcode.commands;

import java.util.List;

public class CommandGroups {

    public static class Sequential extends AutoCommand {
        private final List<AutoCommand> children;
        private int idx = 0;

        public Sequential(List<AutoCommand> children) { this.children = children; }
        public List<AutoCommand> getChildren() { return children; }

        @Override public void start() {
            idx = 0;
            if (!children.isEmpty()) children.get(0).start();
        }
        @Override public void execute(double dt) {
            if (idx >= children.size()) return;
            AutoCommand cur = children.get(idx);
            cur.execute(dt);
            if (cur.isFinished()) {
                cur.end(false);
                idx++;
                if (idx < children.size()) children.get(idx).start();
            }
        }
        @Override public boolean isFinished() { return idx >= children.size(); }

        @Override public void end(boolean interrupted) {
            if (interrupted && idx < children.size() && !children.get(idx).isFinished()) {
                children.get(idx).cancel();
            }
        }
    }

    public static class Parallel extends AutoCommand {
        private final List<AutoCommand> children;
        private final boolean[] finished;

        public Parallel(List<AutoCommand> children) {
            this.children = children;
            this.finished = new boolean[children.size()];
        }
        public List<AutoCommand> getChildren() { return children; }

        @Override public void start() {
            for (int i = 0; i < children.size(); i++) {
                finished[i] = false;
                children.get(i).start();
            }
        }
        @Override public void execute(double dt) {
            for (int i = 0; i < children.size(); i++) {
                if (!finished[i]) {
                    children.get(i).execute(dt);
                    if (children.get(i).isFinished()) {
                        children.get(i).end(false);
                        finished[i] = true;
                    }
                }
            }
        }
        @Override public boolean isFinished() {
            for (boolean f : finished) if (!f) return false;
            return true;
        }
        @Override public void end(boolean interrupted) {
            if (interrupted) {
                for (int i = 0; i < children.size(); i++) {
                    if (!finished[i] && !children.get(i).isFinished()) children.get(i).cancel();
                }
            }
        }
    }

    public static class Race extends AutoCommand {
        private final List<AutoCommand> children;
        private final boolean[] finished;

        public Race(List<AutoCommand> children) {
            this.children = children;
            this.finished = new boolean[children.size()];
        }
        public List<AutoCommand> getChildren() { return children; }

        @Override public void start() {
            for (int i = 0; i < children.size(); i++) {
                finished[i] = false;
                children.get(i).start();
            }
        }
        @Override public void execute(double dt) {
            for (int i = 0; i < children.size(); i++) {
                if (!finished[i]) {
                    children.get(i).execute(dt);
                    if (children.get(i).isFinished()) {
                        children.get(i).end(false);
                        finished[i] = true;
                        for (int j = 0; j < children.size(); j++) {
                            if (!finished[j]) {
                                children.get(j).cancel();
                                finished[j] = true;
                            }
                        }
                        return;
                    }
                }
            }
        }
        @Override public boolean isFinished() {
            for (boolean f : finished) if (!f) return false;
            return true;
        }
        @Override public void end(boolean interrupted) {
            if (interrupted) {
                for (int i = 0; i < children.size(); i++) {
                    if (!finished[i] && !children.get(i).isFinished()) children.get(i).cancel();
                }
            }
        }
    }
}