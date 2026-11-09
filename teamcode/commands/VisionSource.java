package org.firstinspires.ftc.teamcode.commands;

import java.util.Set;

public interface VisionSource {
    /**
     * Return the set of tag IDs currently visible from a fresh Limelight frame.
     * If the latest result is older than maxStalenessMs, return empty.
     */
    Set<Integer> getVisibleTagIds(long maxStalenessMs);
}