package org.firstinspires.ftc.teamcode.commands;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class TagFilter {

    private final Set<Integer> ids;

    private TagFilter(Set<Integer> ids) {
        this.ids = Collections.unmodifiableSet(ids);
    }

    /**
     * Parse "34-37", "34,35,36,37", "34-37,42", "34", or any comma-separated mix.
     */
    public static TagFilter parse(String spec) {
        Set<Integer> out = new HashSet<>();
        for (String chunk : spec.trim().split(",")) {
            chunk = chunk.trim();
            if (chunk.isEmpty()) continue;
            if (chunk.contains("-")) {
                String[] parts = chunk.split("-");
                int lo = Integer.parseInt(parts[0].trim());
                int hi = Integer.parseInt(parts[1].trim());
                if (lo > hi) { int t = lo; lo = hi; hi = t; }
                for (int i = lo; i <= hi; i++) out.add(i);
            } else {
                out.add(Integer.parseInt(chunk));
            }
        }
        if (out.isEmpty()) throw new IllegalArgumentException("Empty tag filter: " + spec);
        return new TagFilter(out);
    }

    public Set<Integer> getIds() { return ids; }

    public boolean anyMatch(Set<Integer> visible) {
        for (int id : ids) if (visible.contains(id)) return true;
        return false;
    }

    @Override public String toString() {
        return ids.toString();
    }
}