package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

/** This will be the path drawer class, straight segments between waypoints.
 * units are in meters and radians.
  */
public class Path {
    private static final double MIN_SEGMENT_M = 0.001; // merge points closer than 1mm

    private final Pose2d[] points;
    private final double[] cumLength; //cumulative length: distance along the path to each point
    private final double totalLength;

    //class constructor: conver the list into usable variable
    public Path(Pose2d... waypoints) {
        List<Pose2d> kept = new ArrayList<>();
        for (Pose2d p : waypoints) {
            Pose2d last = kept.isEmpty() ? null : kept.get(kept.size() - 1);
            if (last != null && Math.hypot(p.x - last.x, p.y - last.y) < MIN_SEGMENT_M) {
                kept.set(kept.size() - 1, p);  // same spot: keep the later heading
            } else {
                kept.add(p);
            }
        }
        points = kept.toArray(new Pose2d[0]);

        cumLength = new double[points.length];
        for (int i = 1; i < points.length; i++) {
            cumLength[i] = cumLength[i - 1] + xy(i).getDistance(xy(i - 1));
        }
        totalLength = cumLength[points.length - 1];
    }

    //helper functions
    public int numSegments() { return points.length - 1; }
    public double getTotalLength() {return totalLength; }
    public Pose2d getEnd() { return points[points.length - 1]; }
    public Translation2d xy(int i) { return new Translation2d(points[i].x, points[i].y); }


    /** Determine which segment that distance s falls on */
    private int segmentAt(double s) {
        int i = 0;
        while (i < numSegments() - 1 && cumLength[i + 1] < s) i++;
        return  i;
    }

    /** Determine how far along segment i distance s is with a fraction */
    private double fractionAt(int i, double s){
        return (s - cumLength[i]) / (cumLength[i + 1] - cumLength[i]);
    }

    /** The point on the path at distance s from the start */
    public Translation2d pointAt(double s){
        if (numSegments() == 0) return  xy(0);
        s = Math.max(0, Math.min(totalLength, s));
        int i = segmentAt(s);
        return xy(i).plus(xy(i + 1).minus(xy(i)).times(fractionAt(i, s)));
    }

    /** The heading at distance s,turning the short way between waypoint headings. */
    public double headingAt(double s) {
        if (numSegments() == 0) return points[0].heading.getRadians();
        s = Math.max(0, Math.min(totalLength, s));
        int i = segmentAt(s);
        double turn = points[i + 1].heading.minus(points[i].heading).getRadians();
        return new Rotation2d(points[i].heading.getRadians() + fractionAt(i, s) * turn).getRadians();
    }

    /** Closest point on the path to the robot, searching segment startSeg and the next one */
    public Projection projection(Translation2d robot, int startSeg){
        if (numSegments() == 0) return new Projection(0,0, robot.getDistance(xy(0)));

        int lastSeg = Math.min(startSeg + 1, numSegments() - 1);
        int bestSeg = startSeg;
        double bestT = 0,
                bestDist = Double.POSITIVE_INFINITY;

        for (int i = startSeg; i <= lastSeg; i++){
            Translation2d a = xy(i);
            Translation2d d = xy(i + 1).minus(a);
            double t = robot.minus(a).dot(d) / d.dot(d);
            t = Math.max(0, Math.min(1,t));
            double dist = robot.getDistance(a.plus(d.times(t)));
            if (dist < bestDist) {
                bestDist = dist;
                bestSeg = i;
                bestT = t;
            }
        }
        double s = cumLength[bestSeg] + bestT * (cumLength[bestSeg + 1] - cumLength[bestSeg]);
        return new Projection(s, bestSeg, bestDist);
    }

    //the class we need to use for the path tracker
    public static class Projection {
        public final double s; // meters along the path
        public final int segment; // segment the closest point is on
        public final double distance; // how far off the path the robot is

        Projection(double s, int segment, double distance){
            this.s = s;
            this.segment = segment;
            this.distance = distance;
        }
    }
}
