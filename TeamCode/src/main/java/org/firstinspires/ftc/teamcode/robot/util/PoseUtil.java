package org.firstinspires.ftc.teamcode.robot.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseUtil {
    private static double x = 0;
    private static double y = 0;
    private static double heading = 0;

    public static Pose2d get(){
        return new Pose2d(x, y, heading);
    }

    public static void set(Pose2d pose){
        x = pose.getX();
        y = pose.getY();
        heading = pose.getHeading();
    }

    public static void set(double x, double y, double heading){
        PoseUtil.x = x;
        PoseUtil.y = y;
        PoseUtil.heading = heading;
    }
}
