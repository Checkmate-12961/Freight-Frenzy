package org.firstinspires.ftc.teamcode.robot.subsystems.barcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;
import org.opencv.core.Rect;

@Config
public class BarcodeConstants {
    //These define the location and size of the left box.
    public static int leftBoxX = 225;
    public static int leftBoxY = 145;
    public static int leftBoxWidth = 50;
    public static int leftBoxHeight = 50;
    public static int leftThreshold = 125;



    //Defines a point with the left boxes coordinates
    public static Point getLeftBoxPointA() {
        return new Point(
                BarcodeConstants.leftBoxX,
                BarcodeConstants.leftBoxY);
    }

    //Defines a second point by adding the length and width to the first point.
    public static Point getLeftBoxPointB() {
        return new Point(
                BarcodeConstants.leftBoxX + BarcodeConstants.leftBoxWidth,
                BarcodeConstants.leftBoxY + BarcodeConstants.leftBoxHeight);
    }

    //Defines a rectangle with the two points as corners.
    public static Rect getLeftBox() {
        return new Rect(
                BarcodeConstants.getLeftBoxPointA(),
                BarcodeConstants.getLeftBoxPointB());
    }

    //These define the location and size of the middle box
    //There is no right box because it would be out of frame
    public static int middleBoxX = 20;
    public static int middleBoxY = 145;
    public static int middleBoxWidth = 50;
    public static int middleBoxHeight = 50;
    public static int middleThreshold = 125;

    //Two points and a rectangle are created similar to the left box.
    public static Point getMiddleBoxPointA() {
        return new Point(
                BarcodeConstants.middleBoxX,
                BarcodeConstants.middleBoxY);
    }

    public static Point getMiddleBoxPointB() {
        return new Point(
                BarcodeConstants.middleBoxX + BarcodeConstants.middleBoxWidth,
                BarcodeConstants.middleBoxY + BarcodeConstants.middleBoxHeight);
    }

    public static Rect getMiddleBox() {
        return new Rect(
                BarcodeConstants.getMiddleBoxPointA(),
                BarcodeConstants.getMiddleBoxPointB());
    }


}
