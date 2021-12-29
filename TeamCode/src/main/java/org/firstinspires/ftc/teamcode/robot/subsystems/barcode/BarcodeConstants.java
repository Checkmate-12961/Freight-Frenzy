package org.firstinspires.ftc.teamcode.robot.subsystems.barcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;
import org.opencv.core.Rect;

@Config
public class BarcodeConstants {
    public static int leftBoxX = 20;
    public static int leftBoxY = 145;
    public static int leftBoxWidth = 50;
    public static int leftBoxHeight = 50;

    public static Point getLeftBoxPointA() {
        return new Point(
                BarcodeConstants.leftBoxX,
                BarcodeConstants.leftBoxY);
    }

    public static Point getLeftBoxPointB() {
        return new Point(
                BarcodeConstants.leftBoxX + BarcodeConstants.leftBoxWidth,
                BarcodeConstants.leftBoxY + BarcodeConstants.leftBoxHeight);
    }
    
    public static Rect getLeftBox() {
        return new Rect(
                BarcodeConstants.getLeftBoxPointA(),
                BarcodeConstants.getLeftBoxPointB());
    }

    public static int middleBoxX = 225;
    public static int middleBoxY = 145;
    public static int middleBoxWidth = 50;
    public static int middleBoxHeight = 50;

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

    public static int rightBoxX = 0;
    public static int rightBoxY = 0;
    public static int rightBoxWidth = 0;
    public static int rightBoxHeight = 0;

    public static Point getRightBoxPointA() {
        return new Point(
                BarcodeConstants.rightBoxX,
                BarcodeConstants.rightBoxY);
    }

    public static Point getRightBoxPointB() {
        return new Point(
                BarcodeConstants.rightBoxX + BarcodeConstants.rightBoxWidth,
                BarcodeConstants.rightBoxY + BarcodeConstants.rightBoxHeight);
    }

    public static Rect getRightBox() {
        return new Rect(
                BarcodeConstants.getRightBoxPointA(),
                BarcodeConstants.getRightBoxPointB());
    }
}
