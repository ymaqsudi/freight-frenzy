package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Freight_Frenzy_Detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }

    private Location location;

    // camera sees 2 of the 3 spots, defining the 2 rects the camera sees

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));

    static double PERCENT_COLOR_THRESHOLD = 0.4;    // play with this number - amount of yellow making it a freight
    public Freight_Frenzy_Detector(Telemetry t) { telemetry = t; }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);    // convert rgb to hsv
        Scalar lowHSV = new Scalar(23, 50, 70);     // yellow
        Scalar highHSV = new Scalar(32, 255, 255);  // yellow

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat (LEFT_ROI);
        Mat right = mat.submat (RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);

        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean freight_left = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean freight_right = rightValue > PERCENT_COLOR_THRESHOLD;

        if (freight_left && freight_right) {
            // not found, idk man
            location = Location.NOT_FOUND;
            telemetry.addData("location", "idk");
        } else if (freight_left) {
            // left
            location = Location.LEFT;
            telemetry.addData("location", "left");
        } else {
            // right
            location = Location.RIGHT;
            telemetry.addData("location", "right");
        }
        telemetry.update();

        // convert greyscale back to rgb to draw colored rect

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar freight = new Scalar (255, 0, 0);
        Scalar empty = new Scalar (0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? freight:empty);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? freight:empty);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
