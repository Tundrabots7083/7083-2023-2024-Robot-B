package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.field.TeamElementLocation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class VisionProcessor implements org.firstinspires.ftc.vision.VisionProcessor {
    public static int LEFT_RECTANGLE_X = 0;
    public static int LEFT_RECTANGLE_WIDTH = 155;
    public static int LEFT_RECTANGLE_Y = 365;
    public static int LEFT_RECTANGLE_HEIGHT = 105;
    public static int MIDDLE_RECTANGLE_X = 375;
    public static int MIDDLE_RECTANGLE_WIDTH = 250;
    public static int MIDDLE_RECTANGLE_Y = 310;
    public static int MIDDLE_RECTANGLE_HEIGHT = 95;

    public static double MIN_PERCENT_DIFFERENCE = 33;
    public static double MIN_LEFT_SAT = 50;
    public static double MIN_MIDDLE_SAT = 30;

    private final Telemetry telemetry;
    public Rect rectLeft = new Rect(LEFT_RECTANGLE_X, LEFT_RECTANGLE_Y, LEFT_RECTANGLE_WIDTH, LEFT_RECTANGLE_HEIGHT);
    public Rect rectMiddle = new Rect(MIDDLE_RECTANGLE_X, MIDDLE_RECTANGLE_Y, MIDDLE_RECTANGLE_WIDTH, MIDDLE_RECTANGLE_HEIGHT);

    TeamElementLocation selection = TeamElementLocation.UNKNOWN;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    public VisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);

        telemetry.addData("[VISION] Left Spike", satRectLeft);
        telemetry.addData("[VISION] Middle Spike", satRectMiddle);

        double percentDifference = getPercentDifference(satRectLeft, satRectMiddle);
        telemetry.addData("[VISION] Percent Difference", percentDifference);

        if (percentDifference < MIN_PERCENT_DIFFERENCE) {
            return TeamElementLocation.RIGHT_SPIKE_MARK;
        } else if (satRectLeft > satRectMiddle && satRectLeft > MIN_LEFT_SAT) {
            return TeamElementLocation.LEFT_SPIKE_MARK;
        } else if (satRectMiddle > satRectLeft && satRectMiddle > MIN_MIDDLE_SAT) {
            return TeamElementLocation.MIDDLE_SPIKE_MARK;
        }
        return TeamElementLocation.RIGHT_SPIKE_MARK;
    }

    private double getPercentDifference(double val1, double val2) {
        return Math.abs(val1 - val2) / ((val1 + val2) / 2) * 100;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);

        selection = (TeamElementLocation) userContext;
        telemetry.addData("Selection", selection);
        switch (selection) {
            case LEFT_SPIKE_MARK:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
            case MIDDLE_SPIKE_MARK:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                break;
            case RIGHT_SPIKE_MARK:
            case UNKNOWN:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
        }
    }

    public TeamElementLocation getSelection() {
        return selection;
    }
}
