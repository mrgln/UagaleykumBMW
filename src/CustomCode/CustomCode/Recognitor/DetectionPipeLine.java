package org.firstinspires.ftc.teamcode.Detectio;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
@Config
public class DetectionPipeLine extends OpenCvPipeline {
    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    public static int outputMode = 0;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 40;
    public static double strictHighS = 255;

    public DetectionPipeLine() {
        frameList = new ArrayList<>();
    }

    public static double redLowHue = 0;
    public static double redHighHue = 8;

    public static double orangeLowHue = 15;
    public static double orangeHighHue = 30;

    public static double blueLowHue = 90;
    public static double blueHighHue = 140;

    @Override
    public void init(Mat mat) {
        blueLineAngle = 90;
        super.init(mat);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Imgproc.medianBlur(mat, mat, 5);

        Scalar highRedLowHSV = new Scalar(redLowHue, 70, 70);
        Scalar highRedHighHSV = new Scalar(redHighHue, 255, 255);
        Scalar lowRedLowHSV = new Scalar(160, 70, 70);
        Scalar lowRedHighHSV = new Scalar(255, 255, 255);

        Mat redThreshHigh = new Mat();
        Mat redThreshLow = new Mat();

        Core.inRange(mat, highRedLowHSV, highRedHighHSV, redThreshHigh);
        Core.inRange(mat, lowRedLowHSV, lowRedHighHSV, redThreshLow);
//        return redThreshHigh;

        Mat redMasked = new Mat();
//        //color the white portion of thresh in with HSV from mat
//        //output into masked
        Core.bitwise_and(mat, mat, redMasked, redThreshHigh);
        Core.bitwise_or(mat, redMasked, redMasked, redThreshLow);

        //calculate average HSV values of the white thresh values
        Scalar redAverage = Core.mean(redMasked, redThreshHigh);

        Mat redScaledMask = new Mat();
    //scale the average saturation to 150
        redMasked.convertTo(redScaledMask, -1, 150 / redAverage.val[1], 0);

        Mat redScaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0);
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255);

        Core.inRange(redScaledMask, strictLowHSV, strictHighHSV, redScaledThresh);

        Mat redEdges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(redScaledThresh, redEdges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> redContours = new ArrayList<>();
        Mat redHierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(redScaledThresh, redContours, redHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double curRedArea = 0;

        for (MatOfPoint contour : redContours)
        {
            curRedArea = Math.max(curRedArea, Imgproc.contourArea(contour));
            contour.release();
        }

        redArea = curRedArea;




        Scalar highGreenLowHSV = new Scalar(40, 50, 40);
        Scalar highGreenHighHSV = new Scalar(85, 255, 255);

        Mat greenThresh = new Mat();

        Core.inRange(mat, highGreenLowHSV, highGreenHighHSV, greenThresh);
//        return redThreshHigh;

        Mat greenMasked = new Mat();
//        //color the white portion of thresh in with HSV from mat
//        //output into masked
        Core.bitwise_and(mat, mat, greenMasked, greenThresh);

        //calculate average HSV values of the white thresh values
        Scalar greenAverage = Core.mean(greenMasked, greenThresh);

        Mat greenScaledMask = new Mat();
        //scale the average saturation to 150
        greenMasked.convertTo(greenScaledMask, -1, 150 / greenAverage.val[1], 0);


        Mat greenScaledThresh = new Mat();
        //you probably want to tune this
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(greenScaledMask, strictLowHSV, strictHighHSV, greenScaledThresh);


        Mat greenEdges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(greenScaledThresh, greenEdges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> greenContours = new ArrayList<>();
        Mat greenHierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(greenScaledThresh, greenContours, greenHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double curGreenArea = 0;

        for (MatOfPoint contour : greenContours)
        {
            curGreenArea = Math.max(curGreenArea, Imgproc.contourArea(contour));
            contour.release();
        }

        greenArea = curGreenArea;


        Scalar highBlueLowHSV = new Scalar( blueLowHue,  30, 55);
        Scalar highBlueHighHSV = new Scalar(blueHighHue, 255, 255);

        Mat blueThresh = new Mat();

        Core.inRange(mat, highBlueLowHSV, highBlueHighHSV, blueThresh);
//        return redThreshHigh;

        Mat blueMasked = new Mat();
//        //color the white portion of thresh in with HSV from mat
//        //output into masked
        Core.bitwise_and(mat, mat, blueMasked, blueThresh);

        //calculate average HSV values of the white thresh values
        Scalar blueAverage = Core.mean(blueMasked, blueThresh);

        Mat blueScaledMask = new Mat();
        //scale the average saturation to 150
        blueMasked.convertTo(blueScaledMask, -1, 150 / blueAverage.val[1], 0);


        Mat blueScaledThresh = new Mat();
        //you probably want to tune this
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(blueScaledMask, strictLowHSV, strictHighHSV, blueScaledThresh);


        Mat blueEdges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(blueScaledThresh, blueEdges, 40, 80);

        //contours, apply post processing to information
        List<MatOfPoint> blueContours = new ArrayList<>();
        Mat blueHierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(blueScaledThresh, blueContours, blueHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat lines = new Mat();

        lines.release();
        Imgproc.HoughLinesP(blueEdges, lines, 1.0, Math.PI/180, 100, 74, 50);

        double maxRho = 0;

        for (int i = 0; i < lines.rows(); i++)
        {
            double[] stats = lines.get(i, 0);

            double theta1 = stats[3] - stats[1];
            double theta2 = stats[2] - stats[0];
            double hyp = Math.hypot(theta1, theta2);

            if (maxRho < hyp)
            {
                maxRho = hyp;
                double angle = Math.toDegrees(Math.atan2(stats[1] - stats[3], stats[0] - stats[2]));
                if (angle > 0)
                {
                    angle = 180 - angle;
                }
                else if (angle < 0)
                {
                    angle = -180 - angle;
                }
                blueLineAngle = angle;
            }
        }


        if (frameList.size() > 5) {
            frameList.remove(0);
        }


        if (outputMode > 0 && outputMode < 4)
            input.release();
        mat.release();

        if (outputMode == 2)
        redScaledThresh.copyTo(input);
        redScaledThresh.release();
        redScaledMask.release();
        redMasked.release();
        redEdges.release();
        redThreshHigh.release();
        redThreshLow.release();
        redHierarchy.release();

//        if (outputMode == 1)
//            orScaledThresh.copyTo(input);
//        orScaledThresh.release();
//        orScaledMask.release();
//        orMasked.release();
//        orEdges.release();
//        orThresh.release();
//        orHierarchy.release();

        if (outputMode == 1)
            blueScaledThresh.copyTo(input);
        blueScaledThresh.release();
        blueScaledMask.release();
        blueMasked.release();
        blueEdges.release();
//        blueThresh.copyTo(input);
        blueThresh.release();
        blueHierarchy.release();

        if (outputMode == 3)
            greenScaledThresh.copyTo(input);
        greenScaledThresh.release();
        greenScaledMask.release();
        greenMasked.release();
        greenEdges.release();
        greenThresh.release();
        greenHierarchy.release();

        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
//        return finalMask;

        return input;
//        return blueThreshHigh;
    }

    public static double redArea = 0;
    public static double greenArea = 0;
    public static double blueLineAngle = 0;

    public static double orangePositionY = 0;
}