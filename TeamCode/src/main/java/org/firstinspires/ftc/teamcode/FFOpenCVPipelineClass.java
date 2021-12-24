package org.firstinspires.ftc.teamcode;

import android.bluetooth.BluetoothA2dp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//Making the Pipeline

public class FFOpenCVPipelineClass extends OpenCvPipeline {



    //Mat = Matric
    //Making the YCbCr Matric
    Mat YCbCr = new Mat();


    public enum Location {

        LEFT,
        MIDDLE,
        RIGHT,
        NOTHING

    }
    static Location location;

    //ROI = Region Of Interest
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(100, 225));

    static final Rect MIDDLE_ROI = new Rect(
            new Point(101, 0),
            new Point(200, 225));

    static final Rect RIGHT_ROI = new Rect(
            new Point(201, 0),
            new Point(300, 225));

    static double PERCENT_COLOR_THRESHOLD = 0.01;


    @Override
    //processFrame calculates where the element is depending on its color
    public Mat processFrame (Mat Input){

        Imgproc.cvtColor(Input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

        Scalar lowYCrCb = new Scalar(53, 62, 27);
        Scalar highYCrCb = new Scalar(128, 139, 109);



        /*
        Yellow by Vikrant
        Scalar lowYCrCb = new Scalar(20, 142, 20);
        Scalar highYCrCb = new Scalar(255, 163, 90);

        Blue
        Scalar lowYCrCb = new Scalar(20, 70, 140);
        Scalar highYCrCb = new Scalar(255, 120, 190);

        Green
        Scalar lowYCrCb = new Scalar(53, 62, 27);
        Scalar highYCrCb = new Scalar(133, 142, 107);

        Teal by Vikrant DID NOT WORK

        Scalar lowYCrCb = new Scalar(70, 89, 141);
        Scalar highYCrCb = new Scalar(255, 89, 141);

        Teal by Naresh DID NOT WORK
        Scalar lowYCrCb = new Scalar(100, 67, 149);
        Scalar highYCrCb = new Scalar(145, 34, 54);

        Cyan Range from https://www.rapidtables.com/web/color/cyan-color.html
        Scalar lowYCrCb = new Scalar(93, 72, 147);
        Scalar highYCrCb = new Scalar(227, 114, 133);

        */

        Core.inRange(YCbCr, lowYCrCb, highYCrCb, YCbCr);

        Mat left = YCbCr.submat(LEFT_ROI);
        Mat middle = YCbCr.submat(MIDDLE_ROI);
        Mat right = YCbCr.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        middle.release();
        right.release();

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneMiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean nothingDetectedLevel1 = leftValue < PERCENT_COLOR_THRESHOLD;
        boolean nothingDetectedLevel2 = middleValue < PERCENT_COLOR_THRESHOLD;
        boolean nothingDetectedLevel3 = rightValue < PERCENT_COLOR_THRESHOLD;

        if (stoneLeft) {

            location = location.LEFT;


        } else if (stoneMiddle) {

            location = location.MIDDLE;

        } else if (stoneRight) {

            location = location.RIGHT;

        }
        else if (nothingDetectedLevel1){

            location = location.NOTHING;

        }
        else if (nothingDetectedLevel2){

            location = location.NOTHING;

        }
        else if (nothingDetectedLevel3){

            location = location.NOTHING;


        }

        Imgproc.cvtColor(YCbCr, YCbCr, Imgproc.COLOR_GRAY2RGB);

        Scalar elementNotDetected = new Scalar(255, 0, 0);
        Scalar elementDetected = new Scalar(0, 255, 0);

        Imgproc.rectangle(YCbCr, LEFT_ROI, location == Location.LEFT? elementDetected:elementNotDetected );
        Imgproc.rectangle(YCbCr, MIDDLE_ROI, location == Location.MIDDLE? elementDetected:elementNotDetected );
        Imgproc.rectangle(YCbCr, RIGHT_ROI, location == Location.RIGHT? elementDetected:elementNotDetected );




        return YCbCr;

    }



    //returns the location that we found above ^^^^^^^^^^^^^^^^^^^^^
    static Location getLocation() {

        return location;


    }

}





