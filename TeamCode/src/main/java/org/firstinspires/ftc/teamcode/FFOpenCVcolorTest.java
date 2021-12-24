package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "FF_OpenCV_Color_Test", group = "Concept")
public class FFOpenCVcolorTest extends LinearOpMode{
    //Webcam variable
    OpenCvWebcam webcam;



    @Override
    public void runOpMode() {

        int Level;

        //Instance of OpenCV class
        FFOpenCVPipelineClass opencv = new FFOpenCVPipelineClass();
        //FFOpenCV opencv;

        //Initializing Camera
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));

        //The OpenCV class is determining what is being shown on the camera. It is the pipeline
        //webcam.setPipeline(opencv);

        //webcam.setMillisecondsPermissionTimeout(2500);

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        webcam.setPipeline(new FFOpenCVPipelineClass());
        webcam.setMillisecondsPermissionTimeout(2500);



        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        //Uses getLocation function from OpenCV class to find and display Level
        if (opModeIsActive()) {

            switch (opencv.getLocation()){

                case LEFT:
                    Level = 1;
                    telemetry.addData("Level", 1);
                    telemetry.update();
                    sleep(1000);
                    break;
                case MIDDLE:
                    Level = 2;
                    telemetry.addData("Level", 2);
                    telemetry.update();
                    sleep(1000);
                    break;
                case RIGHT:
                    Level = 3;
                    telemetry.addData("Level", 3);
                    telemetry.update();
                    sleep(1000);
                    break;



            }

        }


    }



}



