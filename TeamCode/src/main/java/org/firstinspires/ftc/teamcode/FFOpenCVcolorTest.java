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

        int level = 0;

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



            }
        });

        if (opencv.getLocation() == FFOpenCVPipelineClass.Location.LEFT) {
            level = 1;
            telemetry.addData("Level", 1);
            telemetry.update();
            sleep(3000);

        } else if (opencv.getLocation() == FFOpenCVPipelineClass.Location.MIDDLE) {
            level = 2;
            telemetry.addData("Level", 2);
            telemetry.update();
            sleep(3000);

        } else if (opencv.getLocation() == FFOpenCVPipelineClass.Location.RIGHT) {
            level = 3;
            telemetry.addData("Level", 3);
            telemetry.update();
            sleep(3000);

        } else if (opencv.getLocation() == FFOpenCVPipelineClass.Location.NOTHING) {
            level = 0;
            telemetry.addLine("Nothing Detected");
            telemetry.update();
            sleep(3000);

        }

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        //Uses getLocation function from OpenCV class to find and display Level
        if (opModeIsActive()) {


            if (level == 1) {
                telemetry.addLine("Running Program Level 1");
                telemetry.update();
            }
            else if (level == 2) {

                telemetry.addLine("Running Program Level 2");
                telemetry.update();

            }
            else if (level == 3) {

                telemetry.addLine("Running Program Level 3");
                telemetry.update();

            }


        }


    }



}



