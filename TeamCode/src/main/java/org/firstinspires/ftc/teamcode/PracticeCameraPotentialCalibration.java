package org.firstinspires.ftc.teamcode;




import static java.lang.Boolean.TRUE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


import java.util.List;


/*
 * This OpMode illustrates the basics of using both AprilTag recognition and TensorFlow
 * Object Detection.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: Double Vision", group = "Pushbot")

public class PracticeCameraPotentialCalibration extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private AprilTagLibrary.Builder aprilTagLibraryBuilder;
    private AprilTagLibrary aprilTagLibrary;
    private AprilTagMetadata aprilTagMetadata;
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;


    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;


    @Override
    public void runOpMode() {
        initDoubleVision();


        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and TensorFlow Object Detection (TFOD) image processors.
        while (!isStopRequested())  {


            if (opModeInInit()) {
                telemetry.addData("DS preview on/off","3 dots, Camera Stream");
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
            }


            if (myVisionPortal.getProcessorEnabled(aprilTag)) {
                // User instructions: Dpad left or Dpad right.
                telemetry.addLine("Dpad Left to disable AprilTag");
                telemetry.addLine();
                telemetryAprilTag();
            } else {
                telemetry.addLine("Dpad Right to enable AprilTag");
            }
            telemetry.addLine();
            telemetry.addLine("----------------------------------------");
            if (myVisionPortal.getProcessorEnabled(tfod)) {
                telemetry.addLine("Dpad Down to disable TFOD");
                telemetry.addLine();
                telemetryTfod();
            } else {
                telemetry.addLine("Dpad Up to enable TFOD");
            }


            // Push telemetry to the Driver Station.
            telemetry.update();


            if (gamepad1.dpad_left) {
                myVisionPortal.setProcessorEnabled(aprilTag, false);
            } else if (gamepad1.dpad_right) {
                myVisionPortal.setProcessorEnabled(aprilTag, true);
            }
            if (gamepad1.dpad_down) {
                myVisionPortal.setProcessorEnabled(tfod, false);
            } else if (gamepad1.dpad_up) {
                myVisionPortal.setProcessorEnabled(tfod, true);
            }


            sleep(20);


        }   // end while loop


    }   // end method runOpMode()




    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------
        // Creates a new AprilTagLibrary.Builder object and assigns it to a variable.
        // Adds all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
        // Gets the AprilTagLibrary for the current season.
        aprilTagLibraryBuilder = new AprilTagLibrary.Builder().addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        aprilTagLibraryBuilder.setAllowOverwrite(true);
        // Create a new AprilTagMetdata object and assign it to a variable.
        String alliance = "";
        String place = "";
        for(int a=0;a<2;a++)
        {
            for(int s=1;s<4;s++)
            {
                if(a==0)
                {alliance = "Blue";}
                else
                {alliance = "Red";}
                if(s==1)
                {place = "Left";}
                else if(s==2)
                {place = "Center";}
                else
                {place = "Right";}
                // Add a tag to the AprilTagLibrary.Builder.
                aprilTagLibraryBuilder.addTag(new AprilTagMetadata((a*3)+(s), alliance+" Alliance "+place+" ", 0.166, DistanceUnit.METER));
            }
        }

        // Build the AprilTag library and assign it to a variable.
        aprilTagLibrary = aprilTagLibraryBuilder.build();

        aprilTag = new AprilTagProcessor.Builder().setTagLibrary(aprilTagLibrary)
                .build();


        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------


        tfod = new TfodProcessor.Builder()
                .build();


        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------


        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());


        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop


    }   // end method telemetryAprilTag()


    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;


            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop


    }   // end method telemetryTfod()


}   // end class