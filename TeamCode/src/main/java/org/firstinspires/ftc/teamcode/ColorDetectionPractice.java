package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraCalibration;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;
@TeleOp(name="ColorDectionPractice", group="Pushbot")
public class ColorDetectionPractice extends OpMode {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });

    }

    @Override
    public void loop() {
        telemetry.addData("Image Analysis:",pipeline.getAnalysis());
        telemetry.update();
    }


}

class SamplePipeline extends OpenCvPipeline {

    Mat HSV = new Mat();
    Mat HSV1 = new Mat();
    Mat HSV2 = new Mat();
    Mat Y = new Mat();
    int avg;
    Scalar lowerBound, upperBound;


    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToBinary(Mat input) {
        //Convert to HSV
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        //Blur images to remove sensor noise
        //Imgproc.blur(input,HSV,new Size(3,3));

        //Create Kernel Structuring Element
        //Mat Kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,new Size(3,3));
        //Erode
        //Imgproc.erode(input,HSV,Kernel);
        //Dilate
        //Imgproc.dilate(input,HSV,Kernel);

        //Get red pixels(hue >0)
        lowerBound = new Scalar(0,100,100);
        upperBound = new Scalar(10,255,255);
        Core.inRange(input,lowerBound,upperBound,HSV1);

        //Get red pixels (hue < 180)
        lowerBound = new Scalar(160,100,100);
        upperBound = new Scalar(179,255,255);
        Core.inRange(input,lowerBound,upperBound,HSV2);

        //Combine binary images to get all red pixels
        Core.add(HSV1,HSV2,HSV);

        //ArrayList<Mat> HSVChannels = new ArrayList<Mat>(3);
        //Core.split(HSV, HSVChannels);
        //Y = HSVChannels.get(0);


    }

    @Override
    public void init(Mat firstFrame) {
        inputToBinary(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToBinary(input);
        System.out.println("processing requested");
        //avg = (int) Core.mean(Y).val[0];
        //HSV.release(); // don't leak memory!
        HSV1.release(); // don't leak memory!
        HSV2.release(); // don't leak memory!
        Y.release(); // don't leak memory!
        if(HSV == null){HSV.release();return input;}
        else{return HSV;}
    }

    public int getAnalysis() {
        return avg;
    }
}