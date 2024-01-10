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
                telemetry.addData("Image Analysis:",pipeline.getAnalysis());
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
                //webcam.stopStreaming();
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
    Mat HSV3 = new Mat();
    Mat SubMat1 = new Mat();
    Mat SubMat2 = new Mat();
    Mat SubMat3 = new Mat();
    Mat Y = new Mat();
    int avg1,avg2,avg3;
    Scalar lowerBound, upperBound;
    boolean identifyRedAndBlue = true;
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    static final int WidthRect = 130;
    static final int HeightRect = 110;
    static final double HeightRatio = (2.5/6.5);
    static double CheckHeight =(STREAM_HEIGHT-(STREAM_HEIGHT*HeightRatio));


    //3.5/5.4 left far
    //2.5/6.5 up
    //SubMat1 = HSV.submat(new Rect(0,0,630,1080));
    //SubMat2 = HSV.submat(new Rect(640,0,630,1080));
    //SubMat3 = HSV.submat(new Rect(1280,0,630,1080));
    Point LeftTopCorner = new Point(0,CheckHeight-1);
    Point LeftBottomCorner = new Point((STREAM_WIDTH/3)-10,STREAM_HEIGHT-1);
    Point CenterTopCorner = new Point(STREAM_WIDTH/3,CheckHeight-1);
    Point CenterBottomCorner = new Point(((STREAM_WIDTH/3)*2)-10,STREAM_HEIGHT-1);
    Point RightTopCorner = new Point((STREAM_WIDTH/3)*2,CheckHeight);
    Point RightBottomCorner = new Point(STREAM_WIDTH-10,STREAM_HEIGHT-1);

    void openCVPointInitialization(int streamWidth,int streamHeight, double heightRatio, double widthRatio){
        double CheckHeight =(streamHeight-(streamHeight*heightRatio));

        LeftTopCorner = new Point(0,CheckHeight-1);
        LeftBottomCorner = new Point((streamWidth/3)-10,streamHeight-1);
        CenterTopCorner = new Point(streamWidth/3,CheckHeight-1);
        CenterBottomCorner = new Point(((streamWidth/3)*2)-10,streamHeight-1);
        RightTopCorner = new Point((streamWidth/3)*2,CheckHeight);
        RightBottomCorner = new Point(streamWidth-10,streamHeight-1);
    }
    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToBinary(Mat input) {
        //Convert to HSV
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        if(identifyRedAndBlue){
            //Get red pixels(hue >0)
            lowerBound = new Scalar(0d,100d,100d);
            upperBound = new Scalar(10d,255d,255d);
            Core.inRange(HSV,lowerBound,upperBound,HSV1);


            //Get red pixels (hue < 180)
            lowerBound = new Scalar(160,100,100);
            upperBound = new Scalar(179,255,255);
            Core.inRange(HSV,lowerBound,upperBound,HSV2);

            //Get Blue Pixels
            lowerBound = new Scalar(100,90,90);
            upperBound = new Scalar(130,255,255);
            Core.inRange(HSV,lowerBound,upperBound,HSV3);

            //Combine binary images to get all red pixels
            Core.add(HSV1,HSV2,HSV);
            Core.add(HSV,HSV3,HSV);

        }
    }

    @Override
    public void init(Mat firstFrame) {
        inputToBinary(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToBinary(input);
        System.out.println("processing requested");

        //Release The Matrices
        HSV1.release(); // don't leak memory!
        HSV2.release(); // don't leak memory!
        HSV3.release(); // don't leak memory!
        Y.release(); // don't leak memory!

        //Create Sub-matrix to analyze separate parts of the image
        SubMat1 = HSV.submat(new Rect(LeftTopCorner,LeftBottomCorner));
        SubMat2 = HSV.submat(new Rect(CenterTopCorner,CenterBottomCorner));
        SubMat3 = HSV.submat(new Rect(RightTopCorner,RightBottomCorner));

        //Get the Average Brightness from Each Sub-matrix
        avg1 = (int) Core.mean(SubMat1).val[0];
        avg2 = (int) Core.mean(SubMat2).val[0];
        avg3 = (int) Core.mean(SubMat3).val[0];

        //Release the Sub-mats
        SubMat1.release();
        SubMat2.release();
        SubMat3.release();

        //Draw the Rectangles on the screen
        Imgproc.rectangle( // rings
                HSV, // Buffer to draw on
                LeftTopCorner, // First point which defines the rectangle
                LeftBottomCorner, // Second point which defines the rectangle
                new Scalar(100,100,255), // The color the rectangle is drawn in
                50); // Thickness of the rectangle lines
        Imgproc.rectangle(HSV, CenterTopCorner, CenterBottomCorner, new Scalar(100,100,255),50);
        Imgproc.rectangle(HSV, RightTopCorner, RightBottomCorner, new Scalar(100,100,255),50);
        return HSV;
    }

    public String getAnalysis() {
        /*Determines which rectangle has the Team Object within its boundaries*/
        if((avg1 >avg2)&&(avg1>avg3)){return "Left:   "+avg1;}
        if((avg2 >avg1)&&(avg2>avg3)){return "Center:    "+avg2;}
        if((avg3 >avg2)&&(avg3>avg1)){return "Right:   "+avg3;}
        return "ErrorInAnalysis";
    }
}