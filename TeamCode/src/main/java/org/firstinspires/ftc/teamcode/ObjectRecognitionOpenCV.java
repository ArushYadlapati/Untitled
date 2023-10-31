package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**This is a first cut at a program to access an image and do some image processing
 * We will use the following steps:
 * 1. Create a VisionPortal Builder
 * 2. Configure the Camera (webcam name, resolution etc.)
 * 3. Configure livestream (on driver hub)
 * 4. Add processor for OpenCV
 */
/*@Autonomous()
@Disabled
/*public class ObjectRecognitionOpenCV extends LinearOpMode {
    private VisionPortal visionPortal;
    private VisionProcessor TeamPropDetect;
    // create vision portal builder
    {
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Configure which camera to use
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));// use wideangle view to detect team Prop

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);
        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(true);

        // Set and enable the processor.

        builder.addProcessor(TeamPropDetect);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }

    class MyVisionProcessor implements VisionProcessor {
        @Override
        public void init(int width, int height, CameraCalibration calibration)
        {}
        @Override
        public Object processFrame(Mat frame, long captureTimeNanos)
        {return null;}
        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
        {}

        //Create custom OpenCV processor
        MyVisionProcessor customVisionProcessor = new MyVisionProcessor()
    }
*/