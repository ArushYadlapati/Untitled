package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;



/*public class ImageProcessing {
    private void takePhoto () {
        Mat src = Imgcodecs.imread("VisionPortal-CameraFrameCapture-000000");
        Mat dst = Imgcodecs.imwrite("CameraFrameCapture0-HSV");
        //Mat dst1 = "CameraFrameCapture0-HSV-LowRed";
        //Mat dst2 = "CameraFrameCapture0-HSV-HighRed";
        //converts the file from an RGB file to HSV
        Imgproc.cvtColor(src, dst, Imgproc.COLOR_RGB2HSV);

        //Gets red pixels (camera, not game element)
//        Scalar lowerBound = new Scalar(0,200, 30);
//        Scalar upperBound = new Scalar(5,255,255);
        //Core.inRange((src, lowerBound,upperBound, dst1));

    }

}
*/