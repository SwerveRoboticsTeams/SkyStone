package org.firstinspires.ftc.team6220_2019;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class SkystoneDetectionOpenCV extends OpenCVPipeline {

    private Mat blurred = new Mat();
    private Mat hsv = new Mat();
    private Mat yuv = new Mat();
    private Mat h = new Mat(), u = new Mat();
    private Mat finalMat = new Mat();
    private Mat cropped1, cropped2, cropped3;
    private Mat hLow = new Mat(), hHigh = new Mat();
    private ArrayList<Mat> yuvSplit = new ArrayList<>();
    private ArrayList<Mat> hsvSplit = new ArrayList<>();
    private double mean1, mean2, mean3;

    //todo once code works, make these constants in Constants class
    private int[] section1 = {100, 100, 100, 100}; //{N, W, width, height}
    private int[] section2 = {100, 200, 100, 100};
    private int[] section3 = {100, 300, 100, 100};

    @Override
    public Mat processFrame(Mat rgba, Mat gray){
        Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_RGBA2RGB);

        Imgproc.GaussianBlur(rgba, blurred, new Size(5, 5), 0);
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(rgba, yuv, Imgproc.COLOR_RGB2YUV);

        Core.split(yuv, yuvSplit);
        Core.split(hsv, hsvSplit);

        u = yuvSplit.get(1);
        h = hsvSplit.get(0);

        Imgproc.threshold(u, u,90, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(h, hLow, 15, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(h, hHigh, 30, 255, Imgproc.THRESH_BINARY_INV);
        Core.min(hLow, hHigh, h);

        Core.min(u, h, finalMat);

        Rect sect1 = new Rect(section1[0], section1[1], section1[2], section1[3]);
        Rect sect2 = new Rect(section2[0], section2[1], section2[2], section2[3]);
        Rect sect3 = new Rect(section3[0], section3[1], section3[2], section3[3]);

        cropped1 = new Mat(finalMat, sect1);
        cropped2 = new Mat(finalMat, sect2);
        cropped3 = new Mat(finalMat, sect3);

        mean1 = Core.mean(cropped1).val[0];
        mean2 = Core.mean(cropped2).val[1];
        mean3 = Core.mean(cropped3).val[2];

        return new Mat();
    }

}
