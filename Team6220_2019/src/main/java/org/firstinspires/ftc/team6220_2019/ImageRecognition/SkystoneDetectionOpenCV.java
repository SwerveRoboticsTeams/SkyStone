package org.firstinspires.ftc.team6220_2019.ImageRecognition;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class SkystoneDetectionOpenCV extends OpenCVPipeline
{

    private Mat blurred = new Mat();
    private Mat hsv = new Mat();
    private Mat yuv = new Mat();
    private Mat h = new Mat(), u = new Mat();
    private Mat finalMat = new Mat();
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat cropped1, cropped2, cropped3;
    private Mat hLow = new Mat(), hHigh = new Mat();
    private ArrayList<Mat> yuvSplit = new ArrayList<>();
    private ArrayList<Mat> hsvSplit = new ArrayList<>();
    private double mean1, mean2, mean3;

    // todo Adjust w/ actual stones, make constants.
    // {horizontal position, vertical position, width, height}
    private int[] section1 = {120, 160, 80, 160};   // Left stone
    private int[] section2 = {280, 160, 80, 160};   // Middle stone
    private int[] section3 = {440, 160, 80, 160};   // Right stone

    @Override
    public Mat processFrame(Mat rgba, Mat gray)
    {
        // Use displayMat to show contours on phone and rgba for actual data manipulation.
        rgba.copyTo(displayMat);

        // Convert RGBA to RGB to remove alpha
        Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_RGBA2RGB);

        // Blur image to remove effect of noise
        Imgproc.GaussianBlur(rgba, blurred, new Size(5, 5), 0);

        // Convert rgb into hsv and yuv
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(rgba, yuv, Imgproc.COLOR_RGB2YUV);

        // Store yuv and hsv splits into Mat ArrayLists
        Core.split(yuv, yuvSplit);
        Core.split(hsv, hsvSplit);

        // Retrieve the u and h from the ArrayLists
        u = yuvSplit.get(1);
        h = hsvSplit.get(0);

        // u threshold detects yellow if u < 90
        Imgproc.threshold(u, u, 90, 255, Imgproc.THRESH_BINARY_INV);

        // h threshold detects yellow if 15 < h < 30
        Imgproc.threshold(h, hLow, 15, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(h, hHigh, 30, 255, Imgproc.THRESH_BINARY_INV);
        Core.min(hLow, hHigh, h);

        // Yellow is detected only if both u and h detect yellow
        Core.min(u, h, finalMat);

        // Take image subsets where stones should be
        Rect sect1 = new Rect(section1[0], section1[1], section1[2], section1[3]);
        Rect sect2 = new Rect(section2[0], section2[1], section2[2], section2[3]);
        Rect sect3 = new Rect(section3[0], section3[1], section3[2], section3[3]);

        cropped1 = new Mat(finalMat, sect1);
        cropped2 = new Mat(finalMat, sect2);
        cropped3 = new Mat(finalMat, sect3);

        // Evaluate mean color value in each image subset. mean1, mean2, mean3 can now be called from the getter methods.
        mean1 = Core.mean(cropped1).val[0];
        mean2 = Core.mean(cropped2).val[0];
        mean3 = Core.mean(cropped3).val[0];

        // Draw image subset boundaries for the purposes of testing and debugging.
        Imgproc.rectangle(displayMat, new Point(section1[0], section1[1]),
                new Point(section1[0] + section1[3], section1[1] + section1[2]),
                new Scalar(255, 0, 0), 5);
        Imgproc.rectangle(displayMat, new Point(section2[0], section2[1]),
                new Point(section2[0] + section2[3], section2[1] + section2[2]),
                new Scalar(0, 255, 0), 5);
        Imgproc.rectangle(displayMat, new Point(section3[0], section3[1]),
                new Point(section3[0] + section3[3], section3[1] + section3[2]),
                new Scalar(0, 0, 255), 5);

        return displayMat;
    }

    public double getMean1()
    {
        return mean1;
    }

    public double getMean2()
    {
        return mean2;
    }

    public double getMean3()
    {
        return mean3;
    }

}
