package org.firstinspires.ftc.team8923_2018;

import android.graphics.Bitmap;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class OpenCV extends OpenCVPipeline
{
    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.

    private List<Mat> channels = new ArrayList<>();
    private Mat maskYellow = new Mat();  // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat();  // hierarchy used by contours
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)

    private Rect goldRect = new Rect(0,0,0,0);

    // this is just here so we can expose it later thru getContours.
    private List<MatOfPoint> contours = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }

    public synchronized Rect getGoldRect() {
        return goldRect;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        rgba.copyTo(displayMat);
        // filter yellow
        Imgproc.cvtColor(rgba, rgba, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(rgba,rgba,new Size(3,3),0);
        channels = new ArrayList<>();
        Core.split(rgba, channels);
        if(channels.size() > 0){
            Imgproc.threshold(channels.get(1), maskYellow, 90, 255, Imgproc.THRESH_BINARY_INV);
        }

        //Find contours of the yellow mask and draw them to the display mat for viewing

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        if (showContours) Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        double area;
        double maxArea = 0.0;
        Rect rect;
        Rect maxRect = new Rect(0, 0, 0, 0);   // Rect with max area

        // Loop through the contours and find the contour with max area
        for(MatOfPoint cont : contoursYellow){

            // Get bounding rect of contour
            rect = Imgproc.boundingRect(cont);
            area = Imgproc.contourArea(cont);
            if (area > maxArea && rect.x < 80)//max rect is biggest yellow rectangle
            {
                maxArea = area;
                maxRect = rect;
            }
        }
        goldRect = maxRect;

        Imgproc.rectangle(displayMat, maxRect.tl(), maxRect.br(), new Scalar(0,0,255),2); // Draw rect
        // Draw Current X
        Imgproc.putText(displayMat, "Gold", maxRect.tl(),0,1,new Scalar(255,255,255));

        return displayMat;
    }
}
