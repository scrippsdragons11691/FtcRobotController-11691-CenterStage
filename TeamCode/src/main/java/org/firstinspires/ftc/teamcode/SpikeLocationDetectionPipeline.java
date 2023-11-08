package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class SpikeLocationDetectionPipeline extends OpenCvPipeline {

    public Scalar lowerRedCone = new Scalar(35.9, 149.2, 5.7);
    public Scalar upperRedCone = new Scalar(150.2, 203.1, 255);
    public Scalar lowerBlueCone = new Scalar(0, 0, 136.0);
    public Scalar upperBlueCone = new Scalar(255, 255, 194.1);
    int numRedCones = 0;
    int numBlueCones = 0;
    public double erodeWidth = 5;
    public double erodeHeight = 15;
    public double dilateWidth = 15;
    public double dilateHeight = 18;
    public double areaLowThreshold = 50;
    public double areaHighThreshold = 200;
    public AtomicInteger spikeLocation = new AtomicInteger(0);
    boolean enableTelemetry = true;
    boolean addDrawings = true;

    public Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(120,0);

    public int REGION_WIDTH = 80;
    public int REGION_HEIGHT = 240;
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    double detectedOffset = 0;


    /**
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    public ColorSpace colorSpace = ColorSpace.YCrCb;

    /*
     * A good practice when typing EOCV pipelines is
     * declaring the Mats you will use here at the top
     * of your pipeline, to reuse the same buffers every
     * time. This removes the need to call mat.release()
     * with every Mat you create on the processFrame method,
     * and therefore, reducing the possibility of getting a
     * memory leak and causing the app to crash due to an
     * "Out of Memory" error.
     */
    private Mat region1_Mat = new Mat();
    private Mat ycrcbMat       = new Mat();
    private Mat redConeBinaryMat      = new Mat();
    private Mat blueConeBinaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();
    private Mat mask2 = null;

    private Telemetry telemetry = null;

    /**
     * Enum to choose which color space to choose
     * with the live variable tuner isntead of
     * hardcoding it.
     */
    enum ColorSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

    public SpikeLocationDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public AtomicInteger getSpikeLocation(){
        return spikeLocation;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, colorSpace.cvtCode);

        //create a submat for the center of the region
        //region1_Mat = ycrcbMat.submat(new Rect(region1_pointA, region1_pointB));

        //check red cones
        Core.inRange(ycrcbMat, lowerRedCone, upperRedCone, redConeBinaryMat);
        numRedCones = Core.countNonZero(redConeBinaryMat);

        //check blue cones
        Core.inRange(ycrcbMat, lowerBlueCone, upperBlueCone, blueConeBinaryMat);
        numBlueCones = Core.countNonZero(blueConeBinaryMat);

        maskedInputMat.release();

        //determine biggest
        if ( numRedCones > numBlueCones){
            Imgproc.erode(redConeBinaryMat, redConeBinaryMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeWidth, erodeHeight)));
            Imgproc.dilate(redConeBinaryMat, redConeBinaryMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateWidth, dilateHeight)));

            Mat heirarchyMat = new Mat();

            List<MatOfPoint> theContours = new ArrayList<>();
            Imgproc.findContours(redConeBinaryMat, theContours, heirarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            //Core.bitwise_and(input, input, maskedInputMat, redConeBinaryMat);
            //Imgproc.drawContours(maskedInputMat, theContours, -1, new Scalar(0, 255, 0), 2);
            Imgproc.drawContours(input, theContours, -1, new Scalar(0, 255, 0), 1);

            //need to find how many red cones vs blue
            List<Rect> theRectangles = new ArrayList<>();
            double maxRectangleWidth = 0;
            double maxRectX = 0;
            double maxRectY = 0;
            double maxRectangleCenter = 0;

            if (theContours.size() > 0) {
                for (int i = 0; i < theContours.size(); i++) {
                    Rect r = Imgproc.boundingRect(theContours.get(i));

                    double rectWidth = r.width;
                    //find the largest width
                    if (rectWidth > maxRectangleWidth){
                        maxRectangleWidth = rectWidth;
                        maxRectX = r.x;
                        maxRectY = r.y;
                    }

                }
                maxRectangleCenter = maxRectangleWidth / 2;
                detectedOffset = maxRectX + maxRectangleCenter - 160 ; //top left anchor point

            } else {
                detectedOffset = -1000;
            }

            setSpikeLocation(detectedOffset);

            if (addDrawings) {
                Imgproc.drawContours(input, theContours, -1, new Scalar(0, 255, 0), 1);

                //draw a red line for the offset of the largest rectangle
                Imgproc.line(input, new Point(maxRectX + maxRectangleCenter, 0), new Point(maxRectX + maxRectangleCenter, 240), new Scalar(255, 0, 0));

            }

        } else {
            Imgproc.erode(blueConeBinaryMat, blueConeBinaryMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeWidth, erodeHeight)));
            Imgproc.dilate(blueConeBinaryMat, blueConeBinaryMat, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateWidth, dilateHeight)));

            Mat heirarchyMat = new Mat();

            List<MatOfPoint> theContours = new ArrayList<>();
            Imgproc.findContours(blueConeBinaryMat, theContours, heirarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            //Core.bitwise_and(input, input, maskedInputMat, blueConeBinaryMat); //need to fix this to view cones
            //Imgproc.drawContours(maskedInputMat, theContours, -1, new Scalar(0, 255, 0), 1);
            Imgproc.drawContours(input, theContours, -1, new Scalar(0, 255, 0), 1);


            List<Rect> theRectangles = new ArrayList<>();
            double maxRectangleWidth = 0;
            double maxRectX = 0;
            double maxRectY = 0;
            double maxRectangleCenter = 0;

            if (theContours.size() > 0) {
                for (int i = 0; i < theContours.size(); i++) {
                    Rect r = Imgproc.boundingRect(theContours.get(i));

                    double rectWidth = r.width;
                    //find the largest width
                    if (rectWidth > maxRectangleWidth){
                        maxRectangleWidth = rectWidth;
                        maxRectX = r.x;
                        maxRectY = r.y;
                    }

                }
                maxRectangleCenter = maxRectangleWidth / 2;
                detectedOffset = maxRectX + maxRectangleCenter - 160 ; //top left anchor point

            } else {
                detectedOffset = -1000;
            }
            setSpikeLocation(detectedOffset);

            if (addDrawings) {
                Imgproc.drawContours(input, theContours, -1, new Scalar(0, 255, 0), 1);

                //draw a red line for the offset of the largest rectangle
                Imgproc.line(input, new Point(maxRectX + maxRectangleCenter, 0), new Point(maxRectX + maxRectangleCenter, 240), new Scalar(255, 0, 0));

            }
        }

        //find ratio to determine cone height


        /**
         * Add some nice and informative telemetry messages
         */
        telemetry.addData("[>]", "Change these values in tuner menu");
        telemetry.addData("[Color Space]", colorSpace.name());
        telemetry.addData("numRedCones", numRedCones);
        telemetry.addData("numBlueCones", numBlueCones);
        telemetry.addData("detectedOffset", detectedOffset);
        telemetry.addData("spikeLocation", spikeLocation);
        telemetry.update();

        return input;
    }

    void setSpikeLocation(double offset){
        //determine spike location
        int spikeLoc = -1;

        //1 = left, 2 = middle, 3 = right, -1 = unknown
        if (offset == -1000 ){
            spikeLoc = -1;
        } else if (offset < -90){
            spikeLoc = 1;
        } else if (offset > 90){
            spikeLoc = 3;
        } else {
            spikeLoc = 2;
        }
        spikeLocation.set(spikeLoc);
    }

    void drawBlueRectangle(Mat theMat, Rect rect){
        if(true) {
            Imgproc.rectangle( theMat, rect, new Scalar(0, 0, 255), 3);
        }
    }

    void draw_label(Mat img, String text, int x, int y, Scalar color) {
        if (true) {
            int font_face = Imgproc.FONT_HERSHEY_SIMPLEX;
            double scale = 1.25;

            Imgproc.putText(img, text, new Point(x, y), font_face, scale, color, 2, Imgproc.LINE_AA);
        }
    }


}

