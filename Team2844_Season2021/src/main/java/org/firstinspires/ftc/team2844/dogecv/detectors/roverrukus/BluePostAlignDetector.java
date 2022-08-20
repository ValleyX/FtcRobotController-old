package org.firstinspires.ftc.team2844.dogecv.detectors.roverrukus;

import org.firstinspires.ftc.team2844.dogecv.DogeCV;
import org.firstinspires.ftc.team2844.dogecv.detectors.DogeCVDetector;
import org.firstinspires.ftc.team2844.dogecv.filters.LeviColorFilter;
import org.firstinspires.ftc.team2844.dogecv.scoring.MaxAreaScorer;
import org.firstinspires.ftc.team2844.dogecv.scoring.PerfectAreaScorer;
import org.firstinspires.ftc.team2844.dogecv.scoring.RatioScorer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Victo on 9/17/2018.
 */

public class BluePostAlignDetector extends DogeCVDetector {

    // Defining Mats to be used.
    private Mat displayMat = new Mat(); // Display debug info to the screen (this is what is returned)
    private Mat workingMat = new Mat(); // Used for preprocessing and working with (blurring as an example)
    private Mat maskRed = new Mat(); // Yellow Mask returned by color filter
    private Mat hierarchy  = new Mat(); // hierarchy used by coutnours
    private Mat maskBlack  = new Mat(); //

    // Results of the detector
    private boolean found    = false; // Is the gold mineral found
    private boolean aligned  = false; // Is the gold mineral aligned
    private double  goldXPos = 0;     // X Position (in pixels) of the gold element
    private Point   screenPosition = new Point(); // Screen position of the mineral
    private Rect    foundRect = new Rect(); // Found rect

    // Detector settings
    public boolean debugAlignment = true; // Show debug lines to show alignment settings
    public double alignPosOffset  = 0;    // How far from center frame is aligned
    public double alignSize       = 400;  // How wide is the margin of error for alignment
    public int ksize = 5;
    //public int yellowTheshold = 100;
    public int blueTheshold = 145;
    public int blackThreshold = 255;
    public int minimumSize = 0;

    public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer
    //public DogeCV.AreaScoringMethod areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer


    //Create the default filters and scorers

   public LeviColorFilter redFilter      = new LeviColorFilter(LeviColorFilter.ColorPreset.BLUE, blueTheshold); //Default Yellow filter `100
   //  public LeviColorFilter redFilter      = new LeviColorFilter(LeviColorFilter.ColorPreset.RED); //Default Yellow filter `100
    //public LeviColorFilter blackFilter        = new LeviColorFilter(LeviColorFilter.ColorPreset.BLACK ,blackThreshold ); //
    //public DogeCVColorFilter blackFilter = new GrayscaleFilter(0, 5);


    public RatioScorer       ratioScorer       = new RatioScorer(1.0, 1.0);          // Used to find perfect squares 1, 3
    public MaxAreaScorer     maxAreaScorer     = new MaxAreaScorer( 1.0);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer( 5000,0.5); // Used to find objects near a tuned area value

    /**
     * Simple constructor
     */
    public BluePostAlignDetector() {
        super();
        detectorName = "Blue Post Align Detector"; // Set the detector name
    }


    @Override
    public Mat process(Mat input) {

        // Copy the input mat to our working mats, then release it for memory
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        //input.release();

        //Preprocess the working Mat (blur it then apply a yellow filter)
        //Imgproc.GaussianBlur(workingMat, workingMat, new Size(5, 5), 0);
      //  Imgproc.GaussianBlur(workingMat, workingMat, new Size(5, 5), 0);
        Imgproc.GaussianBlur(workingMat, workingMat, new Size(ksize, ksize), 0);
        redFilter.process(workingMat.clone(), maskRed);

      //  blackFilter.process(workingMat.clone(), maskBlack);


        //Find contours of the yellow mask and draw them to the display mat for viewing

        List<MatOfPoint> contoursRed = new ArrayList<>();
        //List<MatOfPoint> contoursBlack = new ArrayList<>();
       // Imgproc.findContours(maskYellow, contoursRed, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskRed, contoursRed, hierarchy, Imgproc.CHAIN_APPROX_SIMPLE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursRed, -1, new Scalar(230, 70, 70), 2);
/*
        //Imgproc.rectangle(blackMask, bestRect.tl(), bestRect.br(), new Scalar(255,255,255), 1, Imgproc.LINE_4, 0);
        blackFilter.process(workingMat.clone(), maskBlack);
        List<MatOfPoint> contoursBlack = new ArrayList<>();
        Imgproc.findContours(maskBlack, contoursBlack, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursBlack,-1,new Scalar(40,40,255),2);
*/

        // Current result
        Rect bestRect = null;
        double bestDiffrence = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better
        int bestY = 480;
        int bestx = 640;
        //return input;
        // Loop through the contours and score them, searching for the best result
        for (MatOfPoint cont : contoursRed) {
            Rect rect = Imgproc.boundingRect(cont);
            if (rect.size().width > minimumSize) {
                double score = calculateScore(cont); // Get the diffrence score using the scoring API
                //score = 1.1;
                // Get bounding rect of contour
                // Rect rect = Imgproc.boundingRect(cont);
                Imgproc.rectangle(displayMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect
                //Imgproc.rectangle(workingMat, rect.tl(), rect.br(), new Scalar(0, 0, 255), 2); // Draw rect
                Imgproc.putText(displayMat, "s " + score, rect.tl(), 0, 1, new Scalar(255, 255, 255));

              //  if (/*(rect.y < bestY) && */ (rect.y > 240) && (rect.x > 150) && (rect.x < bestx - 150))
                if (rect.y < bestY)
                {
                    bestY = rect.y;
                    bestDiffrence = score;
                    bestRect = rect;
                }
                /*
                // If the result is better then the previously tracked one, set this rect as the new best
                if (score < bestDiffrence && rect.y + rect.height / 2 > ((isSideways ? getAdjustedSize().width : getAdjustedSize().height) * (1 - verticalMax)) && rect.y + rect.height / 2 < ((isSideways ? getAdjustedSize().width : getAdjustedSize().height) * (1 - verticalMin))) {
                    bestDiffrence = score;
                    bestRect = rect;
                }

                 */
            }
        }

        // Vars to calculate the alignment logic.

        double alignX = (getAdjustedSize().width / 2) + alignPosOffset; // Center point in X Pixels
        if (isSideways)
            alignX = (getAdjustedSize().height / 2) + alignPosOffset; //Center point in rotated X pixels
        double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
        double alignXMax = alignX + (alignSize / 2); // Max X pos in pixels
        double xPos; // Current Gold X Pos

        if (bestRect != null) {
            if (true /*bestRect.size().width > minimumSize*/) {
                // Show chosen result
                Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255, 0, 0), 4);
                //for test
                //Imgproc.rectangle(workingMat, bestRect.tl(), bestRect.br(), new Scalar(255, 0, 0), 4);
                Imgproc.putText(displayMat, "Terminate", bestRect.tl(), 0, 1, new Scalar(255, 255, 255));

                // Set align X pos
                xPos = bestRect.x + (bestRect.width / 2);
                goldXPos = xPos;

                // Draw center point
                Imgproc.circle(displayMat, new Point(xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0, 255, 0), 2);

                // Check if the mineral is aligned
                if (xPos < alignXMax && xPos > alignXMin) {
                    aligned = true;
                } else {
                    aligned = false;
                }

                // Draw Current X
                if (isSideways) {
                    Imgproc.putText(displayMat, "Current X: " + bestRect.x, new Point(10, getAdjustedSize().width - 10), 0, 0.5, new Scalar(255, 255, 255), 1);
                } else {
                    Imgproc.putText(displayMat, "Current X: " + bestRect.x, new Point(10, getAdjustedSize().height - 10), 0, 0.5, new Scalar(255, 255, 255), 1);
                }
                found = true;
                foundRect = bestRect;
                screenPosition = new Point(bestRect.x, bestRect.y);
            }
        } else {
            found = false;
            aligned = false;
        }
        if (debugAlignment) {

            //Draw debug alignment info
            if (isFound()) {
                if (isSideways) {
                    Imgproc.line(displayMat, new Point(goldXPos, getAdjustedSize().width), new Point(goldXPos, getAdjustedSize().width - 30), new Scalar(255, 255, 0), 2);
                } else {
                    Imgproc.line(displayMat, new Point(goldXPos, getAdjustedSize().height), new Point(goldXPos, getAdjustedSize().height - 30), new Scalar(255, 255, 0), 2);
                }
            }

            if (isSideways) {
                Imgproc.line(displayMat, new Point(alignXMin, getAdjustedSize().width), new Point(alignXMin, getAdjustedSize().width - 40), new Scalar(0, 255, 0), 2);
                Imgproc.line(displayMat, new Point(alignXMax, getAdjustedSize().width), new Point(alignXMax, getAdjustedSize().width - 40), new Scalar(0, 255, 0), 2);
            } else {
                Imgproc.line(displayMat, new Point(alignXMin, getAdjustedSize().height), new Point(alignXMin, getAdjustedSize().height - 40), new Scalar(0, 255, 0), 2);
                Imgproc.line(displayMat, new Point(alignXMax, getAdjustedSize().height), new Point(alignXMax, getAdjustedSize().height - 40), new Scalar(0, 255, 0), 2);
            }
        }

        //Print result
        if (isSideways) {
            Imgproc.putText(displayMat, "Result: " + aligned, new Point(10, getAdjustedSize().width - 30), 0, 1, new Scalar(255, 255, 0), 1);
        } else {
            Imgproc.putText(displayMat, "Result: " + aligned, new Point(10, getAdjustedSize().height - 30), 0, 1, new Scalar(255, 255, 0), 1);
        }

        return displayMat;
        //return workingMat;

    }

    @Override
    public void useDefaults() {
        addScorer(ratioScorer);

        // Add diffrent scoreres depending on the selected mode
        if(areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA){
            addScorer(maxAreaScorer);
        }

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA){
            addScorer(perfectAreaScorer);
        }

    }

    /**
     * Set the alignment settings for GoldAlign
     * @param offset - How far from center frame (in pixels)
     * @param width - How wide the margin is (in pixels, on each side of offset)
     */
    public void setAlignSettings(int offset, int width){
        alignPosOffset = offset;
        alignSize = width;
    }

    /**
     * Returns the gold element's last position in screen pixels
     * @return position in screen pixels
     */
    public Point getScreenPosition(){
        return screenPosition;
    }

    /**
     * Returns the gold element's found rectangle
     * @return gold element rect
     */
    public Rect getFoundRect() {
        return foundRect;
    }


    /**
     * Returns if the gold element is aligned
     * @return if the gold element is alined
     */
    public boolean getAligned(){
        return aligned;
    }

    /**
     * Returns gold element last x-position
     * @return last x-position in screen pixels of gold element
     */
    public double getXPosition(){
        return goldXPos;
    }

    /**
     * Returns if a gold mineral is being tracked/detected
     * @return if a gold mineral is being tracked/detected
     */
    public boolean isFound() {
        return found;
    }
}
