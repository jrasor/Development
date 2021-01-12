/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification,
 * are permitted (subject to the limitations in the disclaimer below)
 * provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 *  endorse or
 * promote products derived from this software without specific prior written
 *  permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 *  THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;


/**
 * This is NOT an opmode.
 * <p>
 * This class implements a colored rectangle detector. The user can supply a
 * range of colors to be accepted, and the class enables drawing rectangles
 * to fit those regions containing the accepted colors.
 *

/* Version history
 * ======= =======
 * v 0.1    1/6/20 Copied and reduced from Pullbot. Only OpenCV properties will
 *          be kept. Finds Ultimate Goal Rings on camera preview.
 * v 0.11   Finds blue rectangles.
 */

public class ColorBoxDetector extends GenericFTCRobot {

  // Vision properties
  public OpenCvInternalCamera2 phoneCam;
  public BoxOrientationAnalysisPipeline boxPipeline;
  // Where the camera lens with respect to the robot.
  // On this robot class, it is centered (left to right), over the drive
  // wheel axis.
  public static final float CAMERA_FORWARD_DISPLACEMENT =
      4.0f * GenericFTCRobot.mmPerInch;   //
  // eg:
  // Camera is 4 Inches in front of robot center
  public static final float CAMERA_VERTICAL_DISPLACEMENT =
      8.0f * GenericFTCRobot.mmPerInch;   // eg:
  // Camera is 8 Inches above ground
  public static final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera
  // is ON the robot's center line
  public static boolean PHONE_IS_PORTRAIT = false;
  public int cameraMonitorViewId;

  // Field related constants.
  // Constants for perimeter Vuforia navigation targets
  // Field outside: 12'. Inside: 1" shorter than that, each Wall.
  public static final float fullField = 142 * GenericFTCRobot.mmPerInch;
  public static final float halfField = fullField / 2;
  public static final float quarterField = fullField / 4;
  // the height of the center of the target image above the floor
  public static final float mmTargetHeight = (6) * GenericFTCRobot.mmPerInch;


  // Initialization.
  HardwareMap hwMap = null;
  private LinearOpMode currentOpMode;
  private ElapsedTime period = new ElapsedTime();

  /* Constructors */
  public ColorBoxDetector() {
    super();
  }

  public ColorBoxDetector(LinearOpMode linearOpMode) {
    currentOpMode = linearOpMode;
  }

  public String init(HardwareMap someHWMap) {
    hwMap = someHWMap;
    String initializationReport = "Pullbot initialization: ";
    // Initialize vision hardware.

    // Create camera instance
    cameraMonitorViewId =
        someHWMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id",
            someHWMap.appContext.getPackageName());
    initializationReport += "Camera Id: " + cameraMonitorViewId + ". ";
    phoneCam =
        OpenCvCameraFactory.getInstance().createInternalCamera2
            (OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

    // Open async and start streaming inside opened callback
    phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        boxPipeline = new BoxOrientationAnalysisPipeline();
        phoneCam.setPipeline(boxPipeline);
      }
    });

    return initializationReport;
  }

  /*
   *										Vision methods
   */

  static class BoxOrientationAnalysisPipeline extends OpenCvPipeline {
    //    Colors used to draw bounding rectangles.
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final int CB_CHAN_MASK_THRESHOLD = 55;
    //    Marking rectangles or other detected shapes.
    static final int CONTOUR_LINE_THICKNESS = 2;
    //    Buffers (matrices) hold image pixels for processing.
    Mat coloredMat = new Mat();  // A new buffer.
    Mat judgedMat = new Mat(); // Accepted or rejected pixels.
    Mat morphedThreshold = new Mat(); // Buffer with smoothed edges.
    //   Buffers used in the smoothing process.
    Mat erodedElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        new Size(3, 3));
    Mat dilatedElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        new Size(6, 6));
    // Detected shapes pasted onto the processed image.
    Mat contoursOnPlainImageMat = new Mat();

    // Lists of rectangle candidates.
    ArrayList<AnalyzedBox> internalBoxList = new ArrayList<>();
    volatile ArrayList<AnalyzedBox> clientBoxList = new ArrayList<>();
    BoxStage[] stages = BoxStage.values();
    //   Currently displayed processing stage.
    int stageNum = 0;

    static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
      //   Draws a rotated rect by drawing each of the 4 lines individually.
      Point[] points = new Point[4]; // corners.
      rect.points(points);
      for (int i = 0; i < 4; ++i) {
        Imgproc.line(drawOn, points[i], points[(i + 1) % 4], GREEN, 2);
      }
    }

    @Override // Overrides method of OpenCVPipeline
    public Mat processFrame(Mat input) {
      // Look for shapes in the image buffer.
      internalBoxList.clear();

      //   Look for edges separating accepted from rejected pixels.
      for (MatOfPoint contour : findBoxContours(input)) {
        analyzeContour(contour, input);
      }
      clientBoxList = new ArrayList<>(internalBoxList);
      return input;
    }

    public ArrayList<AnalyzedBox> getDetectedBoxes() {
      return clientBoxList;
    }

    ArrayList<MatOfPoint> findBoxContours(Mat input) {
      final Scalar YCrCbLOWER_BOUND = new Scalar (12, 0, 96);
      final Scalar YCrCbUPPER_BOUND = new Scalar (96, 128, 225);
      final Scalar RGB_LOWER_BOUND = new Scalar (0, 0, 64);
      final Scalar RGB_UPPER_BOUND = new Scalar (128, 128, 255);
      final Scalar HSV_LOWER_BOUND = new Scalar (220, 1.000, 0.251);
      final Scalar HSV_UPPER_BOUND = new Scalar (250, 0.498, 1.000);
      // Set up the acceptable rectangle colors.
      ArrayList<MatOfPoint> boxContoursList = new ArrayList<>();

      /* YCbCr color space. */
      Imgproc.cvtColor(input, coloredMat, Imgproc.COLOR_RGB2YCrCb);
      Core.inRange(coloredMat, YCrCbLOWER_BOUND, YCrCbUPPER_BOUND, judgedMat);

      /* RGB color space. */
      //coloredMat = input;
      //Core.inRange(coloredMat, RGB_LOWER_BOUND, RGB_UPPER_BOUND, judgedMat);

      /* HSV color space. */
      //Imgproc.cvtColor (input, coloredMat, Imgproc.COLOR_RGB2HSV);
      //Core.inRange(coloredMat, HSV_LOWER_BOUND, HSV_UPPER_BOUND, judgedMat);

      // Smooth the mask edges.
      morphMask(judgedMat, judgedMat);
      // Look for the contours enclosing acceptable Ring colors.
      Imgproc.findContours(judgedMat, boxContoursList, new Mat(),
          Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
      // Draw edges for the contours we find, but not to the main input buffer.
      input.copyTo(contoursOnPlainImageMat);
      Imgproc.drawContours(contoursOnPlainImageMat, boxContoursList, -1,
          BLUE, CONTOUR_LINE_THICKNESS, 8);

      return boxContoursList;
    }

    void morphMask(Mat input, Mat output) {
      //   Noise reduction. Take off some of the raggedy border area, then
      //   puff it back out. That will smooth the border area.
      Imgproc.erode(input, output, erodedElement);
      Imgproc.erode(output, output, erodedElement);
      Imgproc.dilate(output, output, dilatedElement);
      //Imgproc.dilate(output, output, dilatedElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input) {
      AnalyzedBox analyzedRing = new AnalyzedBox();
      //   Transform the contour to a different format.
      Point[] points = contour.toArray();
      MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

      //   Draw a rectangle that best fits the  contour.
      RotatedRect rotatedRectFitToContour =
          Imgproc.minAreaRect(contour2f);
      drawRotatedRect(rotatedRectFitToContour, input);
      analyzedRing.width = (int) rotatedRectFitToContour.size.width;
      analyzedRing.aspectRatio = rotatedRectFitToContour.size.width /
          rotatedRectFitToContour.size.height;
      analyzedRing.top = rotatedRectFitToContour.boundingRect().y;
      analyzedRing.left = rotatedRectFitToContour.boundingRect().x;
      analyzedRing.height = rotatedRectFitToContour.boundingRect().height;
      internalBoxList.add(analyzedRing);
      // The angle OpenCV gives us can be ambiguous, so look at the shape of
      // the rectangle to fix that. This angle is not used for Rings, but
      // will be used for Wobblers.
      double rotRectAngle = rotatedRectFitToContour.angle;
      if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
        rotRectAngle += 90;
      }
    }

    //   Pipeline processing stages. Different image buffers are available at
    //   each one.
    enum BoxStage {
      FINAL,
      Cb,
      MASK,
      MASK_NR,
      CONTOURS
    }

    static class AnalyzedBox {
      double aspectRatio;
      int top;
      int left;
      int height;
      int width;
    }
  }

  //    Use the detected rectangles to count Rings. Taller means more Rings.
  int CountRings(int viewID) {
    int ringsDetected = 0;

    ArrayList<BoxOrientationAnalysisPipeline.AnalyzedBox> rings =
        boxPipeline.getDetectedBoxes();
    // Todo: clean up the logic here.
    if (rings.isEmpty()) {
      // ringsDetected will be left at zero.
    } else {
      for (BoxOrientationAnalysisPipeline.AnalyzedBox ring :
          rings) {
        // This rectangle is in the correct position. How many Rings are
        // stacked in it?
        if (ring.aspectRatio > 1 && ring.aspectRatio <= 2) ringsDetected = 4;
        if (ring.aspectRatio > 2 && ring.aspectRatio <= 4) ringsDetected = 1;
      }
    }

    return ringsDetected;
  }

}
