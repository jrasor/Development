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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single
 * robot. In this case that robot is a Pullbot, a front wheel drive version of
 * the Pushbot in the external samples.
 * <p>
 * The Pullbot class assumes the following device names have been configured
 * on the robot:
 * Note:  All names are lower case.
 * <p>
 * Motor channel:  Left front  drive motor:        "motor0"
 * Motor channel:  Right front drive motor:        "motor1"
 * Motor channel:  Flexes the arm elbow:           "arm"
 * Color sensor:                                   "colorSensor"
 *
 * Before INITializing, manually move the arm to its initial position: back
 * over the robot, pointing down at 45°.
 */

/* Version history
 * ======= =======
 * v 0.1   Late Sept. 2020. Initial conversion of https://github
 * .com/FIRST-Tech-Challenge/FtcRobotController.
 * v.0.2   Early Oct. 2020. Pullbot robot class.
 * v 0.21  11/3/20  Vuforia-assisted approach to Blue Tower Goal.
 * v 1.0    11/14/20 Competition ready. Let's see if inheritance from
 *          GenericFTCRobot is correctly done. 11/16 It's not.
 * v 2.0  11/23/20 and later: Rebuilt from FtcRobotController, external
 *        samples, and pieces of failed FtcRobotController60.
 * v 2.1  11/28/20: added nudge methods and stick tempering.
 * v 2.2  12/1/20: improved nudging and simple driving.
 *        12/2/20 Feature freeze for UGScrimmage2.
 * v 3.0  12/2/20 Initial commit, a copy of UGScrimmage2.
 * v 3.1  12/9/20 Sigmoid motion profiling added.
 * v 3.2  12/16/20 Improved PID approach to Blue Tower Goal with Vuforia.
 * v 3.3beta  12/26/20 Production candidate for Scrimmage 3, and cleanup of
 *            the 12/24 Pullbot mess.
 * v 3.3  1/4/21 Simplified Ring shape detection.
 * v 3.4  1/26/21 Servo arm replaced with heavy, gear driven motor arm. It
 *        has a passive Wobble Goal grabber, capable of scoring it into the Drop
 *        Zone.
 */

public class Pullbot extends GenericFTCRobot {

  // Vision properties
  public Navigator navigator = new Navigator();
  public OpenCvInternalCamera2 phoneCam;
  public RingOrientationAnalysisPipeline ringPipeline;
  // Where the camera lens with respect to the robot.
  // On this robot class, it is centered (left to right), over the drive
  // wheel axis, 6" behind the front of the Pullbot, 6.5 Inches above ground.
  public static final float CAMERA_VERTICAL_DISPLACEMENT =
      6.5f * GenericFTCRobot.mmPerInch;
  public static final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera
  // is ON the robot's center line
  public static boolean PHONE_IS_PORTRAIT = false;
  public int cameraMonitorViewId;
  public static final int tooFarRight = 100;
  public static final int tooHigh = 140;
  public static final int tooWide = 70;
  public static final int tooTall = 60;

  // Field related constants.
  // Constants for perimeter Vuforia navigation targets
  // Field outside: 12'. Inside: 1" shorter than that, each Wall.
  public static final float fullField = 142 * GenericFTCRobot.mmPerInch;
  public static final float halfField = fullField / 2;
  public static final float quarterField = fullField / 4;
  // the height of the center of the target image above the floor
  public static final float mmTargetHeight = (6) * GenericFTCRobot.mmPerInch;

  /* Drive train. */
  // Drive train related constants in inches.
  // These next two are adjusted by calibration.
  public static final double DRIVE_WHEEL_SEPARATION = 13.0;
  public static final double DRIVE_WHEEL_DIAMETER = 3.5;
  // Fixed by drive train design. < 1.0: geared UP. > 1.0: geared DOWN.
  public static final double DRIVE_GEAR_REDUCTION = 1.0;
  public static final double COUNTS_PER_MOTOR_TURN = 1120; // REV HD hex 40:1
  static final double COUNTS_PER_INCH =
      (COUNTS_PER_MOTOR_TURN * DRIVE_GEAR_REDUCTION) /
          (DRIVE_WHEEL_DIAMETER * Math.PI);
  static final double DISTANCE_PER_TURN = DRIVE_WHEEL_DIAMETER * Math.PI;
  /// 11.00" per second.
  // NeveRest40 free run: 160 rpm. Go about 80% of that, so encoders work at
  // high speed under load.
  static final double MAX_MOTOR_RPM = 129;
  static final double MAX_WHEEL_TURNS_PER_SECOND = MAX_MOTOR_RPM / 60; //2.15
  static final double MAX_DRIVE_SPEED =
      MAX_WHEEL_TURNS_PER_SECOND * DISTANCE_PER_TURN; // 23.64"/s
  public DcMotorEx leftDrive = null;
  public DcMotorEx rightDrive = null;

  // Arm related properties
  public final double ARMSPEED = 0.5;
  public final int DEPLOYED = 1917;   // arm extended in front of the Pullbot
  public final int OVER_WALL = 1450;
  public final int STOWED = 0;     // arm retracted back over the Pullbot

  public DcMotorEx arm = null;

  // Pullbot specific sensor members.
  public ColorSensor colorSensor;
  public static VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

  // Initialization.
  HardwareMap hwMap = null;
  private LinearOpMode currentOpMode;
  private ElapsedTime period = new ElapsedTime();

  /* Constructors */
  public Pullbot() {
    super();
  }
  public Pullbot(LinearOpMode linearOpMode) {
    currentOpMode = linearOpMode;
  }

  public String init(HardwareMap someHWMap) {
    hwMap = someHWMap;
    String initializationReport = "Pullbot initialization: ";
    // Initialize vision hardware.
    colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

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
        ringPipeline = new RingOrientationAnalysisPipeline();
        phoneCam.setPipeline(ringPipeline);
      }
    });

    // Define and initialize motors. Stop them.
    leftDrive = hwMap.get(DcMotorEx.class, "motor0");
    rightDrive = hwMap.get(DcMotorEx.class, "motor1");
    leftDrive.setDirection(DcMotor.Direction.FORWARD);
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
    leftDrive.setPower(0);
    rightDrive.setPower(0);
    arm = hwMap.get(DcMotorEx.class, "arm");
    arm.setDirection(DcMotorSimple.Direction.FORWARD);
    // Manually move arm to STOWED position, back over robot at 45°.
    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    return initializationReport;
  }

  /*
   *										Vision methods
   */

  static class RingOrientationAnalysisPipeline extends OpenCvPipeline {
    //    Colors used to draw bounding rectangles.
    //    Todo: do this for Blue Wobble Goal mast. A later Pullbot may be
    //    able to look for one and grab it.
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    //    Threshold is how loosely or tightly to accept colors.
    static final int CB_CHAN_MASK_THRESHOLD = 110;
    //    Marking rectangles or other detected shapes.
    static final int CONTOUR_LINE_THICKNESS = 2;
    static final int CB_CHAN_IDX = 2;
    //    Buffers (matrices) hold image pixels for processing.
    Mat cbMat = new Mat();  // A new buffer.
    Mat thresholdMat = new Mat(); // Accepted or rejected pixels.
    Mat morphedThreshold = new Mat(); // Buffer with smoothed edges.
    //   Buffers used in the smoothing process.
    Mat erodedElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        new Size(3, 3));
    Mat dilatedElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
        new Size(6, 6));
    // Detected shapes pasted onto the processed image.
    Mat contoursOnPlainImageMat = new Mat();

    // Lists of Ring candidates.
    ArrayList<AnalyzedRing> internalRingList = new ArrayList<>();
    volatile ArrayList<AnalyzedRing> clientRingList = new ArrayList<>();
    RingStage[] stages = RingStage.values();
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
      internalRingList.clear();

      //   Look for edges separating accepted from rejected pixels.
      for (MatOfPoint contour : findRingContours(input)) {
        analyzeContour(contour, input);
      }
      clientRingList = new ArrayList<>(internalRingList);
      return input;
    }

    public ArrayList<AnalyzedRing> getDetectedRings() {
      return clientRingList;
    }

    ArrayList<MatOfPoint> findRingContours(Mat input) {
      // Set up the acceptable Ring colors.
      ArrayList<MatOfPoint> ringContoursList = new ArrayList<>();

      // Convert the input image to YCrCb color space, then extract the Cb
      // channel. Cb looks for blue, no matter the lighting.
      Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(cbMat, cbMat, 2);

      // Threshold the Cb channel to form a mask and invert it.
      Imgproc.threshold(cbMat, thresholdMat, CB_CHAN_MASK_THRESHOLD, 255,
          Imgproc.THRESH_BINARY_INV); // inverted blue is yellow.

      // Smooth the mask edges.
      morphMask(thresholdMat, morphedThreshold);

      // Look for the contours enclosing acceptable Ring colors.
      Imgproc.findContours(morphedThreshold, ringContoursList, new Mat(),
          Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

      // Draw edges for the contours we find, but not to the main input buffer.
      input.copyTo(contoursOnPlainImageMat);
      Imgproc.drawContours(contoursOnPlainImageMat, ringContoursList, -1,
          BLUE, CONTOUR_LINE_THICKNESS, 8);

      return ringContoursList;
    }

    void morphMask(Mat input, Mat output) {
      //   Noise reduction. Take off some of the raggedy border area, then
      //   puff it back out. That will smooth the border area.
      Imgproc.erode(input, output, erodedElement);
      Imgproc.erode(output, output, erodedElement);
      Imgproc.dilate(output, output, dilatedElement);
      Imgproc.dilate(output, output, dilatedElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input) {
      AnalyzedRing analyzedRing = new AnalyzedRing();
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
      internalRingList.add(analyzedRing);
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
    enum RingStage {
      FINAL,
      Cb,
      MASK,
      MASK_NR,
      CONTOURS
    }

    static class AnalyzedRing {
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

    ArrayList<RingOrientationAnalysisPipeline.AnalyzedRing> rings =
        ringPipeline.getDetectedRings();
    // Todo: clean up the logic here.
    if (rings.isEmpty()) {
      // ringsDetected will be left at zero.
    } else {
      for (RingOrientationAnalysisPipeline.AnalyzedRing ring :
          rings) {
        if (ring.left > tooFarRight) continue; // reject this "Ring".
        if (ring.top < tooHigh) continue;
        if (ring.width > tooWide) continue;
        if (ring.height > tooTall) continue;
        // This rectangle is in the correct position. How many Rings are
        // stacked in it?
        if (ring.aspectRatio > 1 && ring.aspectRatio <= 2) ringsDetected = 4;
        if (ring.aspectRatio > 2 && ring.aspectRatio <= 4) ringsDetected = 1;
      }
    }

    return ringsDetected;
  }

  /*
   *										Drive Train methods
   */
  private double NUDGE_SPEED = 0.20;

  /*                      Primitive layer.                    */
  // Task layer methods are built up out of members at this layer.
  private double temperedControl(double input) {
    return Math.pow(input, 3.0);
  }

  public void setDriveRunMode(DcMotor.RunMode someRunMode) {
    leftDrive.setMode(someRunMode);
    rightDrive.setMode(someRunMode);
  }

  // Set both drive motors to some behavior when they're told to stop.
  public void setDriveStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
    leftDrive.setZeroPowerBehavior(someBehavior);
    rightDrive.setZeroPowerBehavior(someBehavior);
  }

  public void moveMotor(DcMotor motor, double speed, double inches) {
    int newTarget;

    // Determine new target position, and pass it to motor controller.
    // Negative target because Pullbot motors pull, not push.
    newTarget = (int) (-inches * COUNTS_PER_INCH);
    motor.setTargetPosition(newTarget);
    setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Go!
    motor.setPower(Math.abs(speed));
    while (motor.isBusy()) {
      // Wait until motor is done before doing anything else.
    }

    // Clean up, prepare for next segment.
    motor.setPower(0);
  }

  public void encoderDrive(double leftSpeed, double rightSpeed,
                           double leftInches, double rightInches) {
    // Todo: can two calls to moveMotor work here?
    int newLeftTarget;
    int newRightTarget;

    //  Discard current motor encoder positions.
    setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Determine new target positions, and pass it to motor controller.
    // Negative targets because Pullbot motors pull, not push.
    newLeftTarget = (int) (-leftInches * COUNTS_PER_INCH);
    newRightTarget = (int) (-rightInches * COUNTS_PER_INCH);
    leftDrive.setTargetPosition(newLeftTarget);
    rightDrive.setTargetPosition(newRightTarget);

    setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Go!
    leftDrive.setPower(Math.abs(leftSpeed));
    rightDrive.setPower(Math.abs(rightSpeed));

    while (leftDrive.isBusy() && rightDrive.isBusy()) {
      // Wait until motors done before doing anything else.
    }

    // Clean up, prepare for next segment.
    leftDrive.setPower(0);
    rightDrive.setPower(0);
    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  ElapsedTime runtime = new ElapsedTime();

  /*    Sigmoid profile for change of some variable that this Pullbot can use
  . "Sigmoid" means a graph of the variable looks like the letter S. The
    Greek name for that letter is "sigma".

    It makes a variable v, like robot speed, follow a sigmoid speed vs time
    function over the interval (0, 1). In that interval, the variable smoothly
    and gradually increases from zero to 1:

    v (t) = 0.5 - 0.5 * cos (πt)

    Then it slows down to gently approach v = 0.

    The domain and time range (period) can be scaled. The initial value of v can
    be other than zero. Generalizing with all these possibilities:

    v (t) = Vo + Vscale * (0.5 - 0.5 * cos (πt/period))

    Down in the task layer are methods built up out of this primitive layer
    method. Example: turnArcRadiusSigmoid.
  */
  int changeSpeedSigmoid(double time2DoIt, double startSpeed, double endSpeed,
                         DcMotor someMotor) {
    int Counts = 0;
    double time;
    double power;
    double powerScale = endSpeed - startSpeed;
    do {
      time = runtime.time();
      power =
          startSpeed + powerScale * (0.5 - 0.5 * Math.cos(Math.PI * time / time2DoIt));
      someMotor.setPower(power);
    } while (time < time2DoIt);
    Counts = someMotor.getCurrentPosition();
    return Counts;
  }

  /*                         Task layer.                    */
  // These methods are built up out of primitive layer methods. Opmodes are
  // built up out of methods at this layer.

  //  This one requires no command layer to hardware layer translation.
  //  Just continue going straight.
  public void continueStraight(double speed) {
    leftDrive.setPower(speed);
    rightDrive.setPower(speed);
  }

  //   Simple wrapper for encoderDrive. Just go straight a number of inches.
  public void driveStraight(double speed, double inches) {
    encoderDrive(speed, speed, inches, inches);
  }

  //   Turn on axis, as though with left and right tank drive joysticks in
  //   equal but opposite deflection.
  public void turnAngle(double speed, double angle) { // angle in radians
    double inches = angle * DRIVE_WHEEL_SEPARATION / 2;
    encoderDrive(speed, speed, -inches, inches);
  }

  /*  Turning movements. All angles are in radians. */
  //  Turn at speed through an angle, with a given radius.
  public void turnAngleRadiusDrive(double speed, double angle,
                                   double radius) {

    // One or both turning arcs could be negative.
    // Degenerate cases: angle = 0, R = 0, R = d/2, R = +infinity (straight
    // drive).
    // Calculate 2 target distances, 2 speeds. Then feed 'em to encoderDrive.
    // Arc lengths are angle * radius adjusted for the drive wheels: one
    // shorter, the other longer. Speeds need to be adjusted as well.
    // TODO: handle 4 quadrant cases: forward CW, forward CCW, back CW,
    //  back CCW
    // ** Note negative angle enforces backward movement. What would
    // negative angle and negative radius do?
    double leftRadius = radius - DRIVE_WHEEL_SEPARATION / 2.0;
    double rightRadius = radius + DRIVE_WHEEL_SEPARATION / 2.0;
    double turnAdjustLeft = leftRadius / radius;
    double turnAdjustRight = rightRadius / radius;
    double leftArc = leftRadius * angle;
    double rightArc = rightRadius * angle;
    double leftSpeed = speed * turnAdjustLeft;
    double rightSpeed = speed * turnAdjustRight;

    encoderDrive(leftSpeed, rightSpeed, leftArc, rightArc);
  }

  //    Wrapper for turnAngleRadius
  // TODO make this a wrapper for encoderDrive instead.
  public void turnArcRadiusDrive(double speed, double arc, double radius) {
    double targetAngle = arc / radius;
    turnAngleRadiusDrive(speed, targetAngle, radius);
  }

  public double turnArcRadiusSigmoid(double startSpeed, double endSpeed,
                                     double arc, double radius) {
    double time;
    double speed;
    double leftSpeed, rightSpeed;
    double speedScale = endSpeed - startSpeed;
    double averageSpeed = (startSpeed + endSpeed) / 2.0;
    // Todo: What if radius is negative?
    double turnFudgeFactor =
        (radius + DRIVE_WHEEL_SEPARATION / 2.0) / radius;
    double time2DoIt = arc / (averageSpeed * MAX_DRIVE_SPEED);
    runtime.reset();
    do {
      time = runtime.time();
      speed =
          startSpeed + speedScale * (0.5 - 0.5 * Math.cos(Math.PI * time / time2DoIt));
      leftSpeed = -speed / turnFudgeFactor;
      rightSpeed = -speed * turnFudgeFactor;
      // Normalize speeds so greater is 1, and the lesser is scaled down by
      // the lesser/greater ratio.
      if (Math.abs(leftSpeed) > 1.0) {
        rightSpeed = rightSpeed / Math.abs(leftSpeed);
        leftSpeed = Math.signum(leftSpeed); // was -1.0
      }
      if (Math.abs(rightSpeed) > 1.0) {
        leftSpeed = leftSpeed / Math.abs(rightSpeed);
        rightSpeed = Math.signum(rightSpeed); // was -1.0
      }

      leftDrive.setPower(leftSpeed);
      rightDrive.setPower(rightSpeed);
    } while (time < time2DoIt);
    return time2DoIt;
  }

  // TurnAngleArc not implemented.

  //  Begin a left turn at speed, sharpness of turn decided by ratio.
  // TODO Test on Scrimmage3 build.
  //    1:  go straight.
  //    0:  turn axis is left wheel.
  //    -1: turn axis is between drive wheels. Robot turns on own axis.
  public void steerLeft(double speed, double ratio) {
    Range.clip(ratio, -1.0, 1.0);
    leftDrive.setPower(speed * ratio);
    rightDrive.setPower(speed);
  }

  //  Right analog of steerLeft.
  public void steerRight(double speed, double ratio) {
    Range.clip(ratio, -1.0, 1.0);
    leftDrive.setPower(speed);
    rightDrive.setPower(speed * ratio);
  }

  //  Drive a curved path by making left wheels turn slower and go
  //    shorter path by a factor of ratio. The right wheels will spin
  //    at parameter speed, and travel the full arc.
  public void turnLeft(double speed, double ratio, double arcInches) {
    Range.clip(ratio, -1.0, 1.0);
    encoderDrive(speed * ratio, speed,
        arcInches * ratio, arcInches);
  }

  //  Right analog of turnLeft.
  public void turnRight(double speed, double ratio, double arcInches) {
    Range.clip(ratio, -1.0, 1.0);
    encoderDrive(speed, speed * ratio,
        arcInches, arcInches * ratio);
  }


  /*                      Command layer.                    */
  // Human driver issues commands with gamepad.

  public void enableArm () {
    arm.setPower(0.0);
    while (currentOpMode.gamepad1.dpad_up) { // Away from Driver
      // Move arm towards Field.
      // Stops when fully extended, even if commanded to go more.
      if (arm.getCurrentPosition() >= DEPLOYED) {
        arm.setPower(0.0);
      } else {
        arm.setPower(ARMSPEED);
      }
    }
    while (currentOpMode.gamepad1.dpad_down) {
      // Pull arm back over the robot.
      // Stops when fully retracted, even if commanded to keep coming.
      if (arm.getCurrentPosition() <= STOWED) {
        arm.setPower(0.0);
      } else {
        arm.setPower(-ARMSPEED);
      }
    }
    while (currentOpMode.gamepad1.dpad_right) {
      // Nudge robot right on axis.
      leftDrive.setPower(-0.4*NUDGE_SPEED);
      rightDrive.setPower(0.4*NUDGE_SPEED);
    }
    while (currentOpMode.gamepad1.dpad_left) {
      // Nudge robot left on axis.
      leftDrive.setPower(0.4*NUDGE_SPEED);
      rightDrive.setPower(-0.4*NUDGE_SPEED);
    }
  }

  public void enableNudge() {

    // Gamepad mapping is similar to tank drive.
    if (currentOpMode.gamepad1.left_trigger > 0) {
      // nudge left wheel forward a little
      leftDrive.setPower(-NUDGE_SPEED);
    }
    if (currentOpMode.gamepad1.right_trigger > 0) {
      // nudge right wheel forward a little
      rightDrive.setPower(-NUDGE_SPEED);
    }
    if (currentOpMode.gamepad1.left_bumper) {
      // nudge left wheel back a little
      leftDrive.setPower(NUDGE_SPEED);
    }
    if (currentOpMode.gamepad1.right_bumper) {
      // nudge right wheel back a little
      rightDrive.setPower(NUDGE_SPEED);
    }
  }

  public void tankDrive() {
    //  Tank drive with the two sticks.
    double leftCommand = currentOpMode.gamepad1.left_stick_y;
    double rightCommand = currentOpMode.gamepad1.right_stick_y;
    leftDrive.setPower(Range.clip(temperedControl(leftCommand), -1.0, 1.0));
    rightDrive.setPower(Range.clip(temperedControl(rightCommand), -1.0, 1.0));
  }

  public void simpleDrive() {
    //  Left stick for fore-and-aft, right one for turns.
    double drive = currentOpMode.gamepad1.left_stick_y;
    double turn = currentOpMode.gamepad1.right_stick_x;
    double driveCommand = temperedControl(drive);
    driveCommand = Range.clip(driveCommand, -1.0, 1.0);
    double turnCommand = temperedControl(turn);
    turnCommand = Range.clip(turnCommand, -1.0, 1.0);
    // Might have to wait until here to clip, maybe normalize.
    leftDrive.setPower(temperedControl(drive - turn)); // turn/2.0?
    rightDrive.setPower(temperedControl(drive + turn));
  }

  public void oneStickDrive() {
    double sumPower = 0.0;
    double diffPower = 0.0;
    double leftMotorPower = 0.0;
    double rightMotorPower = 0.0;
    double maxPower = 0.0;
    final double MAX_POWER = 1.0;

    // calculate sum and difference of motor powers.
    sumPower = currentOpMode.gamepad1.left_stick_y; // forward stick is
    // negative; we want power forward.
    diffPower = -currentOpMode.gamepad1.left_stick_x; // left-right stick
    // steers by running motors differently.

    // calculate motor power as a linear combination of sum and difference.
    leftMotorPower = (sumPower + diffPower) / 2;
    rightMotorPower = (sumPower - diffPower) / 2;

    // clip illegal power levels.
    maxPower = Math.max(Math.abs(leftMotorPower), Math.abs(rightMotorPower));
    maxPower = Math.min(maxPower, MAX_POWER);

    // eg: Run wheels in tank mode (note: The joystick goes negative when
    // pushed forwards)
    leftDrive.setPower(leftMotorPower);
    rightDrive.setPower(rightMotorPower);
  }

  // Go to a specified location of the Field, arriving at a specified heading.
  // It needs the current robot location updated in real time by Vuforia.
  public void Drive2XYHeading (double targetX, double targetY,
                               double targetHeading) {

    OpenGLMatrix lastLocation = null;
    //private VuforiaLocalizer vuforia = null;
    boolean targetVisible = false;
    float phoneXRotate = 0;
    float phoneYRotate = 0;
    float phoneZRotate = 0;

    double yCorrection = -0.04;
    double headingCorrection = 0.5;
    final double STRAIGHT_SPEED = 0.6;
    final double TURN_SPEED = 0.2;
    final double MAX_CORRECTION = TURN_SPEED;
    final double MIN_CORRECTION = -TURN_SPEED;
    targetX = 74.0; // Aim for robot front to end up near the picture.
    double currentX = 0;  // We'll refine this by Vuforia if target image is
    // visible.
    double errorX = currentX - targetX;
    final double ERROR_X_TOLERANCE = 16.0;
    // avoids large and unstable bearing changes on final approach.
    targetY = 35.5; // Also so robot can be near the picture.
    double currentY;
    double errorY;
    final double ERROR_Y_TOLERANCE = 1.0;
    double targetHeadingDegrees = 0.0;
    double targetHeadingRadians = targetHeadingDegrees * Math.PI / 180.0;
    double currentHeadingDegrees;
    double currentHeadingRadians;
    double errorHeadingDegrees;
    double errorHeadingRadians = 0.0;

    double targetBearingRadians = 0.0;
    double targetBearingDegrees = targetBearingRadians * 180 / Math.PI;
    double currentBearingRadians;
    double errorBearingRadians;
    double currentBearingDegrees;
    double errorBearingDegrees;

    double correction = 0.0;

      targetVisible = false;
      for (VuforiaTrackable trackable : navigator.allTrackables) {
        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
          targetVisible = true;
          OpenGLMatrix robotLocationTransform =
              ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
          if (robotLocationTransform != null) {
            lastLocation = robotLocationTransform;
          }
          break;
        }
      }

      // Report robot location and heading (if we know).
      if (targetVisible) {
        VectorF translation = lastLocation.getTranslation();
        Orientation rotation =
            Orientation.getOrientation(lastLocation, EXTRINSIC,
                XYZ, DEGREES);
      } else {
      }

      // Report where the robot is located, if we can see a Vuforia image.
      if (targetVisible && Math.abs(errorX) > ERROR_X_TOLERANCE) {

        // Report position (translation) and position error of robot in inches.
        VectorF translation = lastLocation.getTranslation();
        currentX = translation.get(0) / GenericFTCRobot.mmPerInch;
        currentY = translation.get(1) / GenericFTCRobot.mmPerInch;
        errorX = currentX - targetX;
        errorY = currentY - targetY;

        // Report bearing and bearing error of target from robot.
        currentBearingRadians = Math.atan2(-errorY, -errorX);
        currentBearingDegrees = currentBearingRadians * 180.0 / Math.PI;
        errorBearingRadians = currentBearingRadians - targetBearingRadians;
        errorBearingDegrees = errorBearingRadians * 180.0 / Math.PI;

        // Report robot heading in degrees, and error of that heading.
        Orientation rotation =
            Orientation.getOrientation(lastLocation, EXTRINSIC,
                XYZ,
                DEGREES);
        currentHeadingDegrees = rotation.thirdAngle;
        currentHeadingRadians = currentHeadingDegrees * Math.PI / 180.0;
        errorHeadingDegrees = currentHeadingDegrees - targetHeadingDegrees;
        errorBearingRadians = currentBearingRadians - targetBearingRadians;
        errorHeadingRadians = errorHeadingDegrees * Math.PI / 180.0;

        // find motor speed corrections.
        correction =
            yCorrection * errorY - headingCorrection * errorHeadingRadians;
        correction = Math.min(Math.max(correction, MIN_CORRECTION),
            MAX_CORRECTION);
        // Todo: slow down when errorX gets small.

        //  Apply those corrections to drive motors.
        // This is a Pullbot, not a Pushbot.
        leftDrive.setPower(-TURN_SPEED + correction);
        rightDrive.setPower(-TURN_SPEED - correction);

      } else if (!targetVisible) {
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        // Todo: try to recover from this by turning on axis, guessing
        //  from last known position.
      } else {

        // Clean up residual heading error.
        turnAngle(TURN_SPEED, -errorHeadingRadians);
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        currentOpMode.stop();
      }

  }

  // Like the allomorph above, but gets initial position as arguments. It
  // will try to update it with the IMU until it gets firm data from Vuforia.
  // Then it will use that as long as available. It will return to dead
  // reckoning on loss of Vuforia data.
  public void Drive2XYHeading (double targetX, double targetY,
                               double targetHeading, double currentX,
                               double currentY, double currentHeading) {

  }
  // Macros can go here. Most will be used in the opmodes.
}
