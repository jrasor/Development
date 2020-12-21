package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


// TeleOp program that demonstrates PID aided straight drive.

// Y (yellow) commands a continuous straight drive of 48" length.
// A (green) Backwards 24".
//
//  Logitech gamepad buttons:
//  Bumper      Bumper
//  dpad
//           Y
//        X	    B
//           A
//

@TeleOp(name = "Run To Goal PID", group = "Exercises")
//@Disabled
public class Run2GoalPID extends LinearOpMode {
//  Pullbot robot = new Pullbot(this);
  DcMotor leftMotor, rightMotor;
  BNO055IMU imu;
  Orientation lastAngles = new Orientation();
  double globalAngle, power = .30, correction, rotation;
  boolean aButton, bButton, xButton, yButton;
  boolean dpad_Right, dpad_Forward, dpad_Left, dpad_Back;
  PIDController pidRotate, pidDrive, pidDrive2Goal;
  double DRIVE_WHEEL_SEPARATION = 13.0;
  double DRIVE_WHEEL_DIAMETER = 3.5;
  int COUNTS_PER_TURN = 1120;

  @Override
  public void runOpMode() throws InterruptedException {

    leftMotor = hardwareMap.dcMotor.get("motor0");
    rightMotor = hardwareMap.dcMotor.get("motor1");

    leftMotor.setDirection(DcMotor.Direction.REVERSE);
    rightMotor.setDirection(DcMotor.Direction.FORWARD);

    leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // called when init button is  pressed.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;

    // Retrieve and initialize the IMU. We expect the IMU to be built into
    // the robot's REV Hub, or attached to an I2C port on a Core Device
    // Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    imu.initialize(parameters);

    // Set PID proportional value Kp to start reducing power at about 50 degrees
    // of rotation. P by itself may stall before turn completed so we add a
    // bit of I (integral) which causes the PID controller to gently increase
    // power if the turn is not completed.
    pidRotate = new PIDController(.003, .00003, 0);

    // Set PID proportional value to produce non-zero correction value when
    // robot veers off a straight line. P value controls how sensitive the
    // correction is.
    pidDrive = new PIDController(.05, 0, 0);

    // PID controlled drive to a target distance
    pidDrive2Goal = new PIDController(.05, 0, 0);

    telemetry.addData("Mode", "calibrating...");
    telemetry.update();

    // make sure the imu gyro is calibrated before continuing.
    while (!isStopRequested() && !imu.isGyroCalibrated()) {
      sleep(50);
      idle();
    }

    telemetry.addData("Mode", "waiting for start");
    telemetry.addData("imu calibration status",
            imu.getCalibrationStatus().toString());
    telemetry.update();

    waitForStart();

    telemetry.addData("Mode", "running");
    telemetry.update();

    // Set up parameters for driving in a straight line.
    pidDrive.setSetpoint(0);
    pidDrive.setOutputRange(0, power);
    pidDrive.setInputRange(-90, 90);
    power = 0.0;
    //pidDrive.disable();

    while (opModeIsActive()) {

      telemetry.addData("imu heading", "%.1f\u00B0", lastAngles.firstAngle);
      telemetry.addData("global heading", "%.1f\u00B0", globalAngle);
      telemetry.addData("turn rotation", "%.4f", rotation);
      telemetry.addData("correction", "%.4f", correction);
      telemetry.update();

      // Use PID to correct side to side wandering off line.
      correction = pidDrive.performPID(getAngle());
      // Apply that to the power levels.
      leftMotor.setPower(power - correction);
      rightMotor.setPower(power + correction);

      aButton = gamepad1.a;
      bButton = gamepad1.b;
      xButton = gamepad1.x;
      yButton = gamepad1.y;
      dpad_Right = gamepad1.dpad_right;
      dpad_Forward = gamepad1.dpad_up;
      dpad_Left = gamepad1.dpad_left;
      dpad_Back = gamepad1.dpad_down;

      if (aButton) {
        telemetry.addData("command", "A stop.");
        updateTelemetry(telemetry);
        power = 0.0;
        sleep(1000);
      }

      if (yButton) {
        // resume forward drive
        telemetry.addData("command", "Y resume forward drive.");
        updateTelemetry(telemetry);
        pidRotate.disable();
        pidDrive2Goal.disable();
        pidDrive.enable();
        power = 0.30;
      }

      if (dpad_Forward){
        // resume forward drive
        power = 0.30;
        telemetry.addData("command", "D pad forward: go 24 inches.");
        updateTelemetry(telemetry);
        sleep(1000);
        pidRotate.disable();
        pidDrive2Goal.enable();
        pidDrive.disable();
        drive2Goal (24.0, 0.30);
      }
    }
    // turn the motors off.
    stopMotors();
  }

  private void stopMotors() {           // turn the motors off.
    pidDrive.disable();
    pidDrive2Goal.disable();
    pidRotate.disable();
    rightMotor.setPower(0);
    leftMotor.setPower(0);
  }

  /**
   * Resets the cumulative angle tracking to zero.
   */
  private void resetAngle() {
    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX, AngleUnit.DEGREES);

    globalAngle = 0;
  }

  /**
   * Get current cumulative angle rotation from last reset.
   *
   * @return Angle in degrees. + = left, - = right from zero point.
   */
  private double getAngle() {
    // We experimentally determined the Z axis is the axis we want to use for
    // heading angle. We have to process the angle because the imu works in
    // euler angles so the Z axis is returned as 0 to +180 or 0 to -180
    // rolling back to -179 or +179 when rotation passes 180 degrees. We
    // detect this transition and track the total cumulative angle of rotation.

    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX, AngleUnit.DEGREES);

    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

    if (deltaAngle < -180)
      deltaAngle += 360;
    else if (deltaAngle > 180)
      deltaAngle -= 360;

    globalAngle += deltaAngle;

    lastAngles = angles;

    return globalAngle;
  }

  /**
   * Drive forward to a specified distance with PID control.
   *
   * @param inches distance to run
   */
  // Driving in a straight line to a target distance.
  private void drive2Goal(double inches, double power) {
    double targetCounts = inches / DRIVE_WHEEL_DIAMETER
            * COUNTS_PER_TURN;
    pidDrive2Goal.setSetpoint(targetCounts);
    pidDrive2Goal.setOutputRange(0, power);
    pidDrive2Goal.setInputRange(0, targetCounts); // TODO: allow negative goal
    pidDrive2Goal.setTolerance(0.01);
    pidDrive2Goal.enable();

    //int targetCounts = 5000;
    int countsDone = 0;
    leftMotor = hardwareMap.dcMotor.get("motor1");
    rightMotor = hardwareMap.dcMotor.get("motor0");

    // You will need to set this based on your robot's
    // gearing to get forward control input to result in
    // forward motion.
    leftMotor.setDirection(DcMotor.Direction.FORWARD);
    rightMotor.setDirection(DcMotor.Direction.REVERSE);

    leftMotor.setTargetPosition((int) targetCounts);
    rightMotor.setTargetPosition((int) targetCounts);

    // reset encoder counts kept by motors.
    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    pidDrive2Goal.setSetpoint(targetCounts);
    pidDrive2Goal.setOutputRange(0, power);
    pidDrive2Goal.setInputRange(0, targetCounts); // TODO: allow negative goal
    pidDrive2Goal.setTolerance(0.01);
    pidDrive2Goal.enable();

    do {
      telemetry.addData ("Counts done", "%d/%d ", countsDone, targetCounts);
      telemetry.update();
      countsDone = leftMotor.getCurrentPosition();
      power = pidDrive2Goal.performPID(leftMotor.getCurrentPosition());
      // Go!
      leftMotor.setPower(power);
      rightMotor.setPower(power);
    } while (opModeIsActive() && !pidDrive2Goal.onTarget());
  }

  /**
   * Rotate left or right the number of degrees. Does not support
   * turning more than 359 degrees.
   *
   * @param degrees Degrees to turn, + is left - is right
   */
  private void rotate(int degrees, double power) {
    telemetry.addData("command", "turn %f.1\u00B0 turn.");
    // restart imu angle tracking.
    resetAngle();

    // if degrees > 359 we cap at 359 with same sign as original degrees.
    if (Math.abs(degrees) > 359)
      degrees = (int) Math.copySign(359, degrees);

    // Start pid controller. PID controller will monitor the turn angle with
    // respect to the target angle and reduce power as we approach the target
    // angle. This is to prevent the robots momentum from overshooting the
    // turn after we turn off the power. The PID controller reports onTarget
    // () = true when the difference between turn angle and target angle is
    // within 1% of target (tolerance) which is about 1 degree. This helps
    // prevent overshoot. Overshoot depends on the motor and gearing
    // configuration, starting power, weight of the robot and the on target
    // tolerance. If the controller overshoots, it will reverse the sign of
    // the output turning the robot back toward the setpoint value.

    pidRotate.reset();
    pidRotate.setSetpoint(degrees);
    pidRotate.setInputRange(0, degrees);
    pidRotate.setOutputRange(0, power);
    pidRotate.setTolerance(0.01);
    pidRotate.enable();

    // getAngle() returns + when rotating counter clockwise (left) and - when
    // rotating clockwise (right).

    // rotate until turn is completed.

    if (degrees < 0) {
      do {
        power = pidRotate.performPID(getAngle()); // power will be - on right
        // turn.
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
      } while (opModeIsActive() && !pidRotate.onTarget());
    } else    // left turn.
      do {
        power = pidRotate.performPID(getAngle()); // power will be + on left
        // turn.
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
      } while (opModeIsActive() && !pidRotate.onTarget());
    // reset angle tracking on new heading.
    stopMotors();
    pidDrive.disable();
    pidDrive2Goal.disable();

    // wait for motors to calm down.
    sleep(500);

    rotation = getAngle();
    resetAngle();
  }
}
