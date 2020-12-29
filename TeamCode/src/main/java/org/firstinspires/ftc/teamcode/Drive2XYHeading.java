/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia
 * localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 * <p>
 * When images are located, Vuforia is able to determine the position and
 * orientation of the
 * image relative to the camera.  This sample code then combines that
 * information with a
 * knowledge of where the target images are on the field, to determine the
 * location of the camera.
 * <p>
 * From the Audience perspective, the Red Alliance station is on the right
 * and the
 * Blue Alliance Station is on the left.
 * <p>
 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance,
 * Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of
 * each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 * <p>
 * A final calculation then uses the location of the camera on the robot to
 * determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver
 * Station OpMode list.
 * <p>
 * IMPORTANT: In order to use this OpMode, you need to obtain your own
 * Vuforia license key as
 * is explained below.
 */

@TeleOp(name = "Vuforia assisted drive somewhere", group = "Concept")
//@Disabled
public class Drive2XYHeading extends LinearOpMode {

  private OpenGLMatrix lastLocation = null;
  //private VuforiaLocalizer vuforia = null;
  private boolean targetVisible = false;
  private float phoneXRotate = 0;
  private float phoneYRotate = 0;
  private float phoneZRotate = 0;

  double yCorrection = -0.04;
  double headingCorrection = 0.5;
  static final double STRAIGHT_SPEED = 0.6;
  static final double TURN_SPEED = 0.2;
  static final double MAX_CORRECTION = TURN_SPEED;
  static final double MIN_CORRECTION = -TURN_SPEED;
  double targetX = 74.0; // Aim for robot front to end up near the picture.
  double currentX = 0;  // We'll refine this by Vuforia if target image is
  // visible.
  double errorX = currentX - targetX;
  static final double ERROR_X_TOLERANCE = 16.0;
  // avoids large and unstable bearing changes on final approach.
  double targetY = 35.5; // Also so robot can be near the picture.
  double currentY;
  double errorY;
  static final double ERROR_Y_TOLERANCE = 1.0;
  double targetHeadingDegrees = 0.0;
  double targetHeadingRadians = targetHeadingDegrees * Math.PI / 180.0;
  double currentHeadingDegrees;
  double currentHeadingRadians;
  double errorHeadingDegrees;
  double errorHeadingRadians;

  double targetBearingRadians = 0.0;
  double targetBearingDegrees = targetBearingRadians * 180 / Math.PI;
  double currentBearingRadians;
  double errorBearingRadians;
  double currentBearingDegrees;
  double errorBearingDegrees;

  double correction = 0.0;

  @Override
  public void runOpMode() {
    while (!gamepad1.y) {
      telemetry.addLine("Tuning controls. Press yellow Y button to " +
          "finish tuning, even if no tuning done.");
      telemetry.addLine("  Pad left/right: more/less Y error correction" +
          ".");
      telemetry.addLine("  Green B button: more heading correction.");
      telemetry.addLine("  Blue X button: less heading correction.");
      telemetry.addLine("Press the Yellow Y button to finish tuning.");
      if (gamepad1.dpad_right) {
        gamepad1.dpad_right = false;
        yCorrection *= 1.1;
        sleep(300);
      }
      if (gamepad1.dpad_left) {
        gamepad1.dpad_left = false;
        yCorrection /= 1.1;
        sleep(300);
      }
      if (gamepad1.b) {
        gamepad1.b = false;
        headingCorrection *= 1.1;
        sleep(300);
      }
      if (gamepad1.x) {
        gamepad1.x = false;
        headingCorrection /= 1.1;
        sleep(300);
      }
      telemetry.addData("Bearing correction",
          " %5.3f  Heading correction %5.3f", yCorrection,
          headingCorrection);
      telemetry.update();
    }
    Pullbot robot = new Pullbot(this);
    Navigator navigator = new Navigator(this);
    String initReport = robot.init(hardwareMap);
    initReport += navigator.init(hardwareMap);
    telemetry.addData("Robot status", "initialized.");
    telemetry.addData("Initialization report", initReport);
    telemetry.update();

    while (!isStopRequested()) {
      targetVisible = false;
      for (VuforiaTrackable trackable : navigator.allTrackables) {
        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
          telemetry.addData("Visible Target", trackable.getName());
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
        telemetry.addData("Pos ", "{X, Y} = %5.1f\", %5.1f\"",
            translation.get(0) / GenericFTCRobot.mmPerInch,
            translation.get(1) / GenericFTCRobot.mmPerInch);
        Orientation rotation =
            Orientation.getOrientation(lastLocation, EXTRINSIC,
                XYZ, DEGREES);
        telemetry.addData("Heading", "%4.0f\u00B0", rotation.thirdAngle);
      } else {
        telemetry.addData("Visible Target", "none");
      }
      telemetry.update();
      // Report where the robot is located, if we can see a Vuforia image.
      if (targetVisible && Math.abs(errorX) > ERROR_X_TOLERANCE) {

        // Report position (translation) and position error of robot in inches.
        VectorF translation = lastLocation.getTranslation();
        currentX = translation.get(0) / GenericFTCRobot.mmPerInch;
        currentY = translation.get(1) / GenericFTCRobot.mmPerInch;
        errorX = currentX - targetX;
        errorY = currentY - targetY;
        telemetry.addData("Position error", "X, Y = %4.1f, %4.1f",
            errorX, errorY);

        // Report bearing and bearing error of target from robot.
        currentBearingRadians = Math.atan2(-errorY, -errorX);
        currentBearingDegrees = currentBearingRadians * 180.0 / Math.PI;
        errorBearingRadians = currentBearingRadians - targetBearingRadians;
        errorBearingDegrees = errorBearingRadians * 180.0 / Math.PI;
        telemetry.addData("Bearing", " %4.0f\u00B0  error: %4.0f\u00B0",
            currentBearingDegrees, errorBearingDegrees);

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
        telemetry.addData(
            "Heading", "%4.0f\u00B0  Heading error %4.0f\u00B0",
            currentHeadingDegrees, errorHeadingDegrees);

        // find motor speed corrections.
        correction =
            yCorrection * errorY - headingCorrection * errorHeadingRadians;
        correction = Math.min(Math.max(correction, MIN_CORRECTION),
            MAX_CORRECTION);
        // Todo: slow down when errorX gets small.

        //  Apply those corrections to drive motors.
        // This is a Pullbot, not a Pushbot.
        robot.leftDrive.setPower(-TURN_SPEED + correction);
        robot.rightDrive.setPower(-TURN_SPEED - correction);
        telemetry.addData("Motor speeds",
            "correction %5.3f left %5.3f right %5.3f",
            correction, TURN_SPEED - correction, TURN_SPEED + correction);

      } else if (!targetVisible) {
        telemetry.addLine("Visible target lost. Stopping.");
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        // Todo: try to recover from this by turning on axis, guessing
        //  from last known position.
      } else {
        telemetry.addLine("We have arrived. Stopping.");
        // Clean up residual heading error.
        robot.turnAngle(TURN_SPEED, -errorHeadingRadians);
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        stop();
      }
    }

  }
}
