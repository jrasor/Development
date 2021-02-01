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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name = "Scrimmage 4 TeleOp", group = "Competition")
//@Disabled
public class Scrimmage4TeleOp extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private OpenGLMatrix lastLocation = null;
  private VuforiaLocalizer vuforia = null;
  private boolean targetVisible = false;
  private float phoneXRotate = 0;
  private float phoneYRotate = 0;
  private float phoneZRotate = 0;

  @Override
  public void runOpMode() {
    Pullbot robot = new Pullbot(this);
    Navigator navigator = new Navigator(this);
    String initReport = robot.init(hardwareMap);
    initReport += navigator.init(hardwareMap);
    telemetry.addData("Robot status", "initialized.");
    telemetry.addData("Initialization report", initReport);
    telemetry.update();
    
    waitForStart();
    runtime.reset();
    while (opModeIsActive()) {
      robot.simpleDrive();
      //robot.tankDrive();
      //robot.oneStickDrive();
      robot.enableNudge();
      robot.enableArm();

      // check all the trackable targets to see which one (if any) is visible.
      targetVisible = false;
      try {for (VuforiaTrackable trackable : navigator.allTrackables) {
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
      }} catch (Exception e) {
        telemetry.addLine ("Failed to get a visible image: "
            + e.toString() + ". ");
        sleep (2000);
        stop();
      }

      // Report robot location if we know.
      if (targetVisible) {
        // Report position (translation) of robot in inches.
        VectorF translation = lastLocation.getTranslation();
        telemetry.addData("Pos (in)", "{X, Y} = %6.1f, %6.1f",
            translation.get(0) / GenericFTCRobot.mmPerInch,
            translation.get(1) / GenericFTCRobot.mmPerInch);

        // Report the rotation of the robot in degrees.
        Orientation rotation = Orientation.getOrientation(lastLocation,
            EXTRINSIC, XYZ, DEGREES);
        telemetry.addData("Heading", " %4.0f\u00B0", rotation.thirdAngle);
      } else {
        telemetry.addData("Visible Target", "none");
      }
      telemetry.addData("Encoder counts", robot.arm.getCurrentPosition());
      telemetry.update();
    }
  }
}