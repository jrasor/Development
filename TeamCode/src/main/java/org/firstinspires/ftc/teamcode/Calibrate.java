/* Copyright (c) 2018 FIRST. All rights reserved.
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

/*
 * Drive a robot around, and put it on a convenient starting place like a
 * tile corner. The gamepad
 * buttons will run macros. Each macro is one or more Trainerbot movements.
 * Look below to the gamepad button handlers to see what they claim to do. To
 *  calibrate:
 *  - Predict ending place for a macro.
 *  - Run it.
 *  - See if ending place agrees with prediction.
 *  - Tweak Trainerbot properties WHEEL_SEPARATION and WHEEL_DIAMETER_INCHES.
 *  Repeat as necessary to get desired agreement between prediction and
 * actual performance.
 *  CAUTION: wheel slippage can reduce actual movement from that predicted.
 * Try to minimize it.
 *
 * Button mapping assumes a logitech 310 game pad.
 */

@TeleOp(name = "Calibrate", group = "Trainerbot")
//@Disabled
public class Calibrate extends LinearOpMode {
  private static final double DRIVE_SPEED = 0.30;  // was 0.6
  private static final double TURN_SPEED = 0.15;  // was 0.2
  Pullbot robot = new Pullbot(this);

  @Override
  public void runOpMode() {
    robot.init(hardwareMap);
    telemetry.addData("Hardware", " mapped.");
    telemetry.update();

    /* Wait for calibration session to begin */
    telemetry.addData("Operation", "Press Play to activate button macros.");
    telemetry.addData("  Orange Y button", "straight 72\" run.");
    telemetry.addData("  Blue X button", "90° turn CCW, 24\" radius.");
    telemetry.addData("  Red B button", "90° turn CW, 24\" radius.");
    telemetry.addData("  Green A button", "90° spin CW on own axis.");
    telemetry.addData("  Trigger", "forward 2 tiles, left 1.");
    telemetry.update();
    waitForStart();


    while (opModeIsActive()) {
      robot.simpleDrive();
      //robot.tankDrive();
      if (gamepad1.y) {
        // Orange button. Forward one tile.
        robot.driveStraight(DRIVE_SPEED, 72.0);
        //robot.encoderDrive(DRIVE_SPEED, DRIVE_SPEED, 24.0, 24.0);
      }
      if (gamepad1.x) {
        // Blue button. Run curving CCW, 90 degrees. From corner of one tile
        // to opposite corner.
        //   robot.turnAngleRadiusDrive(TURN_SPEED, Math.PI / 2, 24.0);
        // Turn using right wheel only. Fails: right goes a bit, then both stop.
        robot.turnAngleRadiusDrive(TURN_SPEED, Math.PI / 2, 24.0);
      }
      if (gamepad1.b) {
        // Red button. Run forward curving CW, 90 degrees, corner of one tile
        // to opposite
        // corner.
        robot.turnAngleRadiusDrive(TURN_SPEED, -Math.PI / 2, -24.0);
      }
      if (gamepad1.a) {
        // Green button. Turn on axis CW, as though joysticks pushed equal
        // amounts, opposite
        // directions.
        robot.turnAngle(TURN_SPEED, -Math.PI / 2);
      }
      if (gamepad1.left_trigger > 0) {
        // Sigmoid drive forward 2 tiles and left 1 tile.
        //robot.Drive2XYSigmoid(24.0, 48.0);
        // Make sure we're using the encoders
        robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Stub: run the outgoing C path.
        //robot.turnArcRadiusSigmoid(0.0, 1.0, 24.0, 185.0);
        //robot.turnArcRadiusSigmoid(1.0, 1.0, 78.0, 185.0);
        //robot.turnArcRadiusSigmoid(1.0, 0.0, 24.0, 185.0);
        // Stub: run something like outgoing path C.
        robot.turnArcRadiusSigmoid(0.0, 0.5, 24.0, 24.0);
        robot.turnArcRadiusSigmoid(0.5, 0.0, 24.0, -24.0);
      }
    }
  }
}
