/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode operates a single servo arm forward and back, using the gamepad Y button
 * (DEPLOYed) the A button to go back (STOWed). The code is structured as a LinearOpMode.
 *
 * The arm will move as long as one of those buttons is pressed, and stop if
 * they are left alone.
 *
 */

@TeleOp(name = "Operate Arm Slow Buttons", group = "Actuators")
//@Disabled
public class OperateArmSlowButtonsAY extends LinearOpMode {

  static final double STOWED      =  0.0;    // Retracted over robot body
  static final double DEPLOYED    =  0.8;    // Extended out over Field. 1.0 for HiTEKs.
  static final double ITSY_BITSY  = 0.0004;

  // Define class members
  Servo arm;
  double position;

  @Override
  public void runOpMode() {
    //  Initialize servo and arm.
    arm = hardwareMap.get(Servo.class, "arm");
    arm.setPosition(STOWED);
    position = arm.getPosition();
    telemetry.addData("Arm starting at", "%5.2f", position);
    telemetry.update();

    waitForStart();

    //  Full scale for now.
    double positionScale = DEPLOYED - STOWED;
    while(opModeIsActive()){
      position = arm.getPosition();
      if (gamepad1.a) {
        if (position - ITSY_BITSY > STOWED) position -= ITSY_BITSY;
        telemetry.addLine("A");
        arm.setPosition(position);
      }
      if (gamepad1.y) {
        if (position + ITSY_BITSY < DEPLOYED) position += ITSY_BITSY;
        telemetry.addLine("Y");
        arm.setPosition(position);
      }

      // Display the current position and active command.
      telemetry.addData("Arm Position", "%5.2f", position);
      telemetry.update();
      sleep (1);
    }
  }
}