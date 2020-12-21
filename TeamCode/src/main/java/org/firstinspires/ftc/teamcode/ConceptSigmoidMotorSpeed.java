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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode illustrates sigmoid tempering of robot movement. It is
 * structured as a LinearOpMode.
 * <p>
 * It makes a variable v, like robot speed, follow a sigmoid speed vs time
 * function over the interval (0, 1). In that interval, the variable
 * increases from zero to 1:
 * <p>
 *          v (t) = 0.5 - 0.5 * cos (πt)
 * <p>
 * This function starts increasing v slowly from zero, builds up some v until
 * the requested vf is reached, then slows down to gently approach v = 0.
 * <p>
 * The domain and range can be scaled. The initial value of v can be other than
 * zero:
 * <p>
 *          v (t) = Vo + Vscale * (0.5 - 0.5 * cos (πt/period))
 * <p>
 * CYCLE_MS sets the update period for position and reports.
 * <p>
 * We assume an instance of a robot class with drive train methods implemented.
 * Here, the demo robot is a Pullbot. We will assign robot speed to the
 * tempered variable v.
 * <p>
 * Button mapping assumes a logitech 310 game pad.
 */
@TeleOp(name = "Concept: Sigmoid Motor Speed", group = "Concept")
//@Disabled
public class ConceptSigmoidMotorSpeed extends LinearOpMode {

	DcMotor leftMotor;
	DcMotor rightMotor;
	double speed;
	double minSpeed = 0.0;
	double startingSpeed;
	double targetSpeed;
	double speedScale;
	double time2DoIt = 1.0; // seconds to go from start speed to target speed.
	double time;

	@Override
	public void runOpMode() {
		ElapsedTime runtime = new ElapsedTime();

		Pullbot robot = new Pullbot(this);
		robot.init(hardwareMap);
		robot.setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

		waitForStart();

		targetSpeed = 0.5;
		double targetFieldSpeed = targetSpeed * robot.MAX_DRIVE_SPEED;
		startingSpeed = minSpeed;
		telemetry.addData(
				"Testing", "%5.2f sec speed change from %5.3f to %5.3f",
				time2DoIt, startingSpeed, targetSpeed);
		telemetry.addData(">", "Press Start to run.");
		telemetry.update();

		//robot.changeSpeedSigmoid(time2DoIt, startingSpeed, targetSpeed, 0.85);
		//robot.changeSpeedSigmoid(time2DoIt, targetSpeed, startingSpeed, 1.0/0.85);
		//robot.accelerateDistanceSigmoid(24, startingSpeed, targetSpeed, 0.85);
		//  Decelerate now from target speed to rest.
		//robot.accelerateDistanceSigmoid(24, targetSpeed, startingSpeed, 1.0/0.85);
		//robot.turnArcRadiusSigmoid(startingSpeed, targetSpeed, 12, 100);
		//robot.turnArcRadiusSigmoid(targetSpeed, startingSpeed, 12, 100);
		//  Implement Path C.
		//robot.turnArcRadiusSigmoid (minSpeed, 1.0, 24.0, -200.0);
		//robot.turnArcRadiusSigmoid(1.0, 1.0, 80.0, -200.0);
		//robot.turnArcRadiusSigmoid(1.0, 0.0, 24.0, -200.0);
		robot.turnArcRadiusSigmoid(minSpeed, -0.3, -48.0, 120.0); // backwards port
		robot.turnArcRadiusSigmoid(-0.30, minSpeed, -48.0, -120.0); // backwards
		// starboard
		// Turn off motors and signal completion.
		//robot.leftDrive.setPower(0.0);
		//robot.rightDrive.setPower(0.0);
		sleep (5000);
		//telemetry.addData(">", "Done");
		//telemetry.update();
	}
}
