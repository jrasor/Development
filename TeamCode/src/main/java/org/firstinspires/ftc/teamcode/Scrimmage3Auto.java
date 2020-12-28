/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *  all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 *   Counts Rings in the Starter Stack, and pushes a Wobble Goal to the
 * appropriate Target Zone.
 * o no Rings case: Put Wobble Goal in Zone A.
 * o 1 Ring case: Go for Zone B.
 * o 4 Ring case: Go for Zone C.
 */

@Autonomous(name = "Scrimmage 3 Auto", group = "Competition")
//@Disabled
public class Scrimmage3Auto extends LinearOpMode {

  Pullbot robot = new Pullbot(this);
  String initReport = "";
  double straightSpeed = 0.60;
  double turnSpeed = 0.30;
  boolean doPaths = true;

  private void doPathA (){
    robot.turnArcRadiusDrive (turnSpeed, 84.0, 80.0);
  }

  private void doPathB (){
    robot.turnArcRadiusDrive (turnSpeed, 96.0, 240.0);
    robot.turnArcRadiusDrive (turnSpeed, -20.0, 240.0);
  }

  private void doPathC (){
    //  Push Wobbler to Goal Zone C
    //robot.turnArcRadiusDrive (turnSpeed, 123.0, 157.0);
    //robot.turnArcRadiusDrive (turnSpeed, -46.0, 157.0);
    robot.turnArcRadiusSigmoid(0.0, 1.0, 24.0, 190.0);
    robot.turnArcRadiusSigmoid(1.0, 1.0, 60.0, 190.0);
    robot.turnArcRadiusSigmoid(1.0, 0.0, 24.0, 190.0);
    //  Back out to the Launch Line.
    robot.turnArcRadiusSigmoid(0.0, -1.0, -20.0, 190.0);
    robot.turnArcRadiusSigmoid(-1.0, 0.0, -20.0, 190.0);
  }

  @Override
  public void runOpMode() {
    initReport = robot.init(hardwareMap);

    // Tell telemetry to update faster than the default 250ms period :)
    //telemetry.setMsTransmissionInterval(100);
    telemetry.addLine (initReport);
    telemetry.update();
//    sleep (3000);

    waitForStart();

    int ringsDetected = robot.CountRings(robot.cameraMonitorViewId);

    switch (ringsDetected) {
      case (0): {
        telemetry.addLine("Going to Zone A.");
        telemetry.update();
        if (doPaths) doPathA();
        // no backward move to park on Launch Line needed; robot ends up
        // there on pushing the Wobble Goal.
        telemetry.addLine("A path Complete.");
        telemetry.update();
        break;
      }
      case (1): {
        telemetry.addLine("Going to Zone B.");
        telemetry.update();
        if (doPaths) doPathB();
        telemetry.addLine("B path Complete.");
        telemetry.update();
        break;
      }
      case (4): {
        telemetry.addLine("Going to Zone C.");
        telemetry.update();
        if (doPaths) doPathC();
        telemetry.addLine("C path Complete.");
        telemetry.update();
        break;
      }
      default:
        telemetry.addLine(
            "I'm lost. Going to Zone A, and hoping for the best.");
        ringsDetected = 0;
        if (doPaths) doPathA();
        telemetry.update();
    }
    telemetry.update();
    sleep (2000);
  }
}