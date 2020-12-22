/* Copyright (c) 2019 FIRST. All rights reserved.
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

@TeleOp(name="Drive2XYHeading", group ="Concept")
//@Disabled
public class Drive2XYHeading extends LinearOpMode {
    private Pullbot robot = new Pullbot(this);
    private boolean targetVisible = false;
    private float phoneXRotate    = 0.0f;
    private float phoneYRotate    = 0.0f;
    private float phoneZRotate    = 0.0f;
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
    double targetHeadingRadians = targetHeadingDegrees * Math.PI/180.0;
    double currentHeadingDegrees;
    double currentHeadingRadians;
    double errorHeadingDegrees;
    double errorHeadingRadians;

    double targetBearingRadians = 0.0;
    double targetBearingDegrees = targetBearingRadians * 180/ Math.PI;
    double currentBearingRadians;
    double errorBearingRadians;
    double currentBearingDegrees;
    double errorBearingDegrees;

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.2;
    static final double Y_CORRECTION = -0.04;
    static final double HEADING_CORRECTION = 0.5;
    static final double MAX_CORRECTION = TURN_SPEED;
    static final double MIN_CORRECTION = -TURN_SPEED;

    double correction = 0.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        int cameraMonitorViewId =
            hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.lastLocation = null;
        VuforiaLocalizer.Parameters parameters =
            new VuforiaLocalizer.Parameters(robot.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = robot.VUFORIA_KEY;
        parameters.cameraDirection   = robot.CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
        VuforiaTrackables targetsUltimateGoal =
            robot.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -robot.halfField, robot.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, robot.halfField, robot.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-robot.halfField, 0,
                    robot.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(robot.halfField, robot.quarterField,
                    robot.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(robot.halfField, -robot.quarterField,
                    robot.mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        if (robot.CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (robot.PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(robot.CAMERA_FORWARD_DISPLACEMENT,
                        robot.CAMERA_LEFT_DISPLACEMENT,
                        robot.CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsUltimateGoal.activate();

        waitForStart();

        while (!isStopRequested()) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        robot.lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Report robot location and heading (if we know).
            if (targetVisible) {
                VectorF translation = robot.lastLocation.getTranslation();
                telemetry.addData("Pos ", "{X, Y} = %5.1f\", %5.1f\"",
                        translation.get(0) / GenericFTCRobot.mmPerInch,
                    translation.get(1) / GenericFTCRobot.mmPerInch);
                Orientation rotation =
                    Orientation.getOrientation(robot.lastLocation, EXTRINSIC,
                        XYZ, DEGREES);
                telemetry.addData("Heading", "%4.0f\u00B0", rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
            // Report where the robot is located, if we can see a Vuforia image.
            if (targetVisible && Math.abs(errorX) > ERROR_X_TOLERANCE) {

                // Report position (translation) and position error of robot in inches.
                VectorF translation = robot.lastLocation.getTranslation();
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
                    Orientation.getOrientation(robot.lastLocation, EXTRINSIC,
                        XYZ,
                        DEGREES);
                currentHeadingDegrees = rotation.thirdAngle;
                currentHeadingRadians = currentHeadingDegrees * Math.PI/180.0;
                errorHeadingDegrees = currentHeadingDegrees - targetHeadingDegrees;
                errorBearingRadians = currentBearingRadians - targetBearingRadians;
                errorHeadingRadians = errorHeadingDegrees * Math.PI/180.0;
                telemetry.addData(
                    "Heading", "%4.0f\u00B0  Heading error %4.0f\u00B0",
                    currentHeadingDegrees, errorHeadingDegrees);

                // find motor speed corrections.
                correction =
                    Y_CORRECTION * errorY - HEADING_CORRECTION * errorHeadingRadians;
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
                robot.rightDrive.setPower(0.0); // Todo: make a Pullbot method for this.
                // Todo: try to recover from this by turning on axis, guessing
                //  from last known position.
            } else {
                telemetry.addLine("We have arrived. Stopping.");
                // Clean up residual heading error.
                robot.turnAngle (TURN_SPEED, -errorHeadingRadians);
                robot.leftDrive.setPower(0.0);
                robot.rightDrive.setPower(0.0);
                // Disable Tracking when we are done.
                targetsUltimateGoal.deactivate();
                stop();
            }
        }
    }
}
