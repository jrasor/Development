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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//import static org.firstinspires.ftc.teamcode.Navigator
// .RingOrientationAnalysisPipeline.Stage.values;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all needed for an FTC robot to determine
 * its position on the Field.
 */

/* Version history
 * ======= =======
 * v 0.1    12/24/20 copied from Pullbot, then hollowed out to near stub
 *          status. It only uses Vuforia. Later versions may use OpenCV to
 *          detect known Field features, like Walls, floor markings, Ultimate
 *          Goal Towers.
 */

public class Navigator extends GenericFTCRobot {

  // Vision properties
  public OpenCvInternalCamera2 phoneCam;
  public static boolean PHONE_IS_PORTRAIT = false;
  public int cameraMonitorViewId;

  // Field related constants.
  public static final float mmPerInch = 25.4f; // use mm dimensions
  // Constants for perimeter Vuforia navigation targets
  // Field outside: 12'. Inside: 1" shorter than that, each Wall.
  public static final float fullField = 142 * mmPerInch;
  public static final float halfField = fullField / 2;
  public static final float quarterField = fullField / 4;
  // the height of the center of the target image above the floor
  public static final float mmTargetHeight = (6) * mmPerInch;

  // Pullbot specific sensor members.
  public ColorSensor colorSensor;
  public static VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
  /* local OpMode members. */

  // Initialization.
  HardwareMap hwMap = null;
  private LinearOpMode currentOpMode;
  private ElapsedTime period = new ElapsedTime();

  /* Constructors */
  public Navigator() {
    super();
  }
  public Navigator(LinearOpMode linearOpMode) {
    currentOpMode = linearOpMode;
  }

  public String init(HardwareMap someHWMap) {
    hwMap = someHWMap;
    String initializationReport = "";
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
      }
    });

    return initializationReport;
  }

  public String Hello (){
    return "Hello, world, from Navigator!";
  }
  /*
   *										Vision methods
   */
}
