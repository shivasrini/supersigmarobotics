/* Copyright (c) 2023 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name = "TestFtcOpMode", group = "Concept")
public class TestFtcOpMode extends LinearOpMode {

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 13.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02; //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015; //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01; //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    double rangeError;
    double headingError;
    double yawError;
    final double MAX_AUTO_SPEED = 0.5; //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5; //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3; //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive = null; //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null; //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null; //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null; //  Used to control the right back drive wheel

    //private Servo servo_left;
    //private Servo servo_right;
    private DcMotor linear_slide;
    private DcMotor motor_0;
    private DcMotor motor_2;
    private DcMotor motor_1;
    private DcMotor motor_3;
    private DcMotor motor_lever;

    private static final boolean USE_WEBCAM = true; // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 3; // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null; // Used to hold the data for a detected AprilTag

    private static boolean reached = false;
    //start BlueTop
    private Servo servo_left;
    private Servo servo_right;
    //   private DcMotor linear_slide;
    //   private DcMotor motor_0;
    //   private DcMotor motor_2;
    //   private DcMotor motor_1;
    //   private DcMotor motor_3;
    //   private DcMotor motor_lever;

    double ANGLE_OF_OBJECT;
    List<Recognition> myTfodRecognitions;
    int motor0Pos;
    int motor1Pos;
    TfodProcessor myTfodProcessor;
    int motor2Pos;
    //   boolean USE_WEBCAM;
    int motor_lever_pos;
    int motor3Pos;
    int motor_lever_pos_NEW;

    //end BlueTop
    public void msg(String m) {
        telemetry.addLine(m);
        telemetry.update();
        sleep(1000);
    }

//start bluetop

    public void runOpMode_BT() {
        int aprilTagVal;
        msg("starting to initilaze motors");
        servo_left = hardwareMap.get(Servo.class, "servo_left");
        servo_right = hardwareMap.get(Servo.class, "servo_right");
        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");
        motor_0 = hardwareMap.get(DcMotor.class, "motor_0");
        motor_2 = hardwareMap.get(DcMotor.class, "motor_2");
        motor_1 = hardwareMap.get(DcMotor.class, "motor_1");
        motor_3 = hardwareMap.get(DcMotor.class, "motor_3");
        motor_lever = hardwareMap.get(DcMotor.class, "motor_lever");
        msg("ending to initilaze motors");

        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection, using
        // a custom TFLite object detection model.
        //USE_WEBCAM = true;
        servo_left.setPosition(0.38);
        servo_right.setPosition(0.22);
        // Initialize TFOD before waitForStart.
        initTfod();
        // Wait for the match to begin.
        // Indicate that only the zoomed center area of each
        // image will be passed to the TensorFlow object
        // detector. For no zooming, set magnification to 1.0.
        myTfodProcessor.setZoom(1);
        // Set the minimum confidence at which to keep recognitions.
        myTfodProcessor.setMinResultConfidence((float) 0.6);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("key", 123);
        telemetryTfod();
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetryTfod();
                // Push telemetry to the Driver Station.
                telemetry.update();
                motor_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (JavaUtil.listLength(myTfodRecognitions) > 0) {
                    if (ANGLE_OF_OBJECT < -6) {
                        aprilTagVal = 1;
                        telemetry.addData("Left", 123);
                        // Push telemetry to the Driver Station.
                        telemetry.update();
                        left();
                    } else if (ANGLE_OF_OBJECT > -6) {
                        aprilTagVal = 2;
                        telemetry.addData("Straight", 123);
                        // Push telemetry to the Driver Station.
                        telemetry.update();
                        straight();
                    } else {
                        sleep(100);
                    }
                } else if (JavaUtil.listLength(myTfodRecognitions) == 0) {
                    right2();
                } else {
                }
            }
        }
    }

    //end bluetop
    @Override
    public void runOpMode() {
        msg("In runOp Mode");

        boolean targetFound = false; // Set to true when an AprilTag target is detected
        double drive = 0; // Desired forward power/speed (-1 to +1)
        double strafe = 0; // Desired strafe power/speed (-1 to +1)
        double turn = 0; // Desired turning power/speed (-1 to +1)

        msg("before init april tag");

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motor_1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor_0");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motor_3");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor_2");
        motor_lever = hardwareMap.get(DcMotor.class, "motor_lever");
        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        msg("Using Webcam");

        if (USE_WEBCAM) setManualExposure(6, 250); // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        msg("in loop");
        msg("move robot straight");

        //drive2(500,500,500,500,0.3);
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (
                        (detection.metadata != null) &&
                                ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))
                ) {
                    targetFound = true;
                    desiredTag = detection;
                    break; // don't look any further.
                } else {
                    telemetry.addData(
                            "Unknown Target",
                            "Tag ID %d is not in TagLibrary\n",
                            detection.id
                    );
                }
            }

            // Tell the driver what we see, and what to do.
            // if (targetFound) {
            //     telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            //     telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            //     telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            //     telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            //     telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            // } else {
            //     telemetry.addData(">","Drive using joysticks to find valid target\n");
            // }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            //msg("if(targetFound)");
            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                reached = false;
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;
                if (rangeError <= 0.1) {
                    telemetry.addData(">", "breakingout!!!!");
                    telemetry.update();
                    sleep(3000);
                    break;
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                //msg("setting up moving stuff");
                drive =
                        Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn =
                        Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe =
                        Range.clip(
                                -yawError * STRAFE_GAIN,
                                -MAX_AUTO_STRAFE,
                                MAX_AUTO_STRAFE
                        );
                telemetry.addData(
                        "Auto",
                        "Drive %5.2f, Strafe %5.2f, Turn %5.2f, Rangeerror %5.2f",
                        drive,
                        strafe,
                        turn,
                        rangeError
                );
                telemetry.update();
            }

            //telemetry.addLine("moving.
            //telemetry.update();

            if ((reached == false)) {
                moveRobot(drive, strafe, turn);
            }
            if (reached) {
                break;
            }
            sleep(10);
        } //while
        msg("Done with While loop");
        hold_claw_pos(900, 0.4);
        driveAT(400, 400, 400, 400, 0.2);
        slide(400, 0.2);
        hold_claw_pos(100, -0.4);
        //drive2(-300,300,-300,-300,0.5);

        // telemetry.addData(">","starting to go straight");
        // telemetry.update();
        // sleep(3000);
        //drive2(df)
        // telemetry.addData(">","done going straight");
        // telemetry.update();
        // sleep(3000);

    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        double limit = 0.18;
        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        telemetry.addData(
                "power",
                "lf %5.2f, rf %5.2f, lb %5.2f, rb %5.2f",
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );
        telemetry.update();
        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        if (
                (leftFrontPower < limit) &&
                        (rightFrontPower < limit) &&
                        (leftBackPower < limit) &&
                        (rightBackPower < limit)
        ) {
            reached = true;
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal =
                    new VisionPortal.Builder()
                            .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                            .addProcessor(aprilTag)
                            .build();
        } else {
            visionPortal =
                    new VisionPortal.Builder()
                            .setCamera(BuiltinCameraDirection.BACK)
                            .addProcessor(aprilTag)
                            .build();
        }
    }

    /*
        Manually set the camera gain and exposure.
        This can only be called AFTER calling initAprilTag(), and only works for Webcams;
       */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (
                    !isStopRequested() &&
                            (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            ) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(
                    ExposureControl.class
            );
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(
                    GainControl.class
            );
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    /**
     * Describe this function...
     */
    private void driveAT(
            int leftFrontTarget,
            int rightFrontTarget,
            int leftBackTarget,
            int rightBackTarget,
            double speed
    ) {
        double motor0Pos = 0;
        double motor1Pos = 0;
        double motor2Pos = 0;
        double motor3Pos = 0;
        motor0Pos += rightFrontTarget;
        motor1Pos += leftFrontTarget;
        motor2Pos += rightBackTarget;
        motor3Pos += leftBackTarget;

        motor_0 = rightFrontDrive;
        motor_1 = leftFrontDrive;
        motor_2 = rightBackDrive;
        motor_3 = leftBackDrive;

        motor_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor_0.setTargetPosition(1 * rightFrontTarget);
        motor_1.setTargetPosition(1 * leftFrontTarget);
        motor_2.setTargetPosition(1 * rightBackTarget);
        motor_3.setTargetPosition(1 * leftBackTarget);

        motor_0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor_0.setPower(speed);
        motor_1.setPower(speed);
        motor_2.setPower(speed);
        motor_3.setPower(speed);

        while (
                opModeIsActive() &&
                        motor_0.isBusy() &&
                        motor_3.isBusy() &&
                        motor_2.isBusy() &&
                        motor_1.isBusy()
        ) {
            idle();
        }
    }

    private void hold_claw_pos(int lTarget, double power) {
        telemetry.addData("current pos pre", motor_lever.getCurrentPosition());
        telemetry.update();
        motor_lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("current pos post", motor_lever.getCurrentPosition());
        telemetry.update();
        double motor_lever_pos = 0;
        double motor_lever_pos_NEW = 0;
        motor_lever_pos = motor_lever.getCurrentPosition();
        motor_lever_pos_NEW = lTarget;
        telemetry.addData("motor levere pos new", -1 * motor_lever_pos_NEW);
        if (power < 0) {
            motor_lever.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor_lever.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        telemetry.update();
        motor_lever.setTargetPosition(lTarget);
        motor_lever.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_lever.setPower(0.1);
        while (motor_lever.getCurrentPosition() > -1 * motor_lever_pos_NEW) {
            idle();
            telemetry.addData("INLOOP_Original Curr Pos", motor_lever_pos);
            telemetry.addData("In Loop: Curr Pos", motor_lever.getCurrentPosition());
            telemetry.update();
        }
        motor_lever.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_lever.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Motor_lever status", "DONE and stopped");
        telemetry.addData("Original Curr Pos", motor_lever_pos);
        telemetry.addData("Curr Pos", motor_lever.getCurrentPosition());
        telemetry.update();
    }

    private void slide(int LStarget, double LSpower) {
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("current pos post", linear_slide.getCurrentPosition());
        telemetry.update();
        double motor_lever_pos = 0;
        double motor_lever_pos_NEW = 0;
        motor_lever_pos = linear_slide.getCurrentPosition();
        motor_lever_pos_NEW = LStarget;
        telemetry.addData("motor levere pos new", motor_lever_pos_NEW);
        telemetry.update();
        if (LSpower < 0) {
            linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        linear_slide.setTargetPosition(LStarget);
        linear_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear_slide.setPower(Math.abs(LSpower));
        while (linear_slide.getCurrentPosition() < motor_lever_pos_NEW) {
            idle();
            telemetry.addData("INLOOP_Original Curr Pos", motor_lever_pos);
            telemetry.addData("In Loop: Curr Pos", linear_slide.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Motor_lever status", "DONE and stopped");
        telemetry.update();
        linear_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Motor_lever status", "DONE and stopped");
        telemetry.addData("Original Curr Pos", motor_lever_pos);
        telemetry.addData("Curr Pos", linear_slide.getCurrentPosition());
        telemetry.update();
    }

    //start bluetop
    private void straight() {
        drive2(1325, 1325, 1325, 1325, 0.5);
        servo_left.setPosition(0.255);
        hold_claw_pos(1100, 0.8);
        drive2(810, 810, -810, -810, 0.5);
        PIVOT(840, 840, 840, 840, 0.5);
        sleep(52000);
    }

    /**
     * Describe this function...
     */
    private void right2() {
        drive2(1100, 1100, 1100, 1100, 0.3);
        drive2(-620, -620, 620, 620, 0.3);
        servo_left.setPosition(0.255);
        hold_claw_pos(1000, 0.7);
        drive2(820, 820, -820, -820, 0.5);
        PIVOT(820, 820, 820, 820, 0.5);
        drive2(100, 100, 100, 100, 0.3);
        sleep(20000);
    }

    private void left() {
        drive2(925, 925, 925, 925, 0.4);
        drive2(575, 575, -575, -575, 0.4);
        slide(500, 0.4);
        servo_left.setPosition(0.255);
        sleep(300);
        slide(500, -0.4);
        hold_claw_pos(1000, 0.7);
        PIVOT(825, 825, 825, 825, 0.4);
        sleep(200000);
    }

    private void PIVOT(
            int leftFTarget,
            int rightFTarget,
            int leftBTarget,
            int rightBTarget,
            double speed
    ) {
        motor0Pos = 0;
        motor1Pos = 0;
        motor2Pos = 0;
        motor3Pos = 0;
        motor0Pos += rightBTarget;
        motor1Pos += leftBTarget;
        motor2Pos += leftFTarget;
        motor3Pos += rightFTarget;
        motor_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // motor2 normal = reverse; so keeping multiplier to
        // "1" makes it go reverse which is what we need here
        //
        // RB=motor0 is (+); for pivot L we want this to go reverse hence "-1"
        motor_3.setTargetPosition(-1 * leftFTarget);
        motor_2.setTargetPosition(1 * rightBTarget);
        motor_1.setTargetPosition(1 * leftBTarget);
        motor_0.setTargetPosition(rightFTarget);
        motor_0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_2.setPower(speed);
        motor_0.setPower(speed);
        motor_1.setPower(speed);
        motor_3.setPower(speed);
        while (
                opModeIsActive() &&
                        motor_0.isBusy() &&
                        motor_3.isBusy() &&
                        motor_2.isBusy() &&
                        motor_1.isBusy()
        ) {
            idle();
        }
        sleep(100);
    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder VisionPortal_Builder;
        AprilTagProcessor myAprilTagProcessor;
        VisionPortal myVisionPortal;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("blue new cam");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("Pixel"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        VisionPortal_Builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            VisionPortal_Builder.setCamera(
                    hardwareMap.get(WebcamName.class, "Webcam")
            );
        } else {
            // Use the device's back camera.
            VisionPortal_Builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        VisionPortal_Builder.addProcessor(myTfodProcessor);
        VisionPortal_Builder.addProcessor(myAprilTagProcessor);
        // Enable the live camera preview.
        VisionPortal_Builder.enableLiveView(true);
        // Create a VisionPortal by calling build.
        myVisionPortal = VisionPortal_Builder.build();
    }

    private void telemetryTfod() {
        Recognition myTfodRecognition;
        float x;
        float y;

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData(
                "# Objects Detected",
                JavaUtil.listLength(myTfodRecognitions)
        );
        // Iterate through list and call a function to
        // display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("MY DATA");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData(
                    "Image",
                    myTfodRecognition.getLabel() +
                            " (" +
                            JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) +
                            " % Conf.)"
            );
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData(
                    "- Position",
                    JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0)
            );
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData(
                    "- Size",
                    JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) +
                            " x " +
                            JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0)
            );
            telemetry.addData(
                    "Angle_of_object",
                    myTfodRecognition.estimateAngleToObject(AngleUnit.DEGREES)
            );
            // This is to set a variable called "angle_of_object"
            // and set its value based on "estimateAngleToObject"
            ANGLE_OF_OBJECT =
                    myTfodRecognition.estimateAngleToObject(AngleUnit.DEGREES);
        }
    }

    private void drive2(int leftFTarget, int rightFTarget, int leftBTarget, int rightBTarget, double speed) {
        motor0Pos = 0;
        motor1Pos = 0;
        motor2Pos = 0;
        motor3Pos = 0;
        motor0Pos += rightBTarget;
        motor1Pos += leftBTarget;
        motor2Pos += leftFTarget;
        motor3Pos += rightFTarget;
        motor_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_0.setTargetPosition(1 * leftFTarget);
        motor_1.setTargetPosition(-1 * rightBTarget);
        motor_2.setTargetPosition(1 * leftBTarget);
        motor_3.setTargetPosition(1 * rightFTarget);
        motor_0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_2.setPower(speed);
        motor_0.setPower(speed);
        motor_1.setPower(speed);
        motor_3.setPower(speed);
        while (opModeIsActive() && motor_0.isBusy() && motor_3.isBusy() && motor_2.isBusy() && motor_1.isBusy()) {
            idle();
        }
    }


    //end bluetop

}
