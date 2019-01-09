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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Hello
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousCrater", group="Linear Opmode")
//@Disabled
public class AutoCrater extends LinearOpMode {

    HardwareRobot robot   = new HardwareRobot();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean leftArmFoundMineral = false;
    private boolean rightArmFoundMineral = false;

    //stop the arm if it gets to this point and hasn't found a mineral so it doesn't accidentally knock it
    private final double maxArmSensorPosition = 0.6;

    private double leftMineralSensorDistance;
    private double rightMineralSensorDistance;
    private double leftArmPosition = 0.0;
    private double rightArmPosition = 1.0;
    private double armSpeedIncrement = 0.04;
    private double blueLimit = 29;
    private double driveSpeed = 0.3;

    //colors
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float leftHSVValues[] = {0F, 0F, 0F};
    float centerHSVValues[] = {0F, 0F, 0F};
    float rightHSVValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float leftValues[] = leftHSVValues;
    final float centerValues[] = centerHSVValues;
    final float rightValues[] = rightHSVValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //rightArmPosition = rightArmPosition - 0.005;
            robot.leftSensorArm.setPosition(leftArmPosition);
            robot.rightSensorArm.setPosition(rightArmPosition);

            sleep(50);

            //raise elevator
            robot.elevatorMotor.setPower(-0.4);
            if (robot.elevatorLimitTop.getState()) {
                telemetry.addData("Elevator", "Lowering from hook");
            } else {

                robot.elevatorMotor.setPower(0.0);

                //give it a half second to make sure elevator has stopped
                sleep(200);

                // drive left to get off hook
                telemetry.addData("left",0);
                encoderDrive(driveSpeed, "left",3, 5);

                //drive forward to clear hook
                telemetry.addData("forward",0);
                encoderDrive(driveSpeed, "forward", 3, 5);

                // spin to correct
                telemetry.addData("counterClockwise",0);
                encoderDrive(driveSpeed, "counterClockwise",1, 5);


                //drive back to center
                telemetry.addData("right",0);
                encoderDrive(driveSpeed, "right", 2.5, 3);

                //move forward to block
                telemetry.addData("forward",0);
                encoderDrive(driveSpeed, "forward", 15.5, 3);

                leftArmPosition = 0.5;
                rightArmPosition = 0.5;

                while (opModeIsActive()
                        && (!leftArmFoundMineral || !rightArmFoundMineral || (leftMineralSensorDistance < maxArmSensorPosition && rightMineralSensorDistance < maxArmSensorPosition))) {
                    /* left arm sensor - starts at 0 fully extended is 1 */
                    leftMineralSensorDistance = robot.leftSensorArmDistance.getDistance(DistanceUnit.CM);
                    if (!Double.isNaN(leftMineralSensorDistance)) {
                        leftArmFoundMineral = true;
                    } else {

                        leftArmPosition = leftArmPosition + armSpeedIncrement;
                    }

                    /* right arm sensor - starts at 1 fully extended is 0 */
                    rightMineralSensorDistance = robot.rightSensorArmDistance.getDistance(DistanceUnit.CM);
                    if (!Double.isNaN(rightMineralSensorDistance)) {
                        rightArmFoundMineral = true;
                    } else {
                        rightArmPosition = rightArmPosition - armSpeedIncrement;
                    }

                    robot.leftSensorArm.setPosition(leftArmPosition);
                    robot.rightSensorArm.setPosition(rightArmPosition);
                    //add in a pause so the arms move slowly out
                    telemetry.addData("centerSensorDistance", robot.centerSensorDistance.getDistance(DistanceUnit.CM));
                    telemetry.addData("centerColor", robot.centerSensorColor.blue());
                    telemetry.addData("leftSensorDistance", robot.leftSensorArmDistance.getDistance(DistanceUnit.CM));
                    telemetry.addData("leftColor", robot.leftSensorArmColor.blue());
                    telemetry.addData("rightSensorDistance", robot.rightSensorArmDistance.getDistance(DistanceUnit.CM));
                    telemetry.addData("rightColor", robot.rightSensorArmColor.blue());
                    telemetry.update();
//                        telemetry.update();
                    sleep(300);
                    idle(); //We need to call the idle() method at the end of any looping we do to share the phone's processor with other processes on the phone.
                }
                if (leftMineralSensorDistance > 40) {
                    leftArmPosition = leftArmPosition + armSpeedIncrement;
                    robot.leftSensorArm.setPosition(leftArmPosition);
                }
                if (rightMineralSensorDistance > 40) {
                    rightArmPosition = rightArmPosition - armSpeedIncrement;
                    robot.rightSensorArm.setPosition(rightArmPosition);
                }
//                    sleep(1000);
                //we now have minerals in left and right - if one is gold knock it off if both are gold ignore

                //In the HSV model, S=Saturation = (Max(R,G,B) - Min(R,G,B)) / Max(R,G,B). As noted, for white, R, G, and B are about the same and S=0,
                //while for gold, B will be much less than the R and G values so S will be close to 1.
                //Using this simple calculation can be a good quick way to detect white vs. gold, assuming you are close enough to the mineral to get reasonable readings.
                //Using S also auto-normalizes for brightness

                //search for largest saturation = gold

                Color.RGBToHSV((int) (robot.leftSensorArmColor.red() * SCALE_FACTOR),
                        (int) (robot.leftSensorArmColor.green() * SCALE_FACTOR),
                        (int) (robot.leftSensorArmColor.blue() * SCALE_FACTOR),
                        leftHSVValues);
                Color.RGBToHSV((int) (robot.centerSensorColor.red() * SCALE_FACTOR),
                        (int) (robot.centerSensorColor.green() * SCALE_FACTOR),
                        (int) (robot.centerSensorColor.blue() * SCALE_FACTOR),
                        centerHSVValues);
                Color.RGBToHSV((int) (robot.rightSensorArmColor.red() * SCALE_FACTOR),
                        (int) (robot.rightSensorArmColor.green() * SCALE_FACTOR),
                        (int) (robot.rightSensorArmColor.blue() * SCALE_FACTOR),
                        rightHSVValues);

                //add in a pause so the arms move slowly out
                telemetry.addData("centerSensorDistance", robot.centerSensorDistance.getDistance(DistanceUnit.CM));
//                telemetry.addData("centerBlue", robot.centerSensorColor.blue());
                telemetry.addData("centerSaturation", centerHSVValues[1]);
                telemetry.addData("leftSensorDistance", robot.leftSensorArmDistance.getDistance(DistanceUnit.CM));
//                telemetry.addData("leftColor", robot.leftSensorArmColor.blue());
                telemetry.addData("leftSaturation", leftHSVValues[1]);
                telemetry.addData("rightSensorDistance", robot.rightSensorArmDistance.getDistance(DistanceUnit.CM));
//                telemetry.addData("rightColor", robot.rightSensorArmColor.blue());
                telemetry.addData("rightSaturation", rightHSVValues[1]);
                telemetry.update();
                sleep(300);


                if (leftHSVValues[1] > centerHSVValues[1] && leftHSVValues[1] > rightHSVValues[1]) {
                    robot.leftSensorArm.setPosition(1.0);
                    sleep(500);
                    robot.rightSensorArm.setPosition(0.8);
                    robot.leftSensorArm.setPosition(0.2);
                    sleep(300);
                    robot.rightSensorArm.setPosition(1.0);
                    robot.leftSensorArm.setPosition(0.0);
                } else if (rightHSVValues[1] > centerHSVValues[1] && rightHSVValues[1] > leftHSVValues[1]) {
                    robot.rightSensorArm.setPosition(0.0);
                    sleep(500);
                    robot.rightSensorArm.setPosition(0.8);
                    robot.leftSensorArm.setPosition(0.2);
                    sleep(300);
                    robot.rightSensorArm.setPosition(1.0);
                    robot.leftSensorArm.setPosition(0.0);
                } else {
                    //couldn't find gold - retract both and drive forward to hit center
                    robot.rightSensorArm.setPosition(0.8);
                    robot.leftSensorArm.setPosition(0.2);
                    sleep(300);
                    robot.rightSensorArm.setPosition(1.0);
                    robot.leftSensorArm.setPosition(0.0);
                    sleep(300);
                    encoderDrive(driveSpeed,"forward",5,3);
                    encoderDrive(driveSpeed,"backward",5,3);
                }


//                    if (leftArmFoundMineral && robot.leftSensorArmColor.blue() < 29) {
//                        robot.leftSensorArm.setPosition(1.0);
//                        sleep(500);
//                        robot.leftSensorArm.setPosition(0.0);
//                        robot.rightSensorArm.setPosition(1.0);
//                    } else if (rightArmFoundMineral && robot.rightSensorArmColor.blue() < 29) {
//                        robot.rightSensorArm.setPosition(0.0);
//                        sleep(500);
//                        robot.rightSensorArm.setPosition(1.0);
//                        robot.leftSensorArm.setPosition(0.0);
//                    } else {
//                        //couldn't find gold - retract both and drive forward to hit center
//                        robot.rightSensorArm.setPosition(1.0);
//                        robot.leftSensorArm.setPosition(0.0);
//                        encoderDrive(driveSpeed,"forward",3,2);
//                        encoderDrive(driveSpeed,"backward",3,2);
//                    }
//                }
                //move backwards so we don't hit minerals
                //encoderDrive(driveSpeed, "backward", 2, 2);
                /*
                 *
                 * CRATER SPECIFIC CODE
                 *
                 */
                //This is depot so extend arm to drop marker

//                while (opModeIsActive() && robot.armDriveMotor.isBusy()) {
//                    telemetry.addData("Raising Arm To Low Position", robot.armDriveMotor.getCurrentPosition());
////                    telemetry.update();
//                }


                //or maybe
                //robot.armDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                sleep(2000);
//                robot.armDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                encoderDrive(driveSpeed, "left",38, 5);
                encoderDrive(driveSpeed, "clockwise",44, 5);
                encoderDrive(driveSpeed, "forward",10, 2);


                robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armDriveMotor.setTargetPosition(200);
                robot.armDriveMotor.setPower(0.9);


                robot.extenderHexMotor.setPower(1.0);
                robot.extenderHexMotor.setTargetPosition(-1800);
//                    robot.extenderHexMotor.setPower(0.5);

//                while (opModeIsActive()) {
////                    robot.armDriveMotor.setPower(0.9);
////                    robot.armDriveMotor.setTargetPosition(400);
////                    robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    telemetry.addData("Extending Arm", robot.extenderHexMotor.getCurrentPosition());
//                    telemetry.update();
//                }

                //telemetry.addData("extending",0);

                sleep(4000);

//                robot.armDriveMotor.setTargetPosition(0);
//                robot.armDriveMotor.setPower(0.9);

                //eject marker
                telemetry.addData("ejecting",0);
                telemetry.update();
                robot.collectorHexMotor.setPower(-0.7);
                robot.armDriveMotor.setTargetPosition(10);
                sleep(1000);

                robot.collectorHexMotor.setPower(0.0);
//
                robot.armDriveMotor.setTargetPosition(1800);
                sleep(3000);

                encoderDrive(driveSpeed, "counterclockwise",54, 5);


//
//                //retract arm
//                robot.extenderHexMotor.setPower(1.0);
//                robot.extenderHexMotor.setTargetPosition(-200);
                sleep(2000);

                encoderDrive(driveSpeed, "forward",10, 5);

                robot.armDriveMotor.setPower(0.1);
                robot.armDriveMotor.setTargetPosition(10);


//                robot.armDriveMotor.setPower(0.4);
//                robot.armDriveMotor.setTargetPosition(300);
//                robot.extenderHexMotor.setPower(1.0);
//                robot.extenderHexMotor.setTargetPosition(-1500);
                sleep(20000);


//                robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                //drive to left to wall
//                encoderDrive(driveSpeed, "left", 40, 10);
//                //spin clockwise 5 inches
//                encoderDrive(driveSpeed, "clockwise", 5, 5);
//                //drive backwards to crater
//                encoderDrive(driveSpeed, "backward", 60, 10);
//                sleep(28000);
            }
//            telemetry.update();
            idle(); //We need to call the idle() method at the end of any looping we do to share the phone's processor with other processes on the phone.
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             String direction,
                             double inches,
                             double timeoutS) {
        //drive encoder variables
        final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Neverest Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        int newLeftBackTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;

        double lowerXSpeed = 0.625;
        double lowerYSpeed = 0.4545;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //NOTE - leftFront/rightBack move together and leftBack/rightFront move together

            switch (direction) {
                case "forward": {
                    // Determine new target position, and pass to motor controller
                    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    break;
                }
                case "backward": {
                    // Determine new target position, and pass to motor controller
                    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() - (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int)((inches * lowerYSpeed) * COUNTS_PER_INCH);
                    break;
                }
                case "left": {
                    // Determine new target position, and pass to motor controller
                    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    break;
                }
                case "right": {
                    // Determine new target position, and pass to motor controller
                    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    break;
                }
                case "clockwise": {
                    // Determine new target position, and pass to motor controller
                    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightBackTarget = robot.rightBackDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    break;
                }
                case "counterClockwise": {
                    // Determine new target position, and pass to motor controller
                    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)((inches * lowerXSpeed) * COUNTS_PER_INCH);
                    break;
                }
                default: {
                    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition();
                    newRightBackTarget = robot.rightBackDrive.getCurrentPosition();
                    newLeftBackTarget = robot.leftBackDrive.getCurrentPosition();
                    newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition();
                    break;
                }
            }


            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and all motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightBackDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightFrontDrive.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Target",  "Running to %7d :%7d :%7d :%7d",
                        newLeftFrontTarget,
                        newRightBackTarget,
                        newLeftBackTarget,
                        newRightFrontTarget);
                telemetry.addData("Current",  "Running at %7d :%7d :%7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition(),
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition()
                );
                telemetry.update();
                idle(); //We need to call the idle() method at the end of any looping we do to share the phone's processor with other processes on the phone.
            }

            // Stop all motion;
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(250);   // optional pause after each move
        }
    }
}
