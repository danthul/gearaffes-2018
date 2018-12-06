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

@Autonomous(name="AutonomousDepot", group="Linear Opmode")
//@Disabled
public class AutoDepot extends LinearOpMode {

    HardwareRobot robot   = new HardwareRobot();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    boolean leftArmFoundMineral = false;
    boolean centerFoundMineral = false;
    boolean rightArmFoundMineral = false;

    String leftMineralType;
    String centerMineralType;
    String rightMineralType;

    //the distance where the robot should stop and make the call whether gold or white
    final double samplingDistance = 10;
    //stop the arm if it gets to this point and hasn't found a mineral so it doesn't accidentally knock it
    final double maxArmSensorPosition = 0.6;

    double leftMineralSensorDistance;
    double centerMineralSensorDistance;
    double rightMineralSensorDistance;
    double leftArmPosition = 0.0;
    double rightArmPosition = 1.0;
    double armSpeedIncrement = 0.05;
    double blueLimit = 29;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        robot.leftSensorArm.setPosition(leftArmPosition);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            rightArmPosition = rightArmPosition - 0.005;
            robot.leftSensorArm.setPosition(leftArmPosition);
            robot.rightSensorArm.setPosition(rightArmPosition);

            telemetry.addData("left servo pos",  robot.leftSensorArm.getPosition());
            telemetry.addData("right servo pos",  robot.rightSensorArm.getPosition());
            telemetry.addData("left sensor distance", leftMineralSensorDistance);
            telemetry.update();
            sleep(50);

            //raise elevator
            robot.elevatorMotor.setPower(-0.4);
            if (robot.elevatorLimitTop.getState()) {
                telemetry.addData("Elevator", "Lowering from hook");
            } else {
                telemetry.addData("Elevator", "Reached the ground");
                robot.elevatorMotor.setPower(0.0);

                //give it a half second to make sure elevator has stopped
                sleep(500);

                //drive to left to get off hook
                encoderDrive(0.5, 6, 0, 6);

                //drive forwards to get to blocks
                encoderDrive(0.5, 10, 10, 6);

                //drive to right to recenter
                encoderDrive(0.5, 0, 3, 6);

                //drive forward until we reach mineral
                // TODO

                //sample center block
                if (robot.centerSensorDistance.getDistance(DistanceUnit.CM) < samplingDistance) {
                    //TODO - if too close back up

                    centerFoundMineral = true;
                    if (robot.centerSensorColor.blue() < blueLimit) {
                        centerMineralType = "gold";
                        // drive forward to knock off middle mineral
                        //TODO 10 inches?
                        encoderDrive(0.5, 10, 10, 6);
                        encoderDrive(0.5, -10, -10, 6);
                    } else {
                        centerMineralType = "white";
                        //if not gold extend sensor arms
                        //loop until we find minerals or arms hit their maximums
                        while (opModeIsActive() && (!leftArmFoundMineral || !rightArmFoundMineral || (leftMineralSensorDistance < maxArmSensorPosition && rightMineralSensorDistance < maxArmSensorPosition))) {
                            /* left arm sensor - starts at 0 fully extended is 1 */
                            leftMineralSensorDistance = robot.leftSensorArmDistance.getDistance(DistanceUnit.CM);

                            if ((leftMineralSensorDistance < samplingDistance) && !leftArmFoundMineral && leftMineralSensorDistance < maxArmSensorPosition) {
                                leftArmFoundMineral = true;
                                if (robot.leftSensorArmColor.blue() < blueLimit) {
                                    leftMineralType = "gold";
                                }
                            } else {
                                leftArmPosition = leftArmPosition + armSpeedIncrement;
                            }

                            /* right arm sensor - starts at 1 fully extended is 0 */
                            rightMineralSensorDistance = robot.rightSensorArmDistance.getDistance(DistanceUnit.CM);

                            if ((rightMineralSensorDistance < samplingDistance) && !rightArmFoundMineral && rightMineralSensorDistance < maxArmSensorPosition) {
                                rightArmFoundMineral = true;
                                if (robot.rightSensorArmColor.blue() < blueLimit) {
                                    rightMineralType = "gold";
                                }
                            } else {
                                rightArmPosition = rightArmPosition + armSpeedIncrement;
                            }
                            robot.leftSensorArm.setPosition(leftArmPosition);
                            robot.rightSensorArm.setPosition(rightArmPosition);
                            idle(); //We need to call the idle() method at the end of any looping we do to share the phone's processor with other processes on the phone.
                        }
                        //we now have minerals in left and white - if one is gold knock it off if both are gold ignore
                        if (leftArmFoundMineral && leftMineralType.equals("gold") && !rightMineralType.equals("gold")) {
                            robot.leftSensorArm.setPosition(1.0);
                            sleep(500);
                            robot.leftSensorArm.setPosition(0.0);
                        }
                        //we now have minerals in left and white - if one is gold knock it off
                        if (rightArmFoundMineral && rightMineralType.equals("gold") && !leftMineralType.equals("gold")) {
                            robot.rightSensorArm.setPosition(0.0);
                            sleep(500);
                            robot.rightSensorArm.setPosition(1.0);
                        }
                    }
                }

                //TODO recenter?
                //TODO drive to drop off marker
                //TODO drive back to crater

                sleep(28000);


            }
            telemetry.update();
            idle(); //We need to call the idle() method at the end of any looping we do to share the phone's processor with other processes on the phone.
        }
    }

    /*
     * set bottom hex after button press
     */
    protected void driveWithTimer(String direction, double power, long duration) {
        drive(direction, power);
        sleep(duration);
    }

    protected void drive(String direction, double power) {
        if (direction.equals("forward")) {
            robot.leftFrontDrive.setPower(-power);
            robot.rightBackDrive.setPower(-power);
            robot.rightFrontDrive.setPower(-power);
            robot.leftBackDrive.setPower(-power);
        } else if (direction.equals("backward")) {
            robot.leftFrontDrive.setPower(power);
            robot.rightBackDrive.setPower(power);
            robot.rightFrontDrive.setPower(power);
            robot.leftBackDrive.setPower(power);
        } else if (direction.equals("left")) {
            robot.leftFrontDrive.setPower(power);
            robot.rightBackDrive.setPower(power);
            robot.rightFrontDrive.setPower(-power);
            robot.leftBackDrive.setPower(-power);
        } else if (direction.equals("right")) {
            robot.leftFrontDrive.setPower(-power);
            robot.rightBackDrive.setPower(-power);
            robot.rightFrontDrive.setPower(power);
            robot.leftBackDrive.setPower(power);
        } else { //direction could equal stop (or anything else)
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
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
                             double leftInches, double rightInches,
                             double timeoutS) {
        //drive encoder variables
        final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
        final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
        final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                          (WHEEL_DIAMETER_INCHES * 3.1415);
        int newLeftBackTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            //NOTE - leftFront/rightBack move together and leftBack/rightFront move together

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);
            robot.leftFrontDrive.setTargetPosition(newLeftBackTarget);
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

            sleep(250);   // optional pause after each move
        }
    }
}
