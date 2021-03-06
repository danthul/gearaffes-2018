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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Tele", group="Iterative Opmode")

public class Tele extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HardwareRobot robot = new HardwareRobot();
    double elevatorPower;
    double elevatorMaxPower = 1.0;
    boolean elevatorXPressed = false;
    double maxSpeed = 0.6;
    double armDrivePower = 0;
    int armExtendStopPosition;
    int armStopPosition;
    boolean isExtenderBusy = false;
    boolean isArmBusy = false;
    int armDriveMaxExtension = -6400;
    int armExtendYStopPosition = -4000;


    double normalize(double number) {
        if (number > 1) {
            return 1;
        } else if (number < -1) {
            return -1;
        } else {
            return number;
        }
    }



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        robot.armDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armStopPosition = robot.armDriveMotor.getCurrentPosition();
        robot.armDriveMotor.setTargetPosition(armStopPosition);
        robot.armDriveMotor.setPower(0.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /* Drive train **/
        if (gamepad1.right_trigger > 0 || gamepad2.right_bumper) {
            maxSpeed = 0.2;
        } else {
            maxSpeed = 0.6;
        }
        robot.leftFrontDrive.setPower(normalize(-gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * maxSpeed);
        robot.rightFrontDrive.setPower(normalize(-gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * maxSpeed);
        robot.leftBackDrive.setPower(normalize(-gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * maxSpeed);
        robot.rightBackDrive.setPower(normalize(-gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * maxSpeed);


        /* Elevator section **/
        if(gamepad1.y && robot.elevatorLimitTop.getState()) {
            //if top sensor isn't hit and y is pressed, move elevator up with power
            elevatorPower = -elevatorMaxPower;
            elevatorXPressed = false;
        } else if (gamepad1.a && robot.elevatorLimitBottom.getState()) {
            //if bottom sensor isn't hit and a is pressed, move elevator down with power
            elevatorPower = elevatorMaxPower;
            elevatorXPressed = false;
        } else if (gamepad1.x && robot.elevatorLimitTop.getState()) {
            //if top sensor isn't hit and x is pressed, move elevator up to position
           elevatorPower = -elevatorMaxPower;
           elevatorXPressed = true;
        } else {
            if (!elevatorXPressed) {
                elevatorPower = 0;
            }
        }
        //reset elevatorXPressed if top limit is reached
        if(!robot.elevatorLimitTop.getState() && elevatorXPressed) {
            elevatorXPressed = false;
        }
        robot.elevatorMotor.setPower(elevatorPower);

        // range between 0 and -1500

        if (gamepad2.right_stick_y > 0) {
            armStopPosition = robot.armDriveMotor.getCurrentPosition();
          //lowering arm so lower power
            robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armDrivePower = -0.25;
        } else if (gamepad2.right_stick_y < 0){
            if (robot.armDriveMotor.getCurrentPosition() < 1900) {
                armStopPosition = robot.armDriveMotor.getCurrentPosition();
                robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armDrivePower = -gamepad2.right_stick_y * 0.3;
            } else {
                robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armDriveMotor.setTargetPosition(armStopPosition);
                armDrivePower = 1.0;
            }
        } else {
            robot.armDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armDriveMotor.setTargetPosition(armStopPosition);
            armDrivePower = 0.25;
        }

        robot.armDriveMotor.setPower(armDrivePower);

        //if the stick is moved after y is pressed control will be given back to the driver

        if (robot.extenderHexMotor.getCurrentPosition() > 0) {
            robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extenderHexMotor.setPower(1.0);
            robot.extenderHexMotor.setTargetPosition(0);
        } else if (robot.extenderHexMotor.getCurrentPosition() < armDriveMaxExtension) {
                robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extenderHexMotor.setPower(1.0);
                robot.extenderHexMotor.setTargetPosition(armDriveMaxExtension);
        } else if (gamepad2.left_stick_y != 0) {
            robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (robot.extenderHexMotor.getCurrentPosition() < 0 || gamepad2.left_stick_y < 0) {
                robot.extenderHexMotor.setPower(normalize(gamepad2.left_stick_y));
            }
            armExtendStopPosition = robot.extenderHexMotor.getCurrentPosition();
            //if y is pressed arm will extend to 1000
        } else if (gamepad2.y ){
            armExtendStopPosition = armExtendYStopPosition;
            robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extenderHexMotor.setPower(0.1);
            robot.extenderHexMotor.setTargetPosition(armExtendStopPosition);

            //set arm stop position to drive location as arm is being extended/retracted on pressing Y
            if (gamepad2.right_stick_y == 0) {
                armStopPosition = 1150;
            }
        } else {
            //if (!robot.extenderHexMotor.isBusy()) {
            //    robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //    robot.extenderHexMotor.setPower(0);
            //}
            robot.extenderHexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extenderHexMotor.setPower(1.0);
            robot.extenderHexMotor.setTargetPosition(armExtendStopPosition);
        }

        /* Arm Drive Motor **/
        //limit down to 300
        //limit up to 2400

        /* Arm collector **/
       if (gamepad2.left_trigger > 0){
           robot.collectorHexMotor.setPower(1.0);
       }
        else if (gamepad2.right_trigger > 0){
            robot.collectorHexMotor.setPower(-1.0);
        } else {
           robot.collectorHexMotor.setPower(0.0);
       }

        /* Telemetry Data **/
        // Show the elapsed game time and wheel power.
        telemetry.addData("armDriveMotor Current Position", robot.armDriveMotor.getCurrentPosition());
        telemetry.addData("armDriveMotor Target Position", robot.armDriveMotor.getTargetPosition());
        telemetry.addData("extenderHexMotor Current Position", robot.extenderHexMotor.getCurrentPosition());
        telemetry.addData("extenderHexMotor Target Position", robot.extenderHexMotor.getTargetPosition());
        telemetry.addData("extender is busy", isExtenderBusy);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}