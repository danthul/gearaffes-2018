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
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@TeleOp(name="testfordrivetrain", group="Iterative Opmode")

public class testfordrivetrain extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareRobot robot   = new HardwareRobot();
    private int hexMotorCPR = 288; //rev hex core motor has 288 counts per revolution

    //local variables
    double hexPower = 0;
    double maxHexPower = 2.0;
    double elevatorLowPower = 0.4;
    double elevatorHighPower = 1.0;
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
        double elevatorPower = 0;

        double maxSpeed = 0.5;

        /** Drive train **/
        // Setup a variable for each drive wheel to save power level for telemetry
        robot.leftFrontDrive.setPower(normalize(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * maxSpeed);
        robot.rightFrontDrive.setPower(normalize(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * maxSpeed);
        robot.leftBackDrive.setPower(normalize(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * maxSpeed);
        robot.rightBackDrive.setPower(normalize(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * maxSpeed);


        /** Elevator section **/
        if(gamepad1.y && robot.elevatorLimitTop.getState() == true ) {
            //if top sensor isn't hit and y is pressed, move elevator up at low power
            elevatorPower = -elevatorLowPower;
        } else if (gamepad1.a && robot.elevatorLimitBottom.getState() == true) {
            //if bottom sensor isn't hit and a is pressed, move elevator down at low power
            elevatorPower = elevatorLowPower;
        } else if (gamepad1.x && robot.elevatorLimitBottom.getState() == true) {
            //if bottom sensor isn't hit and x is pressed, move elevator down at high power
            elevatorPower = elevatorHighPower;
        } else {
            elevatorPower = 0;
        }

        robot.elevatorMotor.setPower(elevatorPower);

        /** Arm Hex Motors **/


        /** Arm servos - collectors **/


        /** Telemetry Data **/
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Hex Position",  "Hex Position: " + bottomHexMotor.getCurrentPosition());

        //   telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        telemetry.addData("leftBackDrive Position", "Position: " + robot.leftBackDrive.getCurrentPosition());
        telemetry.addData("rightBackDrive Position", "Position: " + robot.rightBackDrive.getCurrentPosition());
        telemetry.addData("leftFrontDrive Position", "Position: " + robot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("rightFrontDrive Position", "Position: " + robot.rightFrontDrive.getCurrentPosition());

//        telemetry.addData("range_left", String.format("%.01f mm", sensorRangeLeft.getDistance(DistanceUnit.MM)));
//        telemetry.addData("range_right", String.format("%.01f mm", sensorRangeRight.getDistance(DistanceUnit.MM)));

//        int position = bottomHexMotor.getCurrentPosition();
//        if (robot.elevatorLimitTop.getState() == true) {
//            telemetry.addData("elevatortop", "is Not Pressed");
//        } else {
//            telemetry.addData("elevatortop", "is Pressed");
//        }
//        if (robot.elevatorLimitBottom.getState() == true) {
//            telemetry.addData("elevatorbottom", "is Not Pressed");
//        } else {
//            telemetry.addData("elevatorbottom", "is Pressed");
//        }
        telemetry.update();
      }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * set bottom hex after button press
     */
//    protected void bottomHexPosition(int position){
//        bottomHexMotor.setTargetPosition(position);
//        if (bottomHexMotor.getCurrentPosition() > bottomHexMotor.getTargetPosition()) {
//            hexPower = -maxHexPower;
//        } else {
//            hexPower = maxHexPower;
//        }
//        bottomHexMotor.setPower(hexPower);
//    }

}
