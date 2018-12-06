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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


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

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // int initialPosition = bottomHexMotor.getCurrentPosition();
        telemetry.addData("Status", "Initialized");

        // telemetry.addData("Initial Position: ", initialPosition);
        telemetry.update();

        //distance sensors
        // sensorRangeLeft = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        // sensorRangeRight = hardwareMap.get(DistanceSensor.class, "right_distance_sensor");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            double averageDistance;

            //  bottomHexMotor.setPower(0.1);
            //  bottomHexMotor.setTargetPosition(100);

//            telemetry.addData("range_left", String.format("%.01f mm", sensorRangeLeft.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range_right", String.format("%.01f mm", sensorRangeRight.getDistance(DistanceUnit.MM)));
//            telemetry.addData("Current Position: ", bottomHexMotor.getCurrentPosition());
            //raise elevator
            robot.elevatorMotor.setPower(-0.2);
            if (robot.elevatorLimitTop.getState() == true) {
                telemetry.addData("elevatortop", "is Not Pressed");
            } else {
                telemetry.addData("elevatortop", "is Pressed");
                robot.elevatorMotor.setPower(0.0);

                sleep(500);

                //drive forwards


                //drive to left
                drive("left",0.3,700);

                //drive forwards
                drive("forward", 0.3, 3000);

                //stop
                drive("forward", 0, 4000);

//                bottomHexMotor.setTargetPosition(40);
//                bottomHexMotor.setPower(1.0);
//                sleep(4000);
//                bottomHexMotor.setTargetPosition(0);
//                bottomHexMotor.setPower(0.8);
//                sleep(4000);
//                bottomHexMotor.setPower(0.0);

                //drive backwards
                drive("backward", 0.3, 300);

                //stop
                drive("forward", 0, 4000);
                sleep(28000);


            }

            telemetry.update();

            idle(); //We need to call the idle() method at the end of any looping we do to share the phone's processor with other processes on the phone.
        }
    }

    /*
     * set bottom hex after button press
     */
    protected void drive(String direction, double power, long duration){
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
        } else {
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
        }
        sleep(duration);
    }
}
