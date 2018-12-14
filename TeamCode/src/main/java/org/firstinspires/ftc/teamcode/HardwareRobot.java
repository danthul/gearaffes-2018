package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareRobot
{
    // Declare OpMode members.
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor elevatorMotor;
    DcMotor armDriveMotor;
    DcMotor extenderHexMotor;
    DcMotor collectorHexMotor;
    DigitalChannel elevatorLimitBottom;
    DigitalChannel elevatorLimitTop;
    DistanceSensor sensorRangeLeft;
    DistanceSensor sensorRangeRight;
    DigitalChannel armLimitBottom;
    Servo leftSensorArm;
    Servo rightSensorArm;
    ColorSensor leftSensorArmColor;
    DistanceSensor leftSensorArmDistance;
    ColorSensor rightSensorArmColor;
    DistanceSensor rightSensorArmDistance;
    ColorSensor centerSensorColor;
    DistanceSensor centerSensorDistance;


    /* Local OpMode members. */
    private HardwareMap hwMap  = null;

    /* Constructor */
    HardwareRobot() {
    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // ************** Drive Train ************************* //
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        // ************** Back Sensors ************************* //
//        sensorRangeLeft = hwMap.get(DistanceSensor.class, "left_distance_sensor");
//        sensorRangeRight = hwMap.get(DistanceSensor.class, "right_distance_sensor");

        // ************** Elevator ************************* //
        // motor
        elevatorMotor = hwMap.get(DcMotor.class, "elevator_motor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        //sensors
        elevatorLimitBottom = hwMap.get(DigitalChannel.class, "elevator_limit_bottom");
        elevatorLimitTop = hwMap.get(DigitalChannel.class, "elevator_limit_top");
        elevatorLimitBottom.setMode(DigitalChannel.Mode.INPUT);
        elevatorLimitTop.setMode(DigitalChannel.Mode.INPUT);

        // ************** Collector Arm ************************* //
        //arm motor
        armDriveMotor = hwMap.get(DcMotor.class, "arm_drive_motor");
        armDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //armDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //arm limit switch
//        armLimitBottom = hwMap.get(DigitalChannel.class, "arm_limit_bottom");
//        armLimitBottom.setMode(DigitalChannel.Mode.INPUT);

        // extender
        extenderHexMotor = hwMap.get(DcMotor.class, "extender_hex_motor");
        extenderHexMotor.setDirection(DcMotor.Direction.FORWARD);
        extenderHexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extenderHexMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extenderHexMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //collector
        collectorHexMotor = hwMap.get(DcMotor.class, "collector_hex_motor");
        collectorHexMotor.setDirection(DcMotor.Direction.FORWARD);
        collectorHexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        collectorHexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ************** Sensor Arms ************************* //
        // servos
        leftSensorArm = hwMap.get(Servo.class, "left_arm_servo");
        rightSensorArm = hwMap.get(Servo.class, "right_arm_servo");

        //sensors
        centerSensorColor = hwMap.get(ColorSensor.class, "center_sensor");
        centerSensorDistance = hwMap.get(DistanceSensor.class, "center_sensor");
        leftSensorArmColor = hwMap.get(ColorSensor.class, "left_arm_sensor");
        leftSensorArmDistance = hwMap.get(DistanceSensor.class, "left_arm_sensor");
        rightSensorArmColor = hwMap.get(ColorSensor.class, "right_arm_sensor");
        rightSensorArmDistance = hwMap.get(DistanceSensor.class, "right_arm_sensor");
    }
}
