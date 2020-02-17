package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBionicbot;

@Autonomous(name="RedStoneSide", group="BionicBot")
public class RevampedRedColorSensing extends LinearOpMode {

HardwareBionicbot robot   = new HardwareBionicbot();   // Use a Pushbot's hardware
private ElapsedTime runtime = new ElapsedTime();

static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
static final double     CIRCUMFERENCE           = WHEEL_DIAMETER_INCHES * Math.PI;
static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
        (CIRCUMFERENCE);
static final double     DRIVE_SPEED             = 0.6;
static final double     TURN_SPEED              = 0.5;
@Override
public void runOpMode() {

    robot.init(hardwareMap);

    robot.BackColorSensor.enableLed(true);
    robot.FrontColorSensor.enableLed(true);

    telemetry.addData("Right Color Sensor is ", "Turned On");
    telemetry.addData("Left Color Sensor is ", "Turned On");

    waitForStart();

    while(opModeIsActive())
    {
        //Drive Forward Certain distance to scan Stones and Skystones
        DriveForwardDistance(.5,30);
        telemetry.addData("Color Number of Left Color Sensor: ", robot.FrontColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)));
        telemetry.addData("Color Number of Right Color Sensor: ", robot.BackColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)));
        telemetry.update();
        sleep(1000);
        //Use the Left Color Sensor to Scan for a SkyStone
        //Checks if the Object is a Stone
        if((robot.FrontColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 8 || robot.FrontColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 9) && (robot.BackColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 8 || robot.BackColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 9))
        {
            //Since the object is a Stone the Robot Strafes to scan the next Stone
            DriveBackwardDistance(.3,5);
            sleep(1000);
            StrafLeftDistance(1,12);
            DriveForwardDistance(.3,8);
            telemetry.addData("Color Number of Left Color Sensor: ", robot.FrontColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)));
            telemetry.addData("Color Number of Right Color Sensor: ", robot.BackColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)));
            telemetry.update();
            sleep(1000);
            //After Strafing the Robot scans again to check what the next object is
            if((robot.FrontColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 8 || robot.FrontColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 9) && (robot.BackColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 8 || robot.BackColorSensor.readUnsignedByte((ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)) == 9) )
            {
                //This is SkyStone Location number 3 (SkyStone Stone Stone)
                //Since the next object is also a Stone the robot strafes to the next object
                DriveBackwardDistance(1,8);
                sleep(500);
                StrafLeftDistance(1,7);
                //After the first two scanned objects are Stones the robot gets the third one because it is a SkyStone
                DriveForwardDistance(1,9);
                //Drops the latch attachment to get the Skystone
                robot.Leftglock.setPosition(.4);
                sleep(1500);
                //After getting the Skystone the Robot delivers is to the building zone
                //Starts by driving backwards with the SkyStone
                DriveBackwardDistance(1,9);
                //Turns Left to Face the Building Zone
                TurnRightDistance(1,23);
                //Drives Forward to deliver Skystone into the building zone
                DriveForwardDistance(1,47);
                //Releases the SkyStone by bringing latch back up
                robot.Leftglock.setPosition(.01);
                //sleep(1000);
                //Drive Backward to the 2nd SkyStone which is located in the 3rd location closest to the wall
                DriveBackwardDistance(1,67);
                //Turns Right so that the robot can face the SkyStone
                TurnLeftDistance(1,23);
                //Strafes a little bit to be fully aligned with the SkyStone
                StrafLeftDistance(1,8);
                //Drives Forward and picks up SkyStone
                DriveForwardDistance(1,17);
                sleep(1000);
                robot.Leftglock.setPosition(.4);


                //Drives Backward with the SkyStone
                DriveBackwardDistance(1,15);
                //Strafes a little with the SkyStone to prevent hitting the wall on the turn
                StrafRightDistance(1,5);
                //Turns Left to face the robot towards the building zone
                TurnRightDistance(1,22);
                //Drive Forward to deliver the SkyStone to the building zone
                DriveForwardDistance(1,70);
                //Releases the SkyStone by bringing the Latch up
                robot.Leftglock.setPosition(.01);
                sleep(1000);
                //Drives Backward to park under the Alliance Bridge
                DriveBackwardDistance(1,15);
                //Waits till the Autonomous time runs out
                sleep(10000);
            }
            else
            {
                //This is SkyStone Location number 2 (Stone SkyStone Stone)
                //After scanning the 2nd Location the color sensors says it isn't a regular stone so it is SkyStone

                StrafRightDistance(1,4);
                DriveForwardDistance(.3,5);
                //Drops the latch attachment to get the Skystone
                robot.Leftglock.setPosition(.4);
                sleep(1500);
                //After getting the Skystone the Robot delivers is to the building zone
                //Starts by driving backwards with the SkyStone
                DriveBackwardDistance(1,11);
                //Turns Left to Face the Building Zone
                TurnRightDistance(1,22);
                //Drives Forward to deliver Skystone into the building zone
                DriveForwardDistance(1,45);
                //Releases the SkyStone by bringing latch back up
                robot.Leftglock.setPosition(.01);
                //Drive Backward to the 2nd SkyStone which is located in the 2nd location in the middle of the group of 3
                DriveBackwardDistance(1,70);
                //Turns Right so that the robot can face the SkyStone
                TurnLeftDistance(1,22);
                //Strafes a little bit to be fully aligned with the SkyStone
                StrafRightDistance(.3,3);
                //Drives Forward and picks up SkyStone
                DriveForwardDistance(1,13);
                sleep(500);
                robot.Leftglock.setPosition(.4);
                sleep(500);
                //Drives Backward with the SkyStone
                DriveBackwardDistance(1,15);
                //Strafes Left to give robot space from wall
                StrafRightDistance(1,5);
                //Turns Left to face the robot towards the building zone
                TurnRightDistance(1,21);
                //Drive Forward to deliver the SkyStone to the building zone
                DriveForwardDistance(1,66);
                //Releases the SkyStone by bringing the Latch up
                robot.Leftglock.setPosition(.01);
                sleep(500);
                //Drives Backward to park under the Alliance Bridge
                DriveBackwardDistance(1,15);
                //Waits till the Autonomous time runs out
                StrafLeftDistance(1,10);
                sleep(10000);
            }

        }
        //If the first object Scanned is not a regular Stone
        else
        {
            //This is SkyStone Location number 1 (SkyStone Stone Stone)
            //After scanning the 1st Location the color sensors says it isn't a regular stone so it is SkyStone
            StrafRightDistance(1,3);
            DriveForwardDistance(.3,5);
            //Drops the latch attachment to get the Skystone
            robot.Leftglock.setPosition(.4);
            sleep(2000);
            //After getting the Skystone the Robot delivers is to the building zone
            //Starts by driving backwards with the SkyStone
            DriveBackwardDistance(1,12);
            //Turns Left to Face the Building Zone
            TurnRightDistance(1,22);
            //Drives Forward to deliver Skystone into the building zone
            DriveForwardDistance(1,36);
            //Releases the SkyStone by bringing latch back up
            robot.Leftglock.setPosition(.01);
            sleep(1000);
            //Drive Backward to the 2nd SkyStone which is located in the 1st location in the left most of the group of 3
            DriveBackwardDistance(1,64);
            //Turns Right so that the robot can face the SkyStone
            TurnLeftDistance(1,22);
            //Strafes a little bit to be fully aligned with the SkyStone
            StrafRightDistance(1,6);
            //Drives Forward and picks up SkyStone
            DriveForwardDistance(1,10);
            sleep(1000);
            robot.Leftglock.setPosition(.4);
            sleep(500);
            //Drives Backward with the SkyStone
            DriveBackwardDistance(1,15);
            //Turns Left to face the robot towards the buil ding zone
            TurnRightDistance(1,21);
            //Drive Forward to deliver the SkyStone to the building zone
            DriveForwardDistance(1,65);
            //Releases the SkyStone by bringing the Latch up
            robot.Leftglock.setPosition(.01);
            //Drives Backward to park under the Alliance Bridge
            DriveBackwardDistance(1,15);
            StrafLeftDistance(1,15);
            //Waits till the Autonomous time runs out
            sleep(10000);



        }
    }
}
public void DriveForwardDistance(double speed, double distanceInches)
{
    double rotationsneeded = distanceInches/CIRCUMFERENCE;
    int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

    robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
    robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);

    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    DriveForward(-speed);

    while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
    {

    }
    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
public void DriveBackwardDistance(double speed, double distanceInches)
{
    double rotationsneeded = distanceInches/CIRCUMFERENCE;
    int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
    robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);
    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);
    robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);

    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    DriveBackward(-speed);

    while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
    {

    }
    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
public void TurnLeftDistance(double speed, double distanceInches)
{
    double rotationsneeded = distanceInches/CIRCUMFERENCE;
    int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
    robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);
    robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);

    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    TurnLeft(speed);


    while( robot.rightDrive.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy() && robot.leftDrive.isBusy() )
    {

    }
    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
public void TurnRightDistance(double speed, int distanceInches)
{
    double rotationsneeded = distanceInches/CIRCUMFERENCE;
    int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
    robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);
    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
    robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);

    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    TurnRight(speed);


    while(robot.leftDrive.isBusy()  && robot.leftBack.isBusy() && robot.rightDrive.isBusy()  && robot.rightBack.isBusy() )
    {

    }
    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
public void StrafLeftDistance(double speed, int distanceInches)
{
    double rotationsneeded = distanceInches/CIRCUMFERENCE;
    int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);
    robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);
    robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);


    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    StrafLeft(speed);

    while(robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftDrive.isBusy()) {

    }

    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
public void StrafRightDistance(double speed, int distanceInches)
{
    double rotationsneeded = distanceInches/CIRCUMFERENCE;
    int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

    robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);
    robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);
    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);

    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    StrafRight(speed);


    while(robot.leftDrive.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy() && robot.rightDrive.isBusy())
    {

    }
    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
public void DriveForward(double power)
{
    robot.leftDrive.setPower(power);
    robot.rightDrive.setPower(power);
    robot.rightBack.setPower(power);
    robot.leftBack.setPower(power);
}
public void DriveBackward(double power)
{
    robot.leftDrive.setPower(-power);
    robot.rightDrive.setPower(-power);
    robot.rightBack.setPower(-power);
    robot.leftBack.setPower(-power);
}
public void TurnLeft(double power)
{
    robot.leftDrive.setPower(-power);
    robot.rightDrive.setPower(power);
    robot.rightBack.setPower(power);
    robot.leftBack.setPower(-power);
}
public void TurnRight(double power)
{
    robot.leftDrive.setPower(power);
    robot.rightDrive.setPower(-power);
    robot.rightBack.setPower(-power);
    robot.leftBack.setPower(power);
}
public void StrafLeft(double power)
{
    robot.leftDrive.setPower(-power);
    robot.rightDrive.setPower(power);
    robot.leftBack.setPower(power);
    robot.rightBack.setPower(-power);
}
public void StrafRight(double power)
{
    robot.leftDrive.setPower(power);
    robot.rightDrive.setPower(-power);
    robot.leftBack.setPower(-power);
    robot.rightBack.setPower(power);
}

public void StopDriving()
{
    robot.leftDrive.setPower(0);
    robot.rightDrive.setPower(0);
    robot.leftBack.setPower(0);
    robot.rightBack.setPower(0);
}
}