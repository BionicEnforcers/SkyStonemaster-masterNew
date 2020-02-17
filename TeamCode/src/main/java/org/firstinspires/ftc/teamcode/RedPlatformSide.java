package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Of Red Platform Side", group="BionicBot")

public class RedPlatformSide extends LinearOpMode
{
HardwareBionicbot         robot   = new HardwareBionicbot();   // Use a Pushbot's hardware
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
public void runOpMode()
{
    robot.init(hardwareMap);

    // Send telemetry message to signify robot waiting;
    telemetry.addData("Status", "Resetting Encoders");    //
    telemetry.update();

       /* robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    telemetry.addData("Path0",  "Starting at %7d :%7d",
            robot.leftDrive.getCurrentPosition(),
            robot.rightDrive.getCurrentPosition(),
            robot.leftBack.getCurrentPosition(),
            robot.rightBack.getCurrentPosition());
    telemetry.update();

    waitForStart();

//    robot.pincher.setPower(-.5);
//    sleep(100);
//    robot.pincher.setPower(0);
    DriveForwardDistance(1,28);
    sleep(500);
    StrafRightDistance(.3,16);
    sleep(1000);
    robot.Leftglock.setPosition(0);
    sleep(1200);
    DriveBackward(.5);
    sleep(1100);
    StopDriving();
    sleep(1000);
    robot.Leftglock.setPosition(.4);
    sleep(500);
    StrafLeftDistance(.4,37);
    sleep(1000);
    DriveForwardDistance(.8,40);
    sleep(2000);
    StrafRightDistance(.5,30);
    sleep(1000);
    DriveBackward(.8);
    sleep(1700);
    StrafLeftDistance(1,50);
    robot.leftDrive.setPower(0);
    robot.rightDrive.setPower(0);
    robot.leftBack.setPower(0);
    robot.rightBack.setPower(0);





}

public void DriveForwardDistance(double speed, int distanceInches)
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

    DriveForward(speed);

    while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
    {

    }
    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

public void DriveBackwardDistance(double speed, int distanceInches)
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

    DriveBackward(speed);

    while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
    {

    }
    StopDriving();
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}

public void TurnLeftDistance(double speed, int distanceInches)
{
    double rotationsneeded = distanceInches/CIRCUMFERENCE;
    int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

    robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
    robot.rightBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);

    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.rightDrive.setPower(speed);
    robot.rightBack.setPower(speed);
    robot.leftDrive.setPower(-speed);
    robot.leftBack.setPower(-speed);


    while( robot.rightDrive.isBusy() && robot.rightBack.isBusy() )
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
    robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    robot.leftDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);

    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.leftDrive.setPower(speed);
    robot.leftBack.setPower(speed);
    robot.rightDrive.setPower(-speed);
    robot.rightBack.setPower(-speed);


    while(robot.leftDrive.isBusy()  && robot.leftBack.isBusy()  )
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


    robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
    robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);

    robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.rightDrive.setPower(speed);
    robot.leftBack.setPower(speed);
    robot.rightBack.setPower(-speed);
    robot.leftDrive.setPower(-speed);



    while(robot.rightDrive.isBusy() && robot.leftBack.isBusy()) {

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

    robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);
    robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);

    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.rightBack.setPower(speed);
    robot.leftDrive.setPower(speed);
    robot.rightDrive.setPower(-speed);
    robot.leftBack.setPower(-speed);


    while(robot.leftDrive.isBusy() && robot.rightBack.isBusy() )
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
