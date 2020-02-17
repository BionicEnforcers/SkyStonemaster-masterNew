package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;


public class MovementHelper{
private boolean red;

private HardwareBionicbot robot;
private OpModeBase opMode;
//private ElapsedTime runtime = new ElapsedTime();
//private static final int SLOW_DOWN_TURN_DEGREES = 20;

public MovementHelper(boolean red, HardwareBionicbot robot, OpModeBase opMode) {
    this.red = red;
    this.robot = robot;
    this.opMode = opMode;
}
//old gyro code
    /*
    public void calibrate(){
        robot.gyro.calibrate();
        while(robot.gyro.isCalibrating() && opMode.opModeIsActive()){
            opMode.telemetry.addData("chill out yo", "gyro calibrating");
            opMode.telemetry.update();
        }
    }


    int heading(){
        if(robot.gyro.getHeading() > 180)
            return 360 - robot.gyro.getHeading();
        else
            return -robot.gyro.getHeading();
    }
    */

public void rotate(int degrees, double speed, boolean reset) {
        /*

        double turnTargetR = -(15.65 * Math.PI * (degrees / 360));
        double turnTargetL = -turnTargetR;

        if(reset)
            calibrate();
        for(DcMotor motor : robot.motors)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for(DcMotor motor : robot.motors)
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setSpeed(degrees, speed);

        while (opMode.opModeIsActive() && Math.abs(-robot.leftBack.getCurrentPosition() - (int)(turnTargetR / 12.57 * 1120)) > 50){
            double currentInches = robot.leftBack.getCurrentPosition();
            opMode.telemetry.addData("target", turnTargetR);
            opMode.telemetry.addData("current", currentInches);
            opMode.telemetry.addData("offset", -robot.leftBack.getCurrentPosition() - (int)(turnTargetR / 12.57 * 1120));
            opMode.telemetry.update();
        }
        while (!opMode.isStopRequested() && opMode.opModeIsActive() && (Math.abs(heading() - degrees)) > 3) {
            if(offset < SLOW_DOWN_TURN_DEGREES) {
                double betterSpeed = speed * (distanceToTarget / SLOW_DOWN_TURN_DEGREES);
                setSpeed(distanceToTarget, betterSpeed);
                opMode.telemetry.addData("speed", "" + betterSpeed);
            }
            setSpeed(degrees, speed);
            //heading = -robot.gyro.getZThing();
            opMode.telemetry.addData("target", degrees);
            opMode.telemetry.addData("current", heading());
            opMode.telemetry.addData("offset", Math.abs(heading() - degrees));
            opMode.telemetry.update();
        }
        for(DcMotor motor : robot.motors){
            motor.setPower(0);
        }
        */
}

private void setSpeed(int target, double speed) {
    if (target > 0) {
        robot.leftBack.setPower(speed);
        robot.leftDrive.setPower(speed);
        robot.rightDrive.setPower(-speed);
        robot.rightBack.setPower(-speed);
    } else {
        robot.leftBack.setPower(-speed);
        robot.leftDrive.setPower(-speed);
        robot.rightDrive.setPower(speed);
        robot.rightBack.setPower(speed);
    }
}


public void goSideways(double speed, double targetDistance){
    if(red)
        targetDistance *= -1;

    for(DcMotor motor : robot.motors){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    int target = (int)(targetDistance / 12.57 * 1120 * Math.sqrt(2));

    setSpeed(target, speed);
    while (opMode.opModeIsActive() && (Math.abs(-robot.leftDrive.getCurrentPosition() - target)) > 50 ){
        double currentTicks = robot.rightBack.getCurrentPosition();
        opMode.telemetry.addData("target", target);
        opMode.telemetry.addData("current", currentTicks);
        opMode.telemetry.update();
    }
    for(DcMotor motor : robot.motors){
        motor.setPower(0);
    }
}
/*
public void driveAngle(double speed, double x, double y)
{
    if (opMode.opModeIsActive()) {
        opMode.telemetry.addData("", "op mode active!");
        opMode.telemetry.update();
    }
    for(DcMotor motor : robot.motors)
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    for(DcMotor motor : robot.motors)
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    try {
        opMode.waitOneFullHardwareCycle();
    } catch(Exception e) {

    }

}
*/
public void drive(double speed, double targetDistance){


    if (opMode.opModeIsActive()) {
        opMode.telemetry.addData("", "op mode active!");
        opMode.telemetry.update();

        for(DcMotor motor : robot.motors)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for(DcMotor motor : robot.motors)
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            /*
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            */
        opMode.telemetry.update();
        try {
            opMode.waitOneFullHardwareCycle();
        } catch(Exception e) {

        }

    }
    while (!opMode.isStopRequested() && opMode.opModeIsActive() && (Math.abs(robot.leftBack.getCurrentPosition() - (int)(targetDistance / 12.57 * 1120))) > 50 ){
        //(robot.leftDrive.isBusy() && robot.rightBack.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy())){
        for(DcMotor motor : robot.motors)
            opMode.telemetry.addData(motor.getDeviceName()+ "", motor.getCurrentPosition());

            /*
            opMode.telemetry.addData("Left Back", robot.leftBack.getCurrentPosition());
            double RightBackTicks = robot.rightBack.getCurrentPosition();
            opMode.telemetry.addData("right back", RightBackTicks);
            double LeftFrontTicks = robot.leftDrive.getCurrentPosition();
            opMode.telemetry.addData("left front", LeftFrontTicks);
            double RightFrontTicks = robot.rightDrive.getCurrentPosition();
            opMode.telemetry.addData("right front", RightFrontTicks);
            */
        opMode.telemetry.update();
        speed = (1 -((double)robot.leftBack.getCurrentPosition() / (targetDistance / 12.57 * 1120))) * speed + .1;
        if(targetDistance < 0)
            speed *= -1;
        for(DcMotor motor : robot.motors)
            motor.setPower(speed);
            /*
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);
            */
        try {
            opMode.waitOneFullHardwareCycle();
        } catch(Exception e) {
        }
    }
    for (DcMotor motor : robot.motors){
        motor.setPower(0);
    }
}

public void testDrive(double speed, double targetDistance){

    if(targetDistance < 0) {
        speed = -Math.abs(speed);
    }
    if (opMode.opModeIsActive()) {
        opMode.telemetry.addData("", "op mode active!");
        opMode.telemetry.update();

        for(DcMotor motor : robot.motors)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        for(DcMotor motor : robot.motors)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /*
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            */
        opMode.telemetry.update();
        try {
            opMode.waitOneFullHardwareCycle();
        } catch(Exception e) {

        }
        for(DcMotor motor: robot.motors)
        {
            motor.setTargetPosition(1000);
            motor.setPower(speed);
        }
              /*
            robot.leftDrive.setTargetPosition(1000);
            robot.rightDrive.setTargetPosition(1000);
            robot.leftBack.setTargetPosition(1000);
            robot.rightBack.setTargetPosition(1000);
            robot.leftDrive.setPower(speed);
            robot.rightDrive.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);
            */
    }
    while (!opMode.isStopRequested() && opMode.opModeIsActive() && (Math.abs(robot.leftBack.getCurrentPosition() - (int)(targetDistance / 12.57 * 1120))) > 50 ){
        //(robot.leftDrive.isBusy() && robot.rightBack.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy())){
        for(DcMotor motor : robot.motors)
            opMode.telemetry.addData(motor.getDeviceName() + "", motor.getCurrentPosition());
            /*
            double LeftBackTicks = robot.leftBack.getCurrentPosition();
            opMode.telemetry.addData("Left Back", LeftBackTicks);
            double RightBackTicks = robot.rightBack.getCurrentPosition();
            opMode.telemetry.addData("right back", RightBackTicks);
            double LeftFrontTicks = robot.leftDrive.getCurrentPosition();
            opMode.telemetry.addData("left front", LeftFrontTicks);
            double RightFrontTicks = robot.rightDrive.getCurrentPosition();
            opMode.telemetry.addData("right front", RightFrontTicks);
            */
        opMode.telemetry.update();
        try {
            opMode.waitOneFullHardwareCycle();
        } catch(Exception e) {
        }
    }
    for (DcMotor motor : robot.motors){
        motor.setPower(0);
    }
}

public void diaganolDrive(double speed, int targetDistance, char direction){

    if (opMode.opModeIsActive()) {
        opMode.telemetry.addData("", "op mode active!");
        opMode.telemetry.update();
        for(DcMotor motor : robot.motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(direction =='l'){
            robot.rightDrive.setPower(speed);
            robot.leftBack.setPower(speed);
        } else {
            robot.leftDrive.setPower(speed);
            robot.rightBack.setPower(speed);
        }

    }
    while (!opMode.isStopRequested() && opMode.opModeIsActive() && (Math.abs(robot.leftBack.getCurrentPosition() - (int)(targetDistance / 12.57 * 1120 * Math.sqrt(2) * 1.5)) > 50) && (Math.abs(robot.rightBack.getCurrentPosition() - (int)(targetDistance / 12.57 * 1120 * Math.sqrt(2) * 1.5)) > 50)){
        //(robot.leftDrive.isBusy() && robot.rightBack.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy())){
        for(DcMotor motor : robot.motors)
            opMode.telemetry.addData(motor.getDeviceName() + "", motor.getCurrentPosition());
            /*
            double LeftBackTicks = robot.leftBack.getCurrentPosition();
            opMode.telemetry.addData("Left Back", LeftBackTicks);
            double RightBackTicks = robot.rightBack.getCurrentPosition();
            opMode.telemetry.addData("right back", RightBackTicks);
            double LeftFrontTicks = robot.leftDrive.getCurrentPosition();
            opMode.telemetry.addData("left front", LeftFrontTicks);
            double RightFrontTicks = robot.rightDrive.getCurrentPosition();
            opMode.telemetry.addData("right front", RightFrontTicks);
            */
        opMode.telemetry.update();
        try {
            opMode.waitOneFullHardwareCycle();
        } catch(Exception e) {

        }
    }
    for (DcMotor motor : robot.motors){
        motor.setPower(0);
    }
}
}

