package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name="SingleMotorTester", group="BionicBot")

public class SingleMotorTestingClass extends LinearOpMode {

    DcMotor Tester;

public void runOpMode()  {

    Tester = hardwareMap.dcMotor.get("Tester");

    waitForStart();

    while(opModeIsActive())
    {
        Tester.setPower(gamepad1.left_stick_y);
    }

}
}
