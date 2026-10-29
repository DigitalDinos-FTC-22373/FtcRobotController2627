package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Created by Tom on 9/26/17.  Updated 9/24/2021 for PIDF.
 * This assumes that you are using a REV Robotics Control Hub or REV Robotics Expansion Hub
 * as your DC motor controller.  This OpMode uses the extended/enhanced
 * PIDF-related functions of the DcMotorEx class.
 */

//@Config
@TeleOp
public class Catapult_tuning extends LinearOpMode {

    // our DC motor
    DcMotorEx catapult;
    DcMotorEx catapult2;
    DcMotorEx intake;
    public double NEW_P = 10.5;
    public double NEW_PP = 1.1;
    public double NEW_I = 0;
    public double NEW_D = -1.7;
    public double NEW_F = 579;
    // These values are for illustration only; they must be set
    // and adjusted for each motor based on its planned usage.
    public double set_angle_degrees = 0;
    public final double TICKS_PER_DEGREE= 537.7/360;
    public boolean IntakeOn = false;
    public int setposition =  (int)(set_angle_degrees * TICKS_PER_DEGREE);

    public void runOpMode() {
        // Get reference to DC motor.
        // Since we are using the Control Hub or Expansion Hub,
        // cast this motor to a DcMotorEx object.
        catapult = (DcMotorEx)hardwareMap.get(DcMotor.class, "catapult0");
        catapult2 = (DcMotorEx)hardwareMap.get(DcMotor.class, "catapult1");

        intake = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        // wait for start command
        waitForStart();
        catapult.setPower(1.0);
        catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult.setDirection(DcMotor.Direction.FORWARD);
        catapult.setTargetPosition(setposition);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        catapult2.setPower(1.0);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setDirection(DcMotor.Direction.REVERSE); // direrction good
        catapult2.setTargetPosition(setposition);
        catapult2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Get the PIDF coefficients for the RUN_TO_POSITION RunMode.
        PIDFCoefficients pidfOrig = catapult.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);


        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        catapult.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        catapult2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);


        catapult.setPositionPIDFCoefficients(NEW_PP);
        catapult2.setPositionPIDFCoefficients(NEW_PP);

        // Re-read coefficients and verify change.
        PIDFCoefficients pidfModified = catapult.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        catapult.setTargetPosition(setposition);
        catapult2.setTargetPosition(setposition);

        // display info to user
        while(opModeIsActive()) {

            if (gamepad1.aWasPressed()){
                if (set_angle_degrees < -50) {
                    set_angle_degrees = 0;
                    catapult.setPower(0.1);
                    catapult2.setPower(0.1);
                    NEW_F= 178;
                }
                else if (set_angle_degrees > -50){
                    catapult.setPower(1);
                    catapult2.setPower(1);
                    set_angle_degrees = -95;
                    NEW_F= 579;
                }
            }

            if(gamepad1.bWasPressed() && IntakeOn == false){
                IntakeOn = true;
                intake.setPower(1);
            }
            else if (gamepad1.bWasPressed() && IntakeOn == true){
                IntakeOn = false;
                intake.setPower(0);
            }

            if (gamepad1.left_bumper){
                NEW_F -= 1;
            }
            if (gamepad1.right_bumper){
                NEW_F += 1;
            }

            if (gamepad1.right_trigger > 0.5){
                NEW_PP += 0.01;
            }
            if (gamepad1.left_trigger > 0.5){
                NEW_PP -= 0.01;
            }

            if (gamepad1.dpad_left){
                NEW_P -= 0.01;
            }
            if (gamepad1.dpad_right){
                NEW_P += 0.01;
            }

            if (gamepad1.dpad_up){
                NEW_D += 0.01;
            }
            if (gamepad1.dpad_down){
                NEW_D -= 0.01;
            }


            setposition =  (int)(set_angle_degrees * TICKS_PER_DEGREE);

            pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            catapult.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            catapult2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            catapult.setPositionPIDFCoefficients(NEW_PP);
            catapult.setTargetPosition(setposition);

            catapult2.setPositionPIDFCoefficients(NEW_PP);
            catapult2.setTargetPosition(setposition);

            pidfModified = catapult.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.addData("New PP",  NEW_PP);
            telemetry.addData("New F",  NEW_F);
            telemetry.addData("New D",  NEW_D);
            telemetry.addData("New P",  NEW_P);
            telemetry.addData("catapult get angle",  catapult.getCurrentPosition()/TICKS_PER_DEGREE);
            telemetry.addData("catapult get angle 2",  catapult2.getCurrentPosition()/TICKS_PER_DEGREE);
            telemetry.addData("setangledegrees", set_angle_degrees);
            telemetry.update();
        }
    }
}