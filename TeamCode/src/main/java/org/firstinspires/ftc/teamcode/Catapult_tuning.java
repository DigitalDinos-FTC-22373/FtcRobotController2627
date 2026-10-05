package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    public double NEW_P = 25;
    public double NEW_PP = 1.2;
    public static final double NEW_I = 0;
    public static final double NEW_D = 0;
    public double NEW_F = 670;
    // These values are for illustration only; they must be set
    // and adjusted for each motor based on its planned usage.
    public double set_angle_degrees = 0;
    public final double TICKS_PER_DEGREE= 537.7/360;
    public int setposition =  (int)(set_angle_degrees * TICKS_PER_DEGREE);

    public void runOpMode() {
        // Get reference to DC motor.
        // Since we are using the Control Hub or Expansion Hub,
        // cast this motor to a DcMotorEx object.
        catapult = (DcMotorEx)hardwareMap.get(DcMotor.class, "testShooterMotor");
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        // wait for start command
        waitForStart();
        catapult.setPower(1.0);
        catapult.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult.setDirection(DcMotor.Direction.FORWARD);
        catapult.setTargetPosition(setposition);
        catapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Get the PIDF coefficients for the RUN_TO_POSITION RunMode.
        PIDFCoefficients pidfOrig = catapult.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        catapult.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        catapult.setPositionPIDFCoefficients(NEW_PP);

        // Re-read coefficients and verify change.
        PIDFCoefficients pidfModified = catapult.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        catapult.setTargetPosition(setposition);

        // display info to user
        while(opModeIsActive()) {

            if (gamepad1.right_bumper){
                NEW_F += 1;
            }
            if (gamepad1.left_bumper){
                NEW_F -= 1;
            }

            if (gamepad1.right_trigger > 0.5){
                NEW_PP += 0.1;
            }
            if (gamepad1.left_trigger > 0.5){
                NEW_PP -= 0.1;
            }

            if (gamepad1.aWasPressed()){
                if (set_angle_degrees > 75) {
                    set_angle_degrees = 20;
                }
                else if (set_angle_degrees < 75){
                    set_angle_degrees = 100;
                }
            }
            setposition =  (int)(set_angle_degrees * TICKS_PER_DEGREE);

            pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            catapult.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            catapult.setPositionPIDFCoefficients(NEW_PP);
            catapult.setTargetPosition(setposition);

            pidfModified = catapult.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Runtime (sec)", "%.01f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
            telemetry.addData("New PP",  NEW_PP);
            telemetry.addData("New F",  NEW_F);
            telemetry.addData("catapult get positon",  catapult.getCurrentPosition());
            telemetry.addData("setposition",  setposition);
            telemetry.update();
        }
    }
}
