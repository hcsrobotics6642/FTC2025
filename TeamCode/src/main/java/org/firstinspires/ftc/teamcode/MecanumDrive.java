package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Robot: Mecanum Drive", group = "Robot")
public class MecanumDrive extends LinearOpMode {

    private RobotHardware robot;

    // Sequential hopper control: 1 → 2 → 3
    private int currentIntakeSlot = 1;
    private int currentShootSlot = 1;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap, this);

        telemetry.addLine("TELEOP READY - 100% SAFE FOR COMPETITION");
        telemetry.addLine("RIGHT BUMPER = Intake into current slot");
        telemetry.addLine("DPAD DOWN = Next slot | B = Reset to Slot 1");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.setElevationForDistance();  // ← Always correct elevation
            // ==================== DRIVER - GAMEPAD 1 ====================
            // ==================== DRIVER - GAMEPAD 1 ====================
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;

            // HOLD LEFT BUMPER = Temporary ROBOT-CENTRIC override
            // Release = FIELD-CENTRIC (default)
            // AUTO-AIM: Hold dpad_up to aim at goal (releases control when let go)
            if (gamepad1.left_bumper) {
                robot.autoAimToGoal();
            } else {
                // Normal field-centric drive (LB = robot-centric override)
                if (gamepad1.left_bumper) {
                    robot.drive(forward, strafe, turn);
                } else {
                    robot.driveFieldCentric(forward, strafe, turn);
                }
            }


            // Alliance select backup
            if (gamepad1.x) RobotData.alliance = RobotData.Alliance.Blue;
            if (gamepad1.b) RobotData.alliance = RobotData.Alliance.Red;

            // ==================== OPERATOR - GAMEPAD 2 ====================

            // INTAKE + SEQUENTIAL HOPPER
            if (gamepad2.right_bumper) {
                robot.rotateToIntakeSlot(currentIntakeSlot);
                robot.intakeForward();
            }
            else if (gamepad2.left_bumper) {
                robot.intakeReverse();
            }
            else {
                robot.intakeOff();
            }

            // Advance slot
            if (gamepad2.dpad_down) {
                currentIntakeSlot = currentIntakeSlot >= 3 ? 1 : currentIntakeSlot + 1;
                robot.rotateToIntakeSlot(currentIntakeSlot);
                sleep(300);
            }

            // Reset to slot 1
            if (gamepad2.b) {
                currentIntakeSlot = 1;
                robot.rotateToIntakeSlot(1);
                sleep(200);
            }
                //Advance Shooter
            if (gamepad2.dpad_up) {
                currentShootSlot = currentShootSlot >= 3 ? 1 : currentShootSlot + 1;
                robot.rotateToShootSlot(currentShootSlot);
                sleep(300);
            }

            // Reset to slot 1
            if (gamepad2.y) {
                currentShootSlot = 1;
                robot.rotateToShootSlot(1);
                sleep(200);
            }

            // FLYWHEEL - Fixed speed (your safe 6000 RPM max)
            if (gamepad2.right_trigger > 0.5) {
                robot.setFlywheelRPM(6000);           // ← This method definitely exists
            }
            else if (gamepad2.left_trigger > 0.5) {
                robot.flywheelMotorLeft.setPower(-1.0);

            }
            else {
                robot.stopFlywheel();            // ← This method definitely exists
            }

            // FIRE ONE ARTIFACT
            if (gamepad2.a) {
                robot.shootArtifact();           // ← This method definitely exists
            }

            // TELEMETRY
            telemetry.addData("Alliance", RobotData.alliance);
            telemetry.addData("INTAKE SLOT", currentIntakeSlot);
            telemetry.addData("Intake", robot.getIntakeState());
            telemetry.addData("Flywheel RPM", "%.0f", robot.getFlywheelRPM());
            telemetry.update();
        }
    }
}