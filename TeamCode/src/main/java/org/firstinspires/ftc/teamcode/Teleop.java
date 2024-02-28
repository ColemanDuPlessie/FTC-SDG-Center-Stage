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

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.ArmAwareIncrementSlides;
import org.firstinspires.ftc.teamcode.backend.commands.ArmAwareSetSlides;
import org.firstinspires.ftc.teamcode.backend.commands.AutoTargetBackdrop;
import org.firstinspires.ftc.teamcode.backend.commands.DriveFromGamepad;
import org.firstinspires.ftc.teamcode.backend.commands.DriverAssistedAutoTargetedDeposit;
import org.firstinspires.ftc.teamcode.backend.commands.DriverAssistedDeposit;
import org.firstinspires.ftc.teamcode.backend.commands.EnableIntakeSafe;
import org.firstinspires.ftc.teamcode.backend.commands.ReadyArmCarefully;
import org.firstinspires.ftc.teamcode.backend.commands.RetractHang;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.WristSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


/**
 * I should probably document this...
 */

@TeleOp(name="Teleop (THIS ONE)")
public class Teleop extends CommandbasedOpmode {

    @Override
    public void init() {
        robot.init(hardwareMap, true);
    }

    private void setIntakeArmPosition() {
        if (robot.slides.getTargetPosition() > 0.05) {return;}
        if (robot.intake.getCurrentSpeed() == 0) {
            robot.arm.holding();
            robot.wrist.holding();
        } else {
            robot.arm.down();
            robot.wrist.down();
        }
    }

    private void toggleDropdown() {
        if (Math.abs(robot.intake.getCurrentDropdownPos() - IntakeSubsystem.dropdownUpPos) <= 0.05) {
            robot.intake.lowerDropdown(dropdownSetpoint);
        } else {
            robot.intake.raiseDropdown();
        }
    }

    private void toggleArm() {
        if (robot.arm.getTargetPosition() == ArmSubsystem.waitingPosition) {
            scheduler.schedule(new ReadyArmCarefully(robot.arm, robot.wrist, timer, robot.slides.getPosition() <= 0.35));
        } else {
            robot.arm.toggle();
            robot.wrist.toggle();
        }
    }

    private void xPressed() {
        if (pad1.getY()) {
            scheduler.schedule(new DriverAssistedAutoTargetedDeposit(robot.arm, robot.wrist, timer, robot.slides, robot.intake));
        } else if (robot.wrist.getTargetPosition() == WristSubsystem.readyPosition) {
            scheduler.schedule(new DriverAssistedDeposit(robot.arm, robot.wrist, timer, robot.slides, robot.intake));
        } else {
            toggleArm();
        }
    }

    private void toggleIntake(boolean isReversed) {
        if (robot.arm.getTargetPosition() == ArmSubsystem.downPosition || robot.arm.getTargetPosition() == ArmSubsystem.downWaitingPosition) {
            if (robot.intake.getCurrentSpeed() == 0.0) {
                scheduler.schedule(new EnableIntakeSafe(robot.intake, robot.arm, robot.wrist, timer, isReversed));
                return;
            } else if ((robot.intake.getCurrentSpeed() > 0.0) == isReversed) {
                robot.intake.setSpeed(0.0);
                scheduler.schedule(new EnableIntakeSafe(robot.intake, robot.arm, robot.wrist, timer, isReversed));
                return;
            } else {
                robot.arm.toggle();
                robot.wrist.toggle();
            }
        }
        if (isReversed) {
            robot.intake.toggleOuttake();
        } else {
            robot.intake.toggleIntake();
        }
    }

    private double slidesSetpoint = 0.3;
    private double slidesSetpointStep = 0.1;
    private int dropdownSetpoint = 4; // Min = 0, Max = 4

    @Override
    public void start() {
        scheduler.setDefaultCommand(robot.drivetrain, new DriveFromGamepad(robot.drivetrain, pad1, SetDrivingStyle.isFieldCentric));

        GamepadEx gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenReleased(robot.slides::hang);
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenReleased(() -> {robot.drone.activate();
                scheduler.schedule(new RetractHang(robot.slides, timer)); // RetractHang only does things if we're already in hanging position
                });

        if (SetDrivingStyle.memorizedSlidePosition) {
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenReleased(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake));
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenReleased(() -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, slidesSetpoint, timer, robot.intake)));
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                    .whenReleased(() -> {
                        slidesSetpoint += slidesSetpointStep;
                        slidesSetpoint = Math.min(1.0, Math.max(0.3, slidesSetpoint));
                        scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, slidesSetpoint, timer, robot.intake));
                    });
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                    .whenReleased(() -> {
                        slidesSetpoint -= slidesSetpointStep;
                        slidesSetpoint = Math.min(1.0, Math.max(0.3, slidesSetpoint));
                        scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, slidesSetpoint, timer, robot.intake));
                    });
        } else {
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenReleased(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake));
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenReleased(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.5, timer, robot.intake));
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                    .whenReleased(() -> scheduler.schedule(new ArmAwareIncrementSlides(robot.slides, robot.arm, robot.wrist, 0.1, timer, robot.intake)));
            gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                    .whenReleased(() -> scheduler.schedule(new ArmAwareIncrementSlides(robot.slides, robot.arm, robot.wrist, -0.1, timer, robot.intake)));
        }

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(() -> {dropdownSetpoint = Math.max(0, dropdownSetpoint-1); robot.intake.lowerDropdown(dropdownSetpoint);});
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(() -> {dropdownSetpoint = Math.min(4, dropdownSetpoint+1); robot.intake.lowerDropdown(dropdownSetpoint);});
        // gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        //         .whenReleased(this::toggleArm);

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenReleased(() -> toggleIntake(false));
        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenReleased(this::xPressed);
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenReleased(() -> toggleIntake(true));
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenReleased(this::toggleDropdown);
        // gamepad.getGamepadButton(GamepadKeys.Button.Y)
        //         .whenPressed(new AutoTargetBackdrop(robot.drivetrain, robot.camera, pad1, timer, robot.slides));

    }

    @Override
    public void loop() {
        telemetry.addData("Arm command", scheduler.requiring(robot.arm));
        telemetry.addData("Drivetrain command", scheduler.requiring(robot.drivetrain));
        telemetry.addData("Slides command", scheduler.requiring(robot.slides));
        telemetry.addData("Slides target position", robot.slides.getTargetPosition());
        telemetry.addData("Slides actual position", robot.slides.getPosition());
        if (scheduler.requiring(robot.drivetrain) instanceof AutoTargetBackdrop) {
            ((AutoTargetBackdrop) scheduler.requiring(robot.drivetrain)).debug(telemetry);
        }
        if (scheduler.requiring(robot.slides) instanceof ArmAwareSetSlides) {
            ((ArmAwareSetSlides) scheduler.requiring(robot.slides)).debug(telemetry);
        }
        for (AprilTagDetection det:robot.camera.getRawTagDetections()) {
            if (det == null || det.ftcPose == null) {continue;}
            telemetry.addLine();
            telemetry.addLine(String.format("April tag with id #%d detected at the following coordinates:", det.id));
            telemetry.addData("X", det.ftcPose.x);
            telemetry.addData("Y", det.ftcPose.y);
            telemetry.addData("Z", det.ftcPose.z);
            telemetry.addData("Pitch", det.ftcPose.pitch);
            telemetry.addData("Yaw", det.ftcPose.yaw);
            telemetry.addData("Roll", det.ftcPose.roll);
        }
    }
}