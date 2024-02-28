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

import static org.firstinspires.ftc.teamcode.SetDrivingStyle.autoSecondsDelay;
import static org.firstinspires.ftc.teamcode.SetDrivingStyle.isBlue;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.ArmAwareSetSlides;
import org.firstinspires.ftc.teamcode.backend.commands.DriverAssistedAutoTargetedDeposit;
import org.firstinspires.ftc.teamcode.backend.commands.EnableIntakeSafe;
import org.firstinspires.ftc.teamcode.backend.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.commands.ReadyArmCarefully;
import org.firstinspires.ftc.teamcode.backend.commands.SlowIntakeFromStack;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;


/**
 * I should probably document this...
 */

@Autonomous(name="Audience Side Auto")
@Config
public class AudienceSideAuto extends CommandbasedOpmode {


    SampleMecanumDrive drive;

    TrajectorySequence startLTraj;
    TrajectorySequence startCTraj;
    TrajectorySequence startRTraj;

    TrajectorySequence intakeTraj;

    TrajectorySequence traverseTraj;

    TrajectorySequence depositLTraj;
    TrajectorySequence depositCTraj;
    TrajectorySequence depositRTraj;

    private static final double REVERSE = Math.toRadians(180);
    private static double CLOCKWISE90 = Math.toRadians(-90);

    public static double STARTX = -37;
    public static double STARTY = -63;
    public static double STARTTHETA = CLOCKWISE90;
    public static double LRPURPLEDEPOSITX = -43.5;
    public static double LRPURPLEDEPOSITXOFFSET = 10.5; // This is correct for Red R, Blue L, and must be negated for Red L, Blue R
    public static double LRPURPLEDEPOSITY = -36;
    public static double LRPURPLEDEPOSITTHETA = REVERSE;
    public static double CPURPLEDEPOSITY = -24.5;
    public static double PIXELINTAKEX = -57;
    public static double PIXELINTAKEY = -24;
    public static double TRAVERSESTARTX = -56;
    public static double TRAVERSEENDX = 48; // TODO
    public static double TRAVERSEY = -12;

    public static double EARLYPARKX = STARTX;
    public static double EARLYPARKY = STARTY;

    public static double DEPOSITX = 52.5;
    public static double DEPOSITY = -36;
    public static double DEPOSITYDELTA = -6;
    public static double PARKX = 63;
    public static double PARKY = SetDrivingStyle.autoParkCenter ? -12 : -62;

    double startHeading;

    @Override
    public void init() {
        robot.init(hardwareMap, false);

        if (isBlue) {
            CLOCKWISE90 *= -1;
            STARTY *= -1;
            STARTTHETA -= REVERSE;
            CPURPLEDEPOSITY *= -1;
            LRPURPLEDEPOSITY *= -1;
            PIXELINTAKEY *= -1;
            TRAVERSEY *= -1;
            DEPOSITY *= -1;
            PARKY *= -1;
            EARLYPARKY *= -1;
        } else {
            PIXELINTAKEX -= 1;
            // TODO DEPOSITY -= 1.5;
        }

        startHeading = robot.drivetrain.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, STARTTHETA);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        startRTraj = drive.trajectorySequenceBuilder(startPose) // This is actually the left trajectory on blue side
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(STARTX-4, LRPURPLEDEPOSITY*0.3+STARTY*0.7), STARTTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(LRPURPLEDEPOSITX, LRPURPLEDEPOSITY, REVERSE), LRPURPLEDEPOSITTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(LRPURPLEDEPOSITX+LRPURPLEDEPOSITXOFFSET, LRPURPLEDEPOSITY, REVERSE), LRPURPLEDEPOSITTHETA+REVERSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.purplePixel.activate())
                .setReversed(false)
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(LRPURPLEDEPOSITX, LRPURPLEDEPOSITY), LRPURPLEDEPOSITTHETA)
                .splineToConstantHeading(new Vector2d(PIXELINTAKEX, PIXELINTAKEY), REVERSE+CLOCKWISE90/2)
                .build();

        startCTraj = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(STARTX-4, CPURPLEDEPOSITY*0.2+STARTY*0.8), STARTTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(STARTX-4, CPURPLEDEPOSITY*0.7+STARTY*0.3, REVERSE), STARTTHETA+REVERSE)
                .splineToConstantHeading(new Vector2d(STARTX-4, CPURPLEDEPOSITY), STARTTHETA) // TODO +REVERSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.purplePixel.activate())
                .setReversed(false)
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(PIXELINTAKEX, PIXELINTAKEY), REVERSE)
                .build();

        Pose2d poseAdj = isBlue ? new Pose2d() : new Pose2d(1, 0, 0); // TODO
        startLTraj = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(STARTX-4, LRPURPLEDEPOSITY*0.3+STARTY*0.7), STARTTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(LRPURPLEDEPOSITX-LRPURPLEDEPOSITXOFFSET, LRPURPLEDEPOSITY, REVERSE), LRPURPLEDEPOSITTHETA+CLOCKWISE90/2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.purplePixel.activate())
                .waitSeconds(0.75)
                .setReversed(false)
                .lineTo(new Vector2d(PIXELINTAKEX-0.5, LRPURPLEDEPOSITY))
                .lineTo(new Vector2d(PIXELINTAKEX-0.5, PIXELINTAKEY))
                .lineTo(new Vector2d(PIXELINTAKEX, PIXELINTAKEY))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {drive.setPoseEstimate(drive.getPoseEstimate().plus(poseAdj));})
                .build();

        if (!SetDrivingStyle.shortAuto) {

            intakeTraj = drive.trajectorySequenceBuilder(new Pose2d(PIXELINTAKEX, PIXELINTAKEY, REVERSE))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        scheduler.schedule(new EnableIntakeSafe(robot.intake, robot.arm, robot.wrist, timer, 0.50));
                        robot.intake.lowerDropdown(4);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> new SlowIntakeFromStack(robot.intake, timer))
                    .lineTo(new Vector2d(PIXELINTAKEX - 1, PIXELINTAKEY))
                    .waitSeconds(1.25)
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> robot.intake.raiseDropdown())
                    .lineTo(new Vector2d(PIXELINTAKEX - 3, PIXELINTAKEY))
                    .waitSeconds(1.5)
                    .lineTo(new Vector2d(PIXELINTAKEX, PIXELINTAKEY))
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        robot.intake.hold();
                        robot.arm.toggle();
                        robot.wrist.toggle();
                    })
                    .build();

            traverseTraj = drive.trajectorySequenceBuilder(new Pose2d(PIXELINTAKEX, PIXELINTAKEY, REVERSE))
                    .setVelConstraint((v, pose2d, pose2d1, pose2d2) -> 35)
                    .addTemporalMarker(4.5, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.3, timer)))
                    .addTemporalMarker(5.5, () -> robot.slides.setTargetPosition(0.0))
                    .addTemporalMarker(6.0, () -> scheduler.schedule(new ReadyArmCarefully(robot.arm, robot.wrist, timer)))
                    .splineToConstantHeading(new Vector2d(PIXELINTAKEX, PIXELINTAKEY * 0.75 + TRAVERSEY * 0.25), -CLOCKWISE90)
                    .splineToConstantHeading(new Vector2d(TRAVERSESTARTX, TRAVERSEY), 0)
                    .splineToConstantHeading(new Vector2d(TRAVERSEENDX, TRAVERSEY), 0)
                    .build();

            double LEFTYDELTA = isBlue ? DEPOSITYDELTA : DEPOSITYDELTA-1;
            depositLTraj = drive.trajectorySequenceBuilder(new Pose2d(TRAVERSEENDX, TRAVERSEY, REVERSE))
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, DEPOSITY * 0.5 + TRAVERSEY * 0.5 - LEFTYDELTA * 0.5), CLOCKWISE90)
                    .splineToConstantHeading(new Vector2d(DEPOSITX, DEPOSITY - LEFTYDELTA), 0)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> scheduler.schedule(new DriverAssistedAutoTargetedDeposit(robot.arm, robot.wrist, timer)))
                    .waitSeconds(2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.3))
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake)))
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, DEPOSITY - LEFTYDELTA), REVERSE)
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, PARKY), (Math.abs(PARKY) > 36) ? CLOCKWISE90 : -CLOCKWISE90)
                    .lineToConstantHeading(new Vector2d(PARKX, PARKY))
                    .build();

            depositCTraj = drive.trajectorySequenceBuilder(new Pose2d(TRAVERSEENDX, TRAVERSEY, REVERSE))
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, DEPOSITY * 0.5 + TRAVERSEY * 0.5), CLOCKWISE90)
                    .splineToConstantHeading(new Vector2d(DEPOSITX, DEPOSITY), 0)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> scheduler.schedule(new DriverAssistedAutoTargetedDeposit(robot.arm, robot.wrist, timer)))
                    .waitSeconds(2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.3))
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake)))
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, DEPOSITY), REVERSE)
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, PARKY), (Math.abs(PARKY) > 36) ? CLOCKWISE90 : -CLOCKWISE90)
                    .lineToConstantHeading(new Vector2d(PARKX, PARKY))
                    .build();

            depositRTraj = drive.trajectorySequenceBuilder(new Pose2d(TRAVERSEENDX, TRAVERSEY, REVERSE))
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, DEPOSITY * 0.5 + TRAVERSEY * 0.5 + DEPOSITYDELTA * 0.5), CLOCKWISE90)
                    .splineToConstantHeading(new Vector2d(DEPOSITX, DEPOSITY + DEPOSITYDELTA), 0)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> scheduler.schedule(new DriverAssistedAutoTargetedDeposit(robot.arm, robot.wrist, timer)))
                    .waitSeconds(2)
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.3))
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake)))
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, DEPOSITY + DEPOSITYDELTA), REVERSE)
                    .splineToConstantHeading(new Vector2d(DEPOSITX - 4, PARKY), (Math.abs(PARKY) > 36) ? CLOCKWISE90 : -CLOCKWISE90)
                    .lineToConstantHeading(new Vector2d(PARKX, PARKY))
                    .build();
        } else {
            intakeTraj = drive.trajectorySequenceBuilder(new Pose2d(PIXELINTAKEX, PIXELINTAKEY, REVERSE))
                    .lineToConstantHeading(new Vector2d(PIXELINTAKEX, EARLYPARKY))
                    .lineToConstantHeading(new Vector2d(EARLYPARKX, EARLYPARKY))
                    .build();
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Prop detection", robot.camera.getPropPosition());
        telemetry.addData("Prop detection confidence", robot.camera.getPropConfidence());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.camera.propDetected();
        if (isBlue) {
            TrajectorySequence temp = startLTraj;
            startLTraj = startRTraj;
            startRTraj = temp;
        }
        TrajectorySequence depositTraj;
        ArrayList<Command> auto = new ArrayList<>();
        if (SetDrivingStyle.autoSecondsDelay != 0) {auto.add(new WaitCommand(autoSecondsDelay*1000));}
        switch (robot.camera.getPropPosition()) {
            case LEFT:
                auto.add(new FollowRRTraj(robot.drivetrain, drive, startLTraj));
                depositTraj = depositLTraj;
                break;
            case RIGHT:
                auto.add(new FollowRRTraj(robot.drivetrain, drive, startRTraj));
                depositTraj = depositRTraj;
                break;
            case CENTER:
            default: // This shouldn't do anything if everything is working
                auto.add(new FollowRRTraj(robot.drivetrain, drive, startCTraj));
                depositTraj = depositCTraj;
                break;
        }
        auto.add(new FollowRRTraj(robot.drivetrain, drive, intakeTraj));
        if (!SetDrivingStyle.shortAuto) {
            auto.add(new FollowRRTraj(robot.drivetrain, drive, traverseTraj));
            auto.add(new FollowRRTraj(robot.drivetrain, drive, depositTraj));
        }
        scheduler.schedule(false, new SequentialCommandGroup(auto.toArray(new Command[0])));
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void end() {
        AutoToTeleopContainer.getInstance().setAngleDelta(startHeading-robot.drivetrain.getHeading()+Math.toRadians(180));
    }
}