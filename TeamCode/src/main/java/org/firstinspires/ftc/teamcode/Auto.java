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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.ArmAwareSetSlides;
import org.firstinspires.ftc.teamcode.backend.commands.DriverAssistedAutoTargetedDeposit;
import org.firstinspires.ftc.teamcode.backend.commands.DriverAssistedDeposit;
import org.firstinspires.ftc.teamcode.backend.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.commands.ReadyArmCarefully;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;


/**
 * I should probably document this...
 */

@Autonomous(name="Auto (THIS ONE)")
@Config
public class Auto extends CommandbasedOpmode {

    SampleMecanumDrive drive;

    TrajectorySequence startLTraj;
    TrajectorySequence startCTraj;
    TrajectorySequence startRTraj;

    TrajectorySequence prepDepositTraj;

    TrajectorySequence depositLTraj;
    TrajectorySequence depositCTraj;
    TrajectorySequence depositRTraj;


    TrajectorySequence parkTraj;

    private static final double REVERSE = Math.toRadians(180);

    public static double STARTX = 12;
    public static double STARTY = -63;
    public static double STARTTHETA = Math.toRadians(-90);
    public static double CPURPLEDEPOSITY = -31.5;
    public static double PREDEPOSITX = 32;
    public static double PREDEPOSITY = -36;
    public static double PREDEPOSITTHETA = REVERSE;
    public static double DEPOSITX = 51;
    public static double DEPOSITY = -36;
    public static double DEPOSITYOFFSET = 7; // The actual pitch is 6 in, but we want to err on the side of correctness
    public static double DEPOSITYSIDEBASEDOFFSET = 1.5;
    public static double DEPOSITTHETA = REVERSE;
    public static double PARKX = 60;
    public static double PARKY = -60;

    double startHeading;

    @Override
    public void init() {
        robot.init(hardwareMap, false);

        double CPURPLEDEPOSITX = STARTX;

        if (SetDrivingStyle.isBlue) {
            STARTY *= -1;
            STARTTHETA -= Math.toRadians(180);
            CPURPLEDEPOSITY *= -1;
            PREDEPOSITY *= -1;
            DEPOSITY *= -1;
            PARKY *= -1;
            CPURPLEDEPOSITX += 3.0;

        }
        DEPOSITY += DEPOSITYSIDEBASEDOFFSET;

        startHeading = robot.drivetrain.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, STARTTHETA);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Pose2d depositPose = new Pose2d(DEPOSITX, DEPOSITY, DEPOSITTHETA);
        Pose2d preDepositPose = new Pose2d(PREDEPOSITX, PREDEPOSITY, PREDEPOSITTHETA);
        Vector2d preParkPose = new Vector2d(DEPOSITX-4, (DEPOSITY+PARKY)/2);
        Vector2d parkPose = new Vector2d(PARKX, PARKY);

        startCTraj = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(CPURPLEDEPOSITX*0.8+PREDEPOSITX*0.2, CPURPLEDEPOSITY*0.7+STARTY*0.3, DEPOSITTHETA+REVERSE), STARTTHETA+REVERSE)
                .splineToSplineHeading(new Pose2d(CPURPLEDEPOSITX, CPURPLEDEPOSITY*1.1-STARTY*0.1, DEPOSITTHETA+REVERSE), STARTTHETA+REVERSE)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(CPURPLEDEPOSITX, CPURPLEDEPOSITY), STARTTHETA)
                .turn(MathUtils.normalizeAngle(STARTTHETA-(DEPOSITTHETA-REVERSE), 0.0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.purplePixel.activate())
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(CPURPLEDEPOSITX, (CPURPLEDEPOSITY+STARTY)/2), STARTTHETA)
                .setReversed(true)
                .splineToSplineHeading(preDepositPose, PREDEPOSITTHETA+REVERSE)
                .build();

        prepDepositTraj = drive.trajectorySequenceBuilder(preDepositPose)
                .setReversed(true)
                .addTemporalMarker(0.0, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.3, timer)))
                .addTemporalMarker(1.0, () -> robot.slides.setTargetPosition(0.0))
                .addTemporalMarker(1.5, () -> scheduler.schedule(new ReadyArmCarefully(robot.arm, robot.wrist, timer)))
                .waitSeconds(1.0)
                .splineToSplineHeading(depositPose, DEPOSITTHETA+REVERSE)
                // .splineToConstantHeading(preParkPose, DEPOSITTHETA)
                // .splineToConstantHeading(parkPose, DEPOSITTHETA)
                .build();

        depositLTraj = drive.trajectorySequenceBuilder(depositPose)
                .strafeRight(DEPOSITYOFFSET)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> scheduler.schedule(new DriverAssistedAutoTargetedDeposit(robot.arm, robot.wrist, timer)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.3))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake)))
                .strafeLeft(DEPOSITYOFFSET)
                .build();
        depositCTraj = drive.trajectorySequenceBuilder(depositPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> scheduler.schedule(new DriverAssistedAutoTargetedDeposit(robot.arm, robot.wrist, timer)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.3))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake)))
                .waitSeconds(1.25)
                .build();
        depositRTraj = drive.trajectorySequenceBuilder(depositPose)
                .strafeLeft(DEPOSITYOFFSET)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> scheduler.schedule(new DriverAssistedAutoTargetedDeposit(robot.arm, robot.wrist, timer)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.slides.setTargetPosition(0.3))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> scheduler.schedule(new ArmAwareSetSlides(robot.slides, robot.arm, robot.wrist, 0.0, timer, robot.intake)))
                .strafeRight(DEPOSITYOFFSET)
                .build();

        parkTraj = drive.trajectorySequenceBuilder(depositPose)
                .splineToConstantHeading(preParkPose, STARTTHETA)
                .splineToConstantHeading(parkPose, DEPOSITTHETA+REVERSE)
                .build();
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
        ArrayList<Command> auto = new ArrayList<>();
        auto.add(new FollowRRTraj(robot.drivetrain, drive, startCTraj)); // TODO L&R
        auto.add(new FollowRRTraj(robot.drivetrain, drive, prepDepositTraj));
        switch (robot.camera.getPropPosition()) {
            case LEFT:
                auto.add(new FollowRRTraj(robot.drivetrain, drive, depositLTraj));
                break;
            case CENTER:
                auto.add(new FollowRRTraj(robot.drivetrain, drive, depositCTraj));
                break;
            case RIGHT:
                auto.add(new FollowRRTraj(robot.drivetrain, drive, depositRTraj));
                break;
        }
        auto.add(new FollowRRTraj(robot.drivetrain, drive, parkTraj));
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