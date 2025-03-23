// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PositionGripper {

    private static final HashMap<ArmState, ArmStateParameters> positionDictionary = new HashMap<ArmState, ArmStateParameters>();
    private static final HashMap<ArmState, HashMap<ArmState, Command>> transitionCommands = new HashMap<ArmState, HashMap<ArmState, Command>>();

    ArmStateManager armStateManager;
    ArmState targetState;

    Command commandToRun;

    Elevator theElevator;
    Arm theArm;
    Wrist theWrist;

    /** Creates a new MoveArm. */
    public PositionGripper(ArmStateManager asm, ArmState s, Elevator e, Arm a, Wrist w) {
        this.armStateManager = asm;
        this.targetState = s;
        theElevator = e;
        theArm = a;
        theWrist = w;

        if (positionDictionary.isEmpty()) {
            initializePositionDictionary();
        }
        if (transitionCommands.isEmpty()) {
            buildTransitionCommands();
        }

        // Use addRequirements() here to declare subsystem dependencies.
    }

    public Command command() {
        return Commands.defer(() -> {
            return transitionCommands.get(armStateManager.getCurrentState()).get(targetState);
        },
        Set.of(theElevator, theArm, theWrist));
    }

    /** CHANGE: this version is just for testing */
    private void initializePositionDictionary() {
        positionDictionary.put(ArmState.Level1, new ArmStateParameters(0, 0, 45, 90, 0.0));
        positionDictionary.put(ArmState.Level2, new ArmStateParameters(0, 8.0, 55, 0, 0));
        positionDictionary.put(ArmState.Level3, new ArmStateParameters(9.0, 15, 55, 0, 0));
        positionDictionary.put(ArmState.Level4, new ArmStateParameters(27.65, 21.137, 48.34, 0, 0));
        positionDictionary.put(ArmState.WaitingForCoral, new ArmStateParameters(0, 16, -75, 0, 0));
        positionDictionary.put(ArmState.Running, new ArmStateParameters(0, 0, 90, 0, 0));
        positionDictionary.put(ArmState.StartAlgaePosition, new ArmStateParameters(0, 0, 45, 90, -0.2));
        positionDictionary.put(ArmState.Climb, new ArmStateParameters(0, 0, 0, 0, 0.0));
    }

    // private void realInitializePositionDictionary() {
    //     // need to change speeds to all of these (right now assuming grabing is + and
    //     // release is -)
    //     // The PreL1 and L1 values below are approximations. The PreL1 arm angle and
    //     // carriage ht are
    //     // to get the arm beyond the elevator frame before rotating the coral in the
    //     // the gripper (which happens in the L1 state).
    //     // Our target ht for the gripper over the trough is 25".
    //     // VALUES IN DEGREES & INCHES. Convert as necessary.
    //     // NOTE: All carriage heights should be reduced by 4" once the carriage resting,
    //     // aka zero,
    //     // level is reset upward to adjust for the chute.
    //     // TEST: For L2-4 scoring states, e.g. ArmState.Level3, we are not changing the
    //     // arm angle
    //     // during the scoring motion to begin testing. Eventually we will lower the
    //     // angle to
    //     // facilitate scoring. Not changing angle is to insure we don't damage the arm
    //     // by running it into the reef. We simply lower the carriage a small amount (2")
    //     // to
    //     // see if the end of the coral is extended too far, too little or just enough to
    //     // slide
    //     // over the top of the reef branch.

    //     // Starting from Running, move the arm angle so that the coral in the gripper is
    //     // outside the
    //     // body of the elevator before rotating the wrist. This is to avoid the coral
    //     // colliding
    //     // with the elevator frame when turned horizontal.
    //     // wpk positionDictionary.put(ArmState.PreLevel1, new ArmStateParameters(0, 22.25, 45, 0, 0));
    //     // We should never enter this state without having gone through PreLevel1 first.
    //     // Once the coral is safely rotated, proceed to the position for ejecting the
    //     // coral
    //     // into the trough.
    //     // The carriage height and arm angle here are chosen so that the coral in the
    //     // gripper would
    //     // be elevated above and slightly over coral resting in the front row of the
    //     // trough.
    //     // The gripper eject speed needs to be high enough that the ejected coral will
    //     // roll off of
    //     // any coral already in the front row of the trough and to the second row.
    //     // STRETCH: We could use odometry to let us know when clear enough from the reef
    //     // to move
    //     // the arm to WaitingForCoral automatically. Same for L2-4. It risks hitting
    //     // another
    //     // obstacle like algae or another bot, but is relatively safe as we are likely
    //     // backing
    //     // away from the reef leaving empty space in front of us. We need ~4" of
    //     // clearance to
    //     // be safe (relative to the reef).
    //     positionDictionary.put(ArmState.Level1, new ArmStateParameters(0, 20, -30, 90, -0.2));
    //     positionDictionary.put(ArmState.ScoreLevel1, new ArmStateParameters(0, 20, -30, 90, 0.0));
    //     // PreL2 is initiated by the operator's L2 button and positions the coral above
    //     // (1")
    //     // the L2 reef branch ready for scoring/release.
    //     // PreL3/4 are similar.
    //     positionDictionary.put(ArmState.Level2, new ArmStateParameters(0, 12.163, 60, 0, 0));
    //     // L2 is the scoring state. It causes the movement of the carriage & arm down
    //     // over the L2
    //     // reef branch. It also softly (velocity to be tested), ejects the coral from
    //     // the
    //     // gripper.
    //     // TBD: It may also rotate the arm more lower to help the coral get on the
    //     // reef branch assuming geometry allows.
    //     // This state should NEVER be entered unless PreL2 is complete.
    //     // Level3/4 states are similar.
    //     positionDictionary.put(ArmState.ScoreLevel2, new ArmStateParameters(0, 10, 60, 0, -0.1));
    //     positionDictionary.put(ArmState.Level3, new ArmStateParameters(1.264, 26.5, 60, 0, 0));
    //     positionDictionary.put(ArmState.ScoreLevel3, new ArmStateParameters(1.264, 24.5, 60, 0, -0.1));
    //     positionDictionary.put(ArmState.Level4, new ArmStateParameters(25.664, 26.5, 60, 0, 0));
    //     positionDictionary.put(ArmState.ScoreLevel4, new ArmStateParameters(25.664, 24.5, 60, 0, -0.1));
    //     // WFC positions the arm so that the gripper is hovering just above where coral
    //     // will arrive in
    //     // the chute.
    //     // After any coral scoring action, we will return to this state.
    //     // TBD: Auto or by operator action?
    //     // TEST: How safely can this occur when we are still parked at the reef? Since
    //     // the arm
    //     // rotates down past horizontal, it could collide with the reef.
    //     positionDictionary.put(ArmState.WaitingForCoral, new ArmStateParameters(0, 18.29, -60, 0, 0));
    //     // LoadCoral lowers the gripper onto coral in the chute and spins the gripper
    //     // wheels to
    //     // pull in the coral.
    //     // This state should never be entered unless previously in PreLoadCoral.
    //     // This state should always be followed by PostLoadCoral.
    //     // TBD: Is the above true if the load fails, i.e. the gripper fails to pick up
    //     // the coral?
    //     positionDictionary.put(ArmState.LoadCoral, new ArmStateParameters(0, 15.69, -75, 0, 0.5));
    //     // PostLoadCoral raises the carriage enough so the the gripper with loaded coral
    //     // clears the
    //     // outside edge of the chute. This allows the arm to rotate up with coral
    //     // without that
    //     // coral colliding with the chute. It allows us to return to the Running
    //     // (travelling)
    //     // position safely.
    //     // This state should NEVER be entered unless previously in LoadCoral.
    //     // TBD: Do we automatically go to Running from here? If so, can we be sure that
    //     // when
    //     // the arm rotates up, that we won't run the gripper into another bot?
    //     // Or is operator input required to go to running?
    //     positionDictionary.put(ArmState.PostLoadCoral, new ArmStateParameters(0, 18, -75, 0, 0));
    //     // positionDictionary.put(ArmState.AlgaeLow, new ArmStateParameters(0, 0, 0, 0,
    //     // -1));
    //     // positionDictionary.put(ArmState.AlgaeHigh, new ArmStateParameters(0, 0, 0, 0,
    //     // -1));
    //     positionDictionary.put(ArmState.Startup, new ArmStateParameters(0, 0, 0, 0, 0));
    //     // Running, aka "travelling", has the carriage down and the arm up with gripper
    //     // rotated for L2-4
    //     // We expect to move here after PostLoadCoral, so whenever there is coral being
    //     // carried.
    //     positionDictionary.put(ArmState.Running, new ArmStateParameters(0, 0, 90, 0, 0));
    //     // FRA is the state/position we move to as we perform algae removal. Here, we
    //     // move from low
    //     // to high.
    //     // TBD: How high is high enough? For starters, we max out elev+carr. Optimize to
    //     // lower level.
    //     // TBD: Do we need to keep the gripper wheels ejecting the whole time we are
    //     // moving to this
    //     // position? If so, leave them running in this position. They will stop on next
    //     // transition.
    //     positionDictionary.put(ArmState.StartAlgaePosition, new ArmStateParameters(0, 22.25, -30, 90, -0.2));
    //     positionDictionary.put(ArmState.FinishRemovingAlgae, new ArmStateParameters(25, 26.5, 60, 90, -0.5));
    //     // positionDictionary.put(ArmState.SafeToLowerArm, new ArmStateParameters(0, 0, 0, 90, 0));

    // }

    private Command createElevatorFirstCommand(ArmState targetState) {
        Command c = Commands.sequence(
                new InstantCommand(() -> armStateManager.setTargetState(targetState)),
                new MoveElevator(theElevator, positionDictionary.get(targetState).getElevatorHeight(),
                        positionDictionary.get(targetState).getCarriageHeight()),
                new ParallelCommandGroup(
                        new MoveArm(theArm, positionDictionary.get(targetState).getArmAngle()),
                        new RotateWrist(theWrist, positionDictionary.get(targetState).getWristAngle())),
                new InstantCommand(() -> armStateManager.setCurrentState(targetState)));
        return c;
    }

    private Command createStraightenArmFirstCommand(ArmState targetState) {
        Command c = Commands.sequence(
                new InstantCommand(() -> armStateManager.setTargetState(targetState)),
                new ParallelCommandGroup(
                        new MoveArm(theArm, positionDictionary.get(ArmState.Running).getArmAngle()),
                        new RotateWrist(theWrist, positionDictionary.get(ArmState.Running).getWristAngle())),
                new MoveElevator(theElevator, positionDictionary.get(targetState).getElevatorHeight(),
                        positionDictionary.get(targetState).getCarriageHeight()),
                new ParallelCommandGroup(
                        new MoveArm(theArm, positionDictionary.get(targetState).getArmAngle()),
                        new RotateWrist(theWrist, positionDictionary.get(targetState).getWristAngle())),
                new InstantCommand(() -> armStateManager.setCurrentState(targetState)));
        return c;
    }

    private Command createParallelMovementCommand(ArmState targetState) {
        Command c = Commands.sequence(
                new InstantCommand(() -> armStateManager.setTargetState(targetState)),
                new ParallelCommandGroup(
                        new MoveElevator(theElevator, positionDictionary.get(targetState).getElevatorHeight(),
                                positionDictionary.get(targetState).getCarriageHeight()),
                        new MoveArm(theArm, positionDictionary.get(targetState).getArmAngle()),
                        new RotateWrist(theWrist, positionDictionary.get(targetState).getWristAngle())),
                new InstantCommand(() -> armStateManager.setCurrentState(targetState)));
        return c;
    }

    private void buildTransitionCommands() {

        // transitions from running
        transitionCommands.put(ArmState.Running, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.Running).put(ArmState.Level1, createElevatorFirstCommand(ArmState.Level1)) ;
        transitionCommands.get(ArmState.Running).put(ArmState.Level2, createElevatorFirstCommand(ArmState.Level2)) ;
        transitionCommands.get(ArmState.Running).put(ArmState.Level3, createElevatorFirstCommand(ArmState.Level3)) ;
        transitionCommands.get(ArmState.Running).put(ArmState.Level4, createElevatorFirstCommand(ArmState.Level4)) ;
        transitionCommands.get(ArmState.Running).put(ArmState.WaitingForCoral, createParallelMovementCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.Running).put(ArmState.StartAlgaePosition, createElevatorFirstCommand(ArmState.StartAlgaePosition)) ;
        transitionCommands.get(ArmState.Running).put(ArmState.Running, createElevatorFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.Running).put(ArmState.Climb, createElevatorFirstCommand(ArmState.Climb)) ;

        // transitions from Level 1
        transitionCommands.put(ArmState.Level1, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.Level1).put(ArmState.Level1, new NoOpCommand()) ;
        transitionCommands.get(ArmState.Level1).put(ArmState.Level2, createStraightenArmFirstCommand(ArmState.Level2)) ;
        transitionCommands.get(ArmState.Level1).put(ArmState.Level3, createStraightenArmFirstCommand(ArmState.Level3)) ;
        transitionCommands.get(ArmState.Level1).put(ArmState.Level4, createStraightenArmFirstCommand(ArmState.Level4)) ;
        transitionCommands.get(ArmState.Level1).put(ArmState.WaitingForCoral, createStraightenArmFirstCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.Level1).put(ArmState.StartAlgaePosition, createStraightenArmFirstCommand(ArmState.StartAlgaePosition)) ;
        transitionCommands.get(ArmState.Level1).put(ArmState.Running, createStraightenArmFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.Level1).put(ArmState.Climb, createStraightenArmFirstCommand(ArmState.Climb)) ;

        // transitions from Level 2
        transitionCommands.put(ArmState.Level2, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.Level2).put(ArmState.Level1, createStraightenArmFirstCommand(ArmState.Level1)) ;
        transitionCommands.get(ArmState.Level2).put(ArmState.Level2, new NoOpCommand()) ;
        transitionCommands.get(ArmState.Level2).put(ArmState.Level3, createStraightenArmFirstCommand(ArmState.Level3)) ;
        transitionCommands.get(ArmState.Level2).put(ArmState.Level4, createStraightenArmFirstCommand(ArmState.Level4)) ;
        transitionCommands.get(ArmState.Level2).put(ArmState.WaitingForCoral, createStraightenArmFirstCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.Level2).put(ArmState.StartAlgaePosition, createStraightenArmFirstCommand(ArmState.StartAlgaePosition)) ;
        transitionCommands.get(ArmState.Level2).put(ArmState.Running, createStraightenArmFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.Level2).put(ArmState.Climb, createStraightenArmFirstCommand(ArmState.Climb)) ;

        // transitions from Level 3
        transitionCommands.put(ArmState.Level3, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.Level3).put(ArmState.Level1, createStraightenArmFirstCommand(ArmState.Level1)) ;
        transitionCommands.get(ArmState.Level3).put(ArmState.Level2, createStraightenArmFirstCommand(ArmState.Level2)) ;
        transitionCommands.get(ArmState.Level3).put(ArmState.Level3, new NoOpCommand()) ;
        transitionCommands.get(ArmState.Level3).put(ArmState.Level4, createStraightenArmFirstCommand(ArmState.Level4)) ;
        transitionCommands.get(ArmState.Level3).put(ArmState.WaitingForCoral, createStraightenArmFirstCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.Level3).put(ArmState.StartAlgaePosition, createStraightenArmFirstCommand(ArmState.StartAlgaePosition)) ;
        transitionCommands.get(ArmState.Level3).put(ArmState.Running, createStraightenArmFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.Level3).put(ArmState.Climb, createStraightenArmFirstCommand(ArmState.Climb)) ;

        // transitions from Level 4
        transitionCommands.put(ArmState.Level4, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.Level4).put(ArmState.Level1, createStraightenArmFirstCommand(ArmState.Level1)) ;
        transitionCommands.get(ArmState.Level4).put(ArmState.Level2, createStraightenArmFirstCommand(ArmState.Level2)) ;
        transitionCommands.get(ArmState.Level4).put(ArmState.Level3, createStraightenArmFirstCommand(ArmState.Level3)) ;
        transitionCommands.get(ArmState.Level4).put(ArmState.Level4, new NoOpCommand()) ;
        transitionCommands.get(ArmState.Level4).put(ArmState.WaitingForCoral, createStraightenArmFirstCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.Level4).put(ArmState.StartAlgaePosition, createStraightenArmFirstCommand(ArmState.StartAlgaePosition)) ;
        transitionCommands.get(ArmState.Level4).put(ArmState.Running, createStraightenArmFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.Level4).put(ArmState.Climb, createStraightenArmFirstCommand(ArmState.Climb)) ;

        // transitions from WaitingForCoral
        transitionCommands.put(ArmState.WaitingForCoral, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.Level1, createStraightenArmFirstCommand(ArmState.Level1)) ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.Level2, createStraightenArmFirstCommand(ArmState.Level2)) ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.Level3, createStraightenArmFirstCommand(ArmState.Level3)) ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.Level4, createStraightenArmFirstCommand(ArmState.Level4)) ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.WaitingForCoral, createStraightenArmFirstCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.StartAlgaePosition, createStraightenArmFirstCommand(ArmState.StartAlgaePosition)) ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.Running, createStraightenArmFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.WaitingForCoral).put(ArmState.Climb, createStraightenArmFirstCommand(ArmState.Climb)) ;
        
        // transitions from StartAlgaePosition
        transitionCommands.put(ArmState.StartAlgaePosition, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.Level1, createStraightenArmFirstCommand(ArmState.Level1)) ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.Level2, createStraightenArmFirstCommand(ArmState.Level2)) ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.Level3, createStraightenArmFirstCommand(ArmState.Level3)) ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.Level4, createStraightenArmFirstCommand(ArmState.Level4)) ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.WaitingForCoral, createStraightenArmFirstCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.StartAlgaePosition, new NoOpCommand()) ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.Running, createStraightenArmFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.StartAlgaePosition).put(ArmState.Climb, createStraightenArmFirstCommand(ArmState.Climb)) ;

        // transitions from StartAlgaePosition
        transitionCommands.put(ArmState.Climb, new HashMap<ArmState, Command>() )  ;
        transitionCommands.get(ArmState.Climb).put(ArmState.Level1, createStraightenArmFirstCommand(ArmState.Level1)) ;
        transitionCommands.get(ArmState.Climb).put(ArmState.Level2, createStraightenArmFirstCommand(ArmState.Level2)) ;
        transitionCommands.get(ArmState.Climb).put(ArmState.Level3, createStraightenArmFirstCommand(ArmState.Level3)) ;
        transitionCommands.get(ArmState.Climb).put(ArmState.Level4, createStraightenArmFirstCommand(ArmState.Level4)) ;
        transitionCommands.get(ArmState.Climb).put(ArmState.WaitingForCoral, createStraightenArmFirstCommand(ArmState.WaitingForCoral)) ;
        transitionCommands.get(ArmState.Climb).put(ArmState.StartAlgaePosition, createStraightenArmFirstCommand(ArmState.StartAlgaePosition)) ;
        transitionCommands.get(ArmState.Climb).put(ArmState.Running, createStraightenArmFirstCommand(ArmState.Running)) ;
        transitionCommands.get(ArmState.Climb).put(ArmState.Climb, new NoOpCommand()) ;


    }

}
