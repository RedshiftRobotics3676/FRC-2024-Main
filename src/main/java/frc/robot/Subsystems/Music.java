// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Orchestra;


public class Music extends SubsystemBase {
  Orchestra orch = new Orchestra();

  /* enum song {Mario, PartyRockAnthem, LastChristmas, AllIWantForChristmasIsYou}
  HashMap<song, String> songMap = new HashMap<song, String>(); */
  
  SendableChooser<String> chooser;
  
  /** Creates a new Music. */
  public Music(CommandSwerveDrivetrain drivetrain) {
    orch.addInstrument(drivetrain.getModule(0).getSteerMotor());
    orch.addInstrument(drivetrain.getModule(0).getDriveMotor());
    orch.addInstrument(drivetrain.getModule(1).getSteerMotor());
    orch.addInstrument(drivetrain.getModule(1).getDriveMotor());
    orch.addInstrument(drivetrain.getModule(2).getSteerMotor());
    orch.addInstrument(drivetrain.getModule(2).getDriveMotor());
    orch.addInstrument(drivetrain.getModule(3).getSteerMotor());
    orch.addInstrument(drivetrain.getModule(3).getDriveMotor());

    /* songMap.put(song.Mario, "mario.chrp");
    songMap.put(song.PartyRockAnthem, "party rock anthem.chrp");
    songMap.put(song.LastChristmas, "last christmas.chrp");
    songMap.put(song.AllIWantForChristmasIsYou, "all i want for christmas is you.chrp"); */

    chooser = new SendableChooser<String>();
    chooser.setDefaultOption("Mario", "songs\\mario.chrp");
    chooser.addOption("Party Rock Anthem", "songs\\party rock anthem.chrp");
    chooser.addOption("Bohemian Rhapsody","songs\\bohemian rhapsody.chrp");
    chooser.addOption("Harder Better Faster Stronger","songs\\harder better faster stronger.chrp");
    chooser.addOption("Sandstorm","songs\\sandstorm.chrp");
    chooser.addOption("Seven Nation Army","songs\\seven nation army.chrp");
    chooser.addOption("Super Mario 64","songs\\super mario 64.chrp");
    chooser.addOption("Viva La Vida","songs\\viva la vida.chrp");
    
    // SmartDashboard.putData(chooser);
  }

  /* public void loadSong(song song) {
    orch.loadMusic(songMap.get(song));
  } */

  public InstantCommand loadFromChooser() {
    return new InstantCommand(() -> orch.loadMusic(chooser.getSelected()));
  }

  public InstantCommand playOrPause() {
    if (orch.isPlaying()) {
      return new InstantCommand(() -> orch.pause());
    }
    else {
      return new InstantCommand(() -> orch.play());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
