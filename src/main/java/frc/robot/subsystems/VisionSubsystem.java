// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera camera = new PhotonCamera(Constants.USB_CAMERA_NAME); // Kamera Adı 
  boolean hasTarget; // Bir hedefin algılanıp algılanmadığını saklar
  PhotonPipelineResult result; // Photonvision'ın döndürdüğü tüm verileri depolar
  public VisionSubsystem() {}

  // Apriltag belirtilen hedef kimliği (varsa) döndürür
  public PhotonTrackedTarget getTargetWithID(int id) { 
    List<PhotonTrackedTarget> targets = result.getTargets(); // İzlenen tüm hedeflerin bir listesini oluşturun
    for (PhotonTrackedTarget i : targets) {
        if (i.getFiducialId() == id) { // Listedeki her hedefin kimliğini kontrol edin
            return i; // Belirtilen kimliğe sahip hedefi buldum!
        }
    }
    return null; // Belirtilen kimliğe sahip hedef bulunamadı
  }

  public PhotonTrackedTarget getBestTarget() {
    if (hasTarget) {
    return result.getBestTarget(); // En iyi (en yakın) hedefi verir
    }
    else {
        return null; // Aksi takdirde, hiçbir hedef bulunamazsa null değerini döndürür.
    }
  }
  public boolean getHasTarget() {
    return hasTarget; // Bir hedefin bulunup bulunmadığını döndürür
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult result = camera.getLatestResult(); // PhotonVision'dan en son sonucu sorgulayın
    hasTarget = result.hasTargets(); // Kamera bir apriltag hedefi tespit ederse, hasTarget boole değeri true olur
    if (hasTarget) {
        this.result = result;
    }
  }
}
