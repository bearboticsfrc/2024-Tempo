package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.LightsConstants;

public class LightsSubsystem extends LightsConstants implements Subsystem {
  private static String strip;
  private static String color;
  private static byte[] data = new byte[10];

  public LightsSubsystem() {
    super();
    strip = "one";
    color = "blank";
    send();
  }
  public void massSignal(String c, String[] Strips){
    for(int i = 0; i < data.length / 2; i++){
      setAll(i ,Strips[i], c);
    }
    LightsConstants.communincation.writeBulk(data, data.length);
  }

  public void send() {
    data[0] = (byte)((char)LightsConstants.strip.get(strip));
    data[1] = (byte)((char)LightsConstants.color.get(color));
    boolean failure = LightsConstants.communincation.writeBulk(data, data.length);

    if (failure){
      DataLogManager.log("led failure has occured");
    } else {
      LightsConstants.communincation.writeBulk(data, data.length);
    }
  }

  public void setColor(String c) {
    color = c;
    send();
  }
  public void setAll(int index, String s, String c){
    data[index * 2] = (byte)((char)LightsConstants.strip.get(s));
    data[index + 1] = (byte)((char)LightsConstants.color.get(c));
  }

  public void setStrip(String s) {
    strip = s;
    send();
  }

  public void all(){
    data[0] = ((byte)(((char)LightsConstants.color.get(color))));
    LightsConstants.communincation.writeBulk(data,data.length - 9);    
  }
}
