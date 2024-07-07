package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(38);
    private final AddressableLED m_led = new AddressableLED(9);
    private final BooleanSupplier m_hasNote;


    public LEDs(BooleanSupplier hasNote){
        m_hasNote = hasNote;
        m_led.setLength(m_buffer.getLength());
        m_buffer.setLED(0, Color.kBlue);
        m_led.setData(m_buffer);
        m_led.start();
        setRRBlue();
    }

    @Override
    public void periodic() {
        if(m_hasNote.getAsBoolean()){
            setNoteOrange();
        }
        else{
            setRRBlue();
        }
    }

    public void setRRBlue(){
        for (var i = 0; i < m_buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_buffer.setLED(i, Color.kBlue);
         }
         m_led.setData(m_buffer);
    }
    public void setNoteOrange(){
        for (var i = 0; i < m_buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_buffer.setLED(i, Color.kOrangeRed);
         }
         m_led.setData(m_buffer);
    }
    
}
