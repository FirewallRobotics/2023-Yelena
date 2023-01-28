package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.hal.AddressableLEDJNI;
// import edu.wpi.first.hal.simulation.AddressableLEDDataJNI;
// import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LEDSubsystem extends SubsystemBase {
    
    // All the steps to initialization can be found in the WPILib Documentation on AddressableLEDs

    
    // Set up variables for LEDs
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    
    // Variables specific to LightScroll()
    private int timer;
    private int scroll_offset;

    // Class contructor (runs once at the start)
    public LEDSubsystem() {

        // Assign PWM port from RoboRIO, previous code used port 1
        m_led = new AddressableLED(1);

        // Assign length (number of LEDs being changed)
        m_ledBuffer = new AddressableLEDBuffer(27);

        // Transfers length from m_ledbuffer to m_led
        m_led.setLength(m_ledBuffer.getLength());

        // Sets LED output data
        m_led.setData(m_ledBuffer);

        // Writes the LED output continuously
        m_led.start();


        // Starting values for variables used in LightScroll()
        timer = 0;
        scroll_offset = 7;
    }

    // Used within code to set basic colors
    private void SetLights(int rgb_red, int rgb_green, int rgb_blue) {
        
        // Sets all LEDs to RGB values
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, rgb_red, rgb_green, rgb_blue);
        }

        // Applies the m_ledbuffer data to the m_led object
        m_led.setData(m_ledBuffer);
    }
    

    // The following methods are for controlling the light displayed by the LEDs 

    public void LightTest() {

        // The way that color information is transmitted to the LEDs is by first assigning the colors to m_ledbuffer
        // and then using m_led.setData(m_ledbuffer); to assign every value to the actual led object.

        
        // Set RGB values (these are for the "Firewall Purple")

        // Simulator RGB (need to be brighter)
        int rgb_red = 150;
        int rgb_green = 0;
        int rgb_blue = 150;

        // Robot RGB (need to be darker)
        /*
        int rgb_red = 15;
        int rgb_green = 0;
        int rgb_blue = 15;
        */
   
        SetLights(rgb_red, rgb_green, rgb_blue);
    }
   
    public void LightOff() {

        // Sets RGB values to 0, turning them off
        SetLights(0, 0, 0);
    }

    public void WarningLight() {

        // Displays red to indicate something is wrong

        // Simulator RGB (needs to be brighter)
        int rgb_red = 200;
        int rgb_green = 0;
        int rgb_blue = 0;

        // Robot RGB (needs to be darker)
        /*
        int rgb_red = 20;
        int rgb_green = 0;
        int rgb_blue = 0;
        */
   
        SetLights(rgb_red, rgb_green, rgb_blue);
    }

    public void ProximityLight(int proximity) {

        // Displays from blue to light teal to indicate proximity to a goal (i.e. distance from scoring)
        

        // Simulator RGB (needs to be brighter)
        int rgb_red = 0;
        int rgb_green = 10; 
        int rgb_blue = 200;

        // Robot RGB (needs to be darker)
        /*
        int rgb_red = 0;
        int rgb_green = 1; 
        int rgb_blue = 20;
        */

        // NOTE: The rgb_green will be multipied by a number from 0-24

        if (proximity < 0) proximity = 0;
        if (proximity > 25) proximity = 25;



        
        if (proximity < 24) {
            SetLights(rgb_red, rgb_green * proximity, rgb_blue);
        }
        else {
            ReadyLight();
        }

    }

    public void ReadyLight() {

        // Displays green to indicate the robot is ready to perform an action (i.e. drop the game piece)

        // Simulator RGB (needs to be brighter)
        int rgb_red = 0;
        int rgb_green = 200;
        int rgb_blue = 0;

        // Robot RGB (needs to be darker)
        /*
        int rgb_red = 0;
        int rgb_green = 20;
        int rgb_blue = 0;
        */
   
        SetLights(rgb_red, rgb_green, rgb_blue);
    }

    public void LightScroll() {
        
        // Meant to have scrolling lights for asthetic; run as default command

        // Set RGB values (these are for the "Firewall Purple")
        int rgb_red = 50;
        int rgb_green = 0;
        int rgb_blue = 50;

        // Set RGB values (these are for the "Firewall Purple")
        /*
        int rgb_red = 5;
        int rgb_green = 0;
        int rgb_blue = 5;
        */

        // NOTE: For the lightscroll effect, the rgb values start 3x smaller because I multiply it by either 1, 2, or 3 to change brightness
        

        // Timer increments every iteration (20ms) and scroll offset decrements every 10 iterations (200ms)
        timer++;

        if (timer == 10) {
            timer = 0;
            scroll_offset--;
        }

        // Scroll offset loop (starts at 6 and ends at 1, looping back at 0)
        if (scroll_offset == 0) scroll_offset = 6;

        // Turns on half of the lights and turns off other half based on the offset
        // Brightness is found by taking the remainder of (Scroll offset + LED position) / 6, and multiplying by that value
        // only if it is less than 4. This means brightness is only ever multiplied by 1, 2, or 3
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if (((scroll_offset + i) % 6) < 4) {
                m_ledBuffer.setRGB(i, rgb_red * ((scroll_offset + i) % 6), rgb_green * (scroll_offset + i), rgb_blue * ((scroll_offset + i) % 6));
            }
            else{
                m_ledBuffer.setRGB(i, 0, 0, 0);
                // Turns off light
            }
        }

        // Applies the m_ledbuffer data to the m_led object
        m_led.setData(m_ledBuffer);

    }
}




