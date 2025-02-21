package frc.robot.leds;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;

public class ASCIInimation
{
    static AddressableLEDBuffer ledBuffer;
    private final Color[] bits;

    public ASCIInimation(String text, Color lo, Color hi, Color buf, Color wrd)
    {
        final int tlen9 = text.length() * 9;
        bits = new Color[tlen9 + 1];

        bits[0]     = wrd;
        bits[tlen9] = wrd;

        int idx = 1;
        for(char c : text.toCharArray()) 
        {
            int ci = c;
            for(int i = 0; i < 8; i++)
            {
                bits[idx++] = (ci & 1) == 0 ? lo : hi;
                ci >>= 1;
            }

            if(idx != tlen9)
            {
                bits[idx++] = buf;
            }
        }
    }

    
    public void periodic()
    {
        Timer timer = new Timer();
        final int offset = (int)(timer.get() / 0.1);

        for(int i = 0; i < ledBuffer.getLength(); i++)
        {
            final int x = (i + offset) % ledBuffer.getLength();
            final Color c = bits[x % bits.length];

            ledBuffer.setLED(i, c);
        }
    }
}
