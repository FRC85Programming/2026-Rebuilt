package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static final int PWM_PORT  = 9;
    private static final int LED_COUNT = 30;

    private static final double BRIGHTNESS = 0.4;

    private final AddressableLED       m_led;
    private final AddressableLEDBuffer m_buffer;

    public enum Animation {
        IDLE, AUTO, RED_ALLIANCE, BLUE_ALLIANCE, LIGHTNING, BUILT_ON_BRAINS,
        BLINK_GREEN, BLINK_WHITE, ALLIANCE_SPECIFIC
    }

    private Animation m_current = Animation.IDLE;
    private int m_tick = 0;

    private static final int BLINK_HALF_PERIOD = 13;

    private final double[] m_boltGlow = new double[LED_COUNT];
    private int m_boltCooldown = 0;
    private final java.util.Random m_rng = new java.util.Random();

    private static final int BOB_MAX_PULSES = 6;
    private final double[][] m_bobPulses = new double[BOB_MAX_PULSES][4];
    private final boolean[]  m_bobActive = new boolean[BOB_MAX_PULSES];
    private int m_bobFireCd = 0;

    private Animation allianceAnimation = Animation.BLUE_ALLIANCE;

    public LEDSubsystem() {
        m_led = new AddressableLED(PWM_PORT);
        m_buffer = new AddressableLEDBuffer(LED_COUNT);
        m_led.setLength(LED_COUNT);
        m_led.setData(m_buffer);
        m_led.start();
    }

    // 🔧 Scale helper
    private int scale(int value) {
        return (int)(value * BRIGHTNESS);
    }

    public void setAnimation(Animation anim) {
        if (m_current != anim) {
            m_current = anim;
            m_tick = 0;
        }
    }

    public Animation getAnimation() { return m_current; }

    @Override
    public void periodic() {
        switch (m_current) {
            case IDLE            -> animateIdle();
            case AUTO            -> animateAuto();
            case RED_ALLIANCE    -> animateRedAlliance();
            case BLUE_ALLIANCE   -> animateBlueAlliance();
            case LIGHTNING       -> animateLightning();
            case BUILT_ON_BRAINS -> animateBuiltOnBrains();
            case BLINK_GREEN     -> animateBlinkGreen();
            case BLINK_WHITE     -> animateBlinkWhite();
            case ALLIANCE_SPECIFIC -> animateAllianceSpecific();
        }
        m_led.setData(m_buffer);
        m_tick++;
    }

    private void animateIdle() {
        for (int i = 0; i < LED_COUNT; i++) {
            int hue = (m_tick * 2 + i * (180 / LED_COUNT)) % 180;
            m_buffer.setHSV(i, hue, 255, (int)(200 * BRIGHTNESS));
        }
    }

    private void animateAuto() {
        int period = LED_COUNT * 2;
        int pos = m_tick % period;
        if (pos >= LED_COUNT) pos = period - 1 - pos;

        for (int i = 0; i < LED_COUNT; i++) {
            int dist = Math.abs(i - pos);
            int trail = 8;

            if (dist == 0) {
                m_buffer.setRGB(i, scale(255), scale(255), scale(255));
            } else if (dist <= trail) {
                double t = 1.0 - (double) dist / trail;
                m_buffer.setRGB(i,
                    scale((int)(0 * t)),
                    scale((int)(220 * t)),
                    scale((int)(255 * t)));
            } else {
                m_buffer.setRGB(i, 0, 0, scale(20));
            }
        }
    }

    private void animateRedAlliance() {
        for (int i = 0; i < LED_COUNT; i++) {
            double wave  = Math.sin((i * 0.5) - (m_tick * 0.12));
            double wave2 = Math.sin((i * 0.3) - (m_tick * 0.07) + 1.2);
            double intensity = clamp((wave + wave2 + 2.0) / 4.0);
            double flicker = (Math.sin(i * 73.4 + m_tick * 0.8 + i) + 1) * 0.5;

            int r = scale((int) clamp(intensity * 255 + flicker * 30, 0, 255));
            int g = scale((int) clamp(intensity * intensity * 160 + flicker * 20, 0, 255));
            int b = scale((int) clamp(flicker * 10, 0, 40));

            m_buffer.setRGB(i, r, g, b);
        }
    }

    private void animateBlueAlliance() {
        for (int i = 0; i < LED_COUNT; i++) {
            double wave  = Math.sin((i * 0.4) - (m_tick * 0.10));
            double wave2 = Math.sin((i * 0.7) - (m_tick * 0.06) + 2.0);
            double intensity = clamp((wave + wave2 + 2.0) / 4.0);

            double sparkle = (Math.sin(i * 53.1 + m_tick * 1.1) + 1) * 0.5;
            boolean isCrest = intensity > 0.75 && sparkle > 0.6;

            int r, g, b;
            if (isCrest) {
                r = scale((int)(sparkle * 100));
                g = scale((int)(180 + sparkle * 60));
                b = scale(255);
            } else {
                r = 0;
                g = scale((int)(intensity * 80));
                b = scale((int)(80 + intensity * 175));
            }

            m_buffer.setRGB(i, r, g, b);
        }
    }

    private void animateLightning() {
        if (m_boltCooldown <= 0) {
            m_boltCooldown = 8 + m_rng.nextInt(33);
            int center = m_rng.nextInt(LED_COUNT);

            for (int i = 0; i < LED_COUNT; i++) {
                double dist = Math.abs(i - center);
                double flash = Math.exp(-dist * 0.55);
                m_boltGlow[i] = Math.max(m_boltGlow[i], flash);
            }
        } else {
            m_boltCooldown--;
        }

        for (int i = 0; i < LED_COUNT; i++) {
            double g = m_boltGlow[i];

            int r = scale((int)(g * g * 255));
            int gr2 = scale((int)(g * g * 200));
            int b = scale((int)(Math.min(1, g * 1.5) * 255));

            m_buffer.setRGB(i, r, gr2, b);

            m_boltGlow[i] = Math.max(0, g - 0.025 - g * 0.045);
        }
    }

    private void animateBuiltOnBrains() {
        double breath = (Math.sin(m_tick * 0.063) + 1.0) * 0.5;

        for (int i = 0; i < LED_COUNT; i++) {
            int baseR = 0;
            int baseG = scale((int)(15 + breath * 18));
            int baseB = scale((int)(80 + breath * 90));
            m_buffer.setRGB(i, baseR, baseG, baseB);
        }
    }

    private void animateBlinkGreen() {
        boolean on = (m_tick % (BLINK_HALF_PERIOD * 2)) < BLINK_HALF_PERIOD;
        int value = on ? scale(255) : 0;
        for (int i = 0; i < LED_COUNT; i++) {
            m_buffer.setRGB(i, 0, value, 0);
        }
    }

    private void animateBlinkWhite() {
        boolean on = (m_tick % (BLINK_HALF_PERIOD * 2)) < BLINK_HALF_PERIOD;
        int value = on ? scale(255) : 0;
        for (int i = 0; i < LED_COUNT; i++) {
            m_buffer.setRGB(i, value, value, value);
        }
    }

    private static double clamp(double v) { return Math.max(0, Math.min(1, v)); }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void setAllianceAnimation(Animation animation) {
        allianceAnimation = animation;
    }

    private void animateAllianceSpecific() {
        setAnimation(allianceAnimation);
    }
}