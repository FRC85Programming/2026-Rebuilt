package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private static final int PWM_PORT  = 9;
    private static final int LED_COUNT = 19;

    private static final double BRIGHTNESS = 0.4;

    private final AddressableLED       m_led;
    private final AddressableLEDBuffer m_buffer;

    public enum Animation {
        IDLE, AUTO, RED_ALLIANCE, BLUE_ALLIANCE, LIGHTNING, BUILT_ON_BRAINS,
        BLINK_GREEN, BLINK_WHITE, ALLIANCE_SPECIFIC, BAD_BATTERY
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

    // Red alliance flare state — per-LED brightness boost that decays each tick
    private final double[] m_redFlare = new double[LED_COUNT];

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
            case BUILT_ON_BRAINS -> animateBuiltOnBrains();
            case BLINK_GREEN     -> animateBlinkGreen();
            case BLINK_WHITE     -> animateBlinkWhite();
            case ALLIANCE_SPECIFIC -> animateAllianceSpecific();
            case BAD_BATTERY       -> animateBadBattery();
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

    /**
     * Red Alliance: base color ~#ff1440 (bright crimson). A fast traveling wave
     * moves down the strip while a slower global breathe modulates overall brightness.
     * Random flares occasionally spike individual LEDs to near-white-red for life.
     */
    private void animateRedAlliance() {
        // Base color target: R=255, G=20, B=60 (bright crimson, slightly above #e50f33)
        final double BASE_R = 255, BASE_G = 20, BASE_B = 60;

        // Slow global breathe: dims the whole strip gently (0.65 – 1.0 range)
        double breathe = 0.65 + 0.35 * ((Math.sin(m_tick * 0.05) + 1.0) * 0.5);

        // Fast traveling wave moving down the strip (0.0 – 1.0 per LED)
        // Creates a bright crest that sweeps continuously
        double waveSpeed = 0.18;
        double waveFreq  = 0.55;

        // Randomly ignite a flare on a single LED every ~20 ticks on average
        if (m_rng.nextInt(20) == 0) {
            int idx = m_rng.nextInt(LED_COUNT);
            m_redFlare[idx] = 1.0;
        }

        for (int i = 0; i < LED_COUNT; i++) {
            // Traveling wave: value 0.0–1.0, crest moves over time
            double wave = (Math.sin(i * waveFreq - m_tick * waveSpeed) + 1.0) * 0.5;

            // Secondary slower counter-wave for depth/interference pattern
            double wave2 = (Math.sin(i * 0.25 + m_tick * 0.07) + 1.0) * 0.5;

            // Blend: base + wave boost + secondary wave
            double intensity = breathe * (0.55 + 0.30 * wave + 0.15 * wave2);
            intensity = clamp(intensity + m_redFlare[i] * 0.5);

            int r = scale((int) clamp(intensity * BASE_R, 0, 255));
            int g = scale((int) clamp(intensity * BASE_G + m_redFlare[i] * 40, 0, 255));
            int b = scale((int) clamp(intensity * BASE_B * 0.4 + m_redFlare[i] * 20, 0, 255));

            m_buffer.setRGB(i, r, g, b);

            // Decay flare
            m_redFlare[i] = Math.max(0, m_redFlare[i] - 0.06);
        }
    }

    /**
     * Blue Alliance: a single bright comet that chases back and forth across
     * the strip, leaving a short fading trail of deep blue behind it.
     */
    private void animateBlueAlliance() {
        int period = LED_COUNT * 2;
        int pos = m_tick % period;
        // Bounce: go 0→N-1, then N-1→0
        if (pos >= LED_COUNT) pos = period - 1 - pos;

        final int TRAIL = 6;

        for (int i = 0; i < LED_COUNT; i++) {
            int dist = Math.abs(i - pos);

            if (dist == 0) {
                // Comet head: bright white-blue
                m_buffer.setRGB(i, scale(160), scale(200), scale(255));
            } else if (dist <= TRAIL) {
                // Trail: fades from blue to dark blue
                double t = 1.0 - (double) dist / TRAIL;
                m_buffer.setRGB(i,
                    0,
                    scale((int)(60 * t * t)),
                    scale((int)(200 * t)));
            } else {
                // Background: very dim blue
                m_buffer.setRGB(i, 0, 0, scale(18));
            }
        }
    }

    /**
     * Bad Battery: slow, ominous dark-red blink. Spends most of its time nearly
     * off, then ramps up to a dim crimson over ~0.5s and fades back down. The
     * sine-based envelope gives it a pulse feel rather than a hard on/off flash.
     */
    private void animateBadBattery() {
        // Period: ~150 ticks (~3s at 50Hz). Sine squared keeps it dark most of the cycle.
        double phase = (m_tick % 150) / 150.0 * 2 * Math.PI;
        double pulse = Math.pow(Math.max(0, Math.sin(phase)), 3);

        int r = scale((int)(pulse * 180));   // dark crimson — never bright
        int g = 0;
        int b = 0;

        for (int i = 0; i < LED_COUNT; i++) {
            m_buffer.setRGB(i, r, g, b);
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