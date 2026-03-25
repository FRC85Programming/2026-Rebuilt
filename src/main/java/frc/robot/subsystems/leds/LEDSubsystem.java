package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED Subsystem for FRC robot using PWM addressable LEDs (e.g. WS2812B).
 *
 * Animations:
 *   IDLE        – Rainbow ripple flowing along the strip
 *   AUTO        – Cyan-to-white pulse that sweeps forward like a scanner
 *   RED_ALLIANCE – Fiery red/orange lava flow with ember highlights
 *   BLUE_ALLIANCE – Deep blue ocean wave with aqua crests
 *   LIGHTNING      – Violent white/violet superbolts with rumbling purple afterglow
 *   BUILT_ON_BRAINS – Team 85 B.O.B. gold neural pulses: synapses firing across the strip
 *
 * All animations are time-based (no busy-waits) and driven by periodic().
 */
public class LEDSubsystem extends SubsystemBase {

    // ── Hardware ─────────────────────────────────────────────────────────────
    private static final int PWM_PORT    = 9;   // Change to your PWM port
    private static final int LED_COUNT   = 30;

    private final AddressableLED       m_led;
    private final AddressableLEDBuffer m_buffer;

    // ── Animation state ───────────────────────────────────────────────────────
    public enum Animation { IDLE, AUTO, RED_ALLIANCE, BLUE_ALLIANCE, LIGHTNING, BUILT_ON_BRAINS }

    private Animation m_current = Animation.IDLE;

    /** Tick counter incremented every periodic() call (~50 Hz). */
    private int m_tick = 0;

    // Lightning animation state
    private final double[] m_boltGlow   = new double[LED_COUNT]; // per-pixel afterglow 0–1
    private int  m_boltCooldown = 0;   // ticks until next bolt is allowed
    private final java.util.Random m_rng = new java.util.Random();

    // Built On Brains (BOB) animation state
    // Each synapse: [origin pixel, direction (+1/-1), current position (float), brightness]
    private static final int  BOB_MAX_PULSES = 6;
    private final double[][] m_bobPulses     = new double[BOB_MAX_PULSES][4]; // [pos, dir, age, brightness]
    private final boolean[]  m_bobActive     = new boolean[BOB_MAX_PULSES];
    private int              m_bobFireCd     = 0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public LEDSubsystem() {
        m_led    = new AddressableLED(PWM_PORT);
        m_buffer = new AddressableLEDBuffer(LED_COUNT);
        m_led.setLength(LED_COUNT);
        m_led.setData(m_buffer);
        m_led.start();
    }

    // ── Public API ────────────────────────────────────────────────────────────
    public void setAnimation(Animation anim) {
        if (m_current != anim) {
            m_current = anim;
            m_tick = 0;          // restart animation from beginning
        }
    }

    public Animation getAnimation() { return m_current; }

    // ── Periodic ──────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        switch (m_current) {
            case IDLE           -> animateIdle();
            case AUTO           -> animateAuto();
            case RED_ALLIANCE   -> animateRedAlliance();
            case BLUE_ALLIANCE  -> animateBlueAlliance();
            case LIGHTNING      -> animateLightning();
            case BUILT_ON_BRAINS-> animateBuiltOnBrains();
        }
        m_led.setData(m_buffer);
        m_tick++;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  IDLE – full-spectrum rainbow that flows smoothly along the strip
    // ═════════════════════════════════════════════════════════════════════════
    private void animateIdle() {
        // Each LED is offset in hue; hue itself advances with tick
        // Speed: ~1 full rainbow scroll per 3.6 seconds at 50 Hz
        for (int i = 0; i < LED_COUNT; i++) {
            int hue = (m_tick * 2 + i * (180 / LED_COUNT)) % 180;
            m_buffer.setHSV(i, hue, 255, 200);
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  AUTO – bright cyan scanner pulse on a dark navy background
    //         Pulse head is pure white; tail fades to cyan then navy.
    // ═════════════════════════════════════════════════════════════════════════
    private void animateAuto() {
        // Pulse bounces back and forth (ping-pong)
        int period = LED_COUNT * 2;           // full round-trip
        int pos    = m_tick % period;
        if (pos >= LED_COUNT) pos = period - 1 - pos;   // reverse

        for (int i = 0; i < LED_COUNT; i++) {
            int dist  = Math.abs(i - pos);
            int trail = 8;                    // tail length in pixels

            if (dist == 0) {
                // Bright white head
                m_buffer.setRGB(i, 255, 255, 255);
            } else if (dist <= trail) {
                // Cyan fade-out trail
                double t = 1.0 - (double) dist / trail;
                int r = (int)(0   * t);
                int g = (int)(220 * t);
                int b = (int)(255 * t);
                m_buffer.setRGB(i, r, g, b);
            } else {
                // Dark navy base
                m_buffer.setRGB(i, 0, 0, 20);
            }
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  RED ALLIANCE – lava flow: deep red base with rolling orange/yellow
    //                 embers that flicker individually
    // ═════════════════════════════════════════════════════════════════════════
    private void animateRedAlliance() {
        for (int i = 0; i < LED_COUNT; i++) {
            // Flowing sine wave creates lava "blobs"
            double wave  = Math.sin((i * 0.5) - (m_tick * 0.12));
            double wave2 = Math.sin((i * 0.3) - (m_tick * 0.07) + 1.2);

            // Combined 0–1 intensity
            double intensity = clamp((wave + wave2 + 2.0) / 4.0);

            // Ember flicker: small fast noise per pixel
            double flicker = (Math.sin(i * 73.4 + m_tick * 0.8 + i) + 1) * 0.5;

            // Map intensity to red→orange→yellow
            int r = (int) clamp(intensity * 255 + flicker * 30, 0, 255);
            int g = (int) clamp(intensity * intensity * 160 + flicker * 20, 0, 255);
            int b = (int) clamp(flicker * 10, 0, 40);

            m_buffer.setRGB(i, r, g, b);
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  BLUE ALLIANCE – deep ocean wave: midnight blue with rolling aqua crests
    // ═════════════════════════════════════════════════════════════════════════
    private void animateBlueAlliance() {
        for (int i = 0; i < LED_COUNT; i++) {
            // Two overlapping waves for a realistic ocean feel
            double wave  = Math.sin((i * 0.4) - (m_tick * 0.10));
            double wave2 = Math.sin((i * 0.7) - (m_tick * 0.06) + 2.0);

            double intensity = clamp((wave + wave2 + 2.0) / 4.0);

            // Foam/crest sparkle
            double sparkle = (Math.sin(i * 53.1 + m_tick * 1.1) + 1) * 0.5;
            boolean isCrest = intensity > 0.75 && sparkle > 0.6;

            int r, g, b;
            if (isCrest) {
                // White-aqua foam
                r = (int)(sparkle * 100);
                g = (int)(180 + sparkle * 60);
                b = 255;
            } else {
                // Deep blue body
                r = 0;
                g = (int)(intensity * 80);
                b = (int)(80 + intensity * 175);
            }

            m_buffer.setRGB(i, clampI(r), clampI(g), clampI(b));
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  LIGHTNING – violent superbolts with branching and purple afterglow
    //
    //  Each "strike" event:
    //    1. Picks a random center pixel.
    //    2. Radiates a sharp white/violet flash outward with exponential falloff.
    //    3. Optionally fires a secondary "branch" bolt offset from the first.
    //    4. Glow array decays each tick → deep violet rumble between strikes.
    //    5. A dim storm base (dark indigo) is always present underneath.
    // ═════════════════════════════════════════════════════════════════════════
    private void animateLightning() {
        // ── Fire a new bolt? ──────────────────────────────────────────────────
        if (m_boltCooldown <= 0) {
            // Randomise gap: 8–40 ticks (0.16 s – 0.8 s)
            m_boltCooldown = 8 + m_rng.nextInt(33);

            // Primary bolt
            int center = m_rng.nextInt(LED_COUNT);
            for (int i = 0; i < LED_COUNT; i++) {
                double dist  = Math.abs(i - center);
                double flash = Math.exp(-dist * 0.55);   // sharp exponential falloff
                m_boltGlow[i] = Math.max(m_boltGlow[i], flash);
            }

            // ~55% chance of a secondary branch bolt
            if (m_rng.nextDouble() < 0.55) {
                int branch = clampI(center + m_rng.nextInt(13) - 6);
                branch = Math.max(0, Math.min(LED_COUNT - 1, branch));
                for (int i = 0; i < LED_COUNT; i++) {
                    double dist  = Math.abs(i - branch);
                    double flash = Math.exp(-dist * 0.9) * 0.65;  // dimmer branch
                    m_boltGlow[i] = Math.max(m_boltGlow[i], flash);
                }
            }
        } else {
            m_boltCooldown--;
        }

        // ── Render each pixel ─────────────────────────────────────────────────
        for (int i = 0; i < LED_COUNT; i++) {
            double g = m_boltGlow[i];

            // Storm base: dark rolling indigo (always present)
            double stormWave = (Math.sin(i * 0.35 - m_tick * 0.05) + 1) * 0.5;
            int baseR = (int)(stormWave * 12);
            int baseG = 0;
            int baseB = (int)(20 + stormWave * 25);

            // Glow layer: white core → violet afterglow → purple rumble
            // g=1 → pure white; g=0.5 → bright violet; g=0 → nothing
            int glowR = (int)(g * g * 255);               // white only at peak
            int glowG = (int)(g * g * 200);               // white fades first
            int glowB = (int)(Math.min(1, g * 1.5) * 255); // blue/violet lingers

            int r = clampI(baseR + glowR);
            int gr2 = clampI(baseG + glowG);
            int b = clampI(baseB + glowB);

            m_buffer.setRGB(i, r, gr2, b);

            // Decay glow (slower near 0 for a rumble tail)
            m_boltGlow[i] = Math.max(0, g - 0.025 - g * 0.045);
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  BUILT ON BRAINS (Team 85 B.O.B.) – gold neural synapse pulses
    //
    //  Inspired by Team 85's gold & blue color scheme and "Built On Brains" motto.
    //
    //  A deep royal-blue breathing baseline (the "resting brain") pulses slowly.
    //  Every 6–18 ticks, a new synapse fires from a random node:
    //    • A bright gold pulse head shoots outward in both directions.
    //    • The head flashes white-gold at peak, fading to warm gold then nothing.
    //    • The blue baseline ripples gently underneath, like an active mind.
    //    • Multiple pulses coexist, overlapping additively, like real neural firing.
    //    • Occasionally a "burst" fires 2 pulses simultaneously (big idea moment).
    // ═════════════════════════════════════════════════════════════════════════
    private void animateBuiltOnBrains() {
        // BOB gold & blue palette (RGB targets):
        //   Gold peak : 255, 210,  10  (bright white-gold)
        //   Gold mid  : 220, 140,   0  (warm gold)
        //   Blue base :   0,  20, 100  (deep royal blue, breathing to 0, 40, 180)

        // ── Spawn new pulses ─────────────────────────────────────────────────
        if (m_bobFireCd <= 0) {
            m_bobFireCd = 20 + m_rng.nextInt(30);  // fire every 0.40–1.0 s

            // How many to spawn this event (burst chance)
            int count = (m_rng.nextDouble() < 0.2) ? 2 : 1;

            for (int s = 0; s < count; s++) {
                int origin = m_rng.nextInt(LED_COUNT);
                // Fire left AND right from origin
                for (int dir : new int[]{-1, 1}) {
                    for (int slot = 0; slot < BOB_MAX_PULSES; slot++) {
                        if (!m_bobActive[slot]) {
                            m_bobPulses[slot][0] = origin;          // position
                            m_bobPulses[slot][1] = dir;             // direction
                            m_bobPulses[slot][2] = 0;               // age
                            m_bobPulses[slot][3] = 0.85 + m_rng.nextDouble() * 0.15; // brightness
                            m_bobActive[slot] = true;
                            break;
                        }
                    }
                }
            }
        } else {
            m_bobFireCd--;
        }

        // ── Breathing royal-blue baseline ────────────────────────────────────
        // Slow 2 s breath cycle
        double breath = (Math.sin(m_tick * 0.063) + 1.0) * 0.5;  // 0–1

        // ── Accumulate pulse contributions into a glow buffer ────────────────
        double[] glowR = new double[LED_COUNT];
        double[] glowG = new double[LED_COUNT];
        double[] glowB = new double[LED_COUNT];

        for (int slot = 0; slot < BOB_MAX_PULSES; slot++) {
            if (!m_bobActive[slot]) continue;

            double pos   = m_bobPulses[slot][0];
            double age   = m_bobPulses[slot][2];
            double brite = m_bobPulses[slot][3];

            // Pulse head with a 3-pixel soft glow halo
            for (int i = 0; i < LED_COUNT; i++) {
                double dist = Math.abs(i - pos);
                if (dist > 4.0) continue;
                double falloff = Math.exp(-dist * 1.2) * brite;

                // White-gold at fresh ages, pure gold as it fades
                double ageFade = Math.max(0, 1.0 - age / 32.0);
                glowR[i] += falloff * (210 + 45 * ageFade);   // 210→255 when fresh
                glowG[i] += falloff * (100 + 110 * ageFade);  // 100→210 when fresh
                glowB[i] += falloff * (30  * ageFade);         // slight white shimmer at peak
            }

            // Advance pulse
            m_bobPulses[slot][0] += m_bobPulses[slot][1] * 0.4;  // ~0.4 LEDs/tick (half speed)
            m_bobPulses[slot][2]++;

            // Deactivate when off-strip or faded
            if (pos < -2 || pos > LED_COUNT + 2 || age > 40) {
                m_bobActive[slot] = false;
            }
        }

        // ── Write to buffer ──────────────────────────────────────────────────
        for (int i = 0; i < LED_COUNT; i++) {
            // Breathing base: deep royal blue
            int baseR = 0;
            int baseG = (int)(15 + breath * 18);
            int baseB = (int)(80 + breath * 90);

            int r = clampI(baseR + (int) glowR[i]);
            int g = clampI(baseG + (int) glowG[i]);
            int b = clampI(baseB + (int) glowB[i]);

            m_buffer.setRGB(i, r, g, b);
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────
    private static double clamp(double v)              { return Math.max(0, Math.min(1, v)); }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private static int clampI(int v)                  { return Math.max(0, Math.min(255, v)); }
}