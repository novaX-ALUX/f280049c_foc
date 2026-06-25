/*
 * s1_rails_clock.js - esc6288_revA bring-up STAGE 1: rails + clock. OBSERVE-ONLY.
 *
 * Confirms the MCU came up on the right clock before anything else. The esc6288 uses a 10 MHz
 * resonator -> SYSCTL_IMULT(20) -> 100 MHz; if that assumption is wrong (e.g. 50 MHz) every
 * downstream timing (PWM freq, dead-band, ADC, 1 ms tick) is off. We check it WITHOUT a scope by
 * comparing the firmware's 1 ms tick (g_now_ms) against host wall-clock over a known interval:
 * tick_delta / wall_ms ~= 1.0 means the timebase is correct; ~0.5 means SYSCLK is half, ~2.0 double.
 *
 * SAFETY: pure observe. Never enables the system, never un-trips the EPWM, never drives PWM. The
 * power stage stays held off by the trip-zone (OST) the whole time. Run with NO bus voltage first.
 *
 * Usage: dss.sh tools/flash/esc6288_revA/s1_rails_clock.js <ccxml> <product.out|is01_intro_hal.out>
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function f(x,n){ return (isNaN(x)?"nan":x.toFixed(n)); }

var ccxml=arguments[0], out=arguments[1];
var WALL_MS = 1000;   // host-timed window for the tick-rate measurement

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function bail(why){
    p(""); p("!!!!!! STAGE 1 (rails/clock) FAILED: " + why);
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("======== esc6288 STAGE 1: rails + clock (observe-only) ========");

// 1) run init to the dead-wait; HAL must have come up.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var hh=num("halHandle"), es=num("motorVars.flagEnableSys");
p("HAL: halHandle=" + hh + " (expect nonzero)   flagEnableSys=" + es + " (expect 0 = parked)");
if(!(hh>0)) bail("halHandle invalid -- HAL_init did not complete (rails/clock or boot problem).");

// 2) clock check: tick rate vs host wall-clock. g_now_ms increments in the 1 ms ISR, which is
//    derived from SYSCLK; the ratio exposes a wrong SYSCLK without needing a scope.
var t0=num("g_now_ms");
if(isNaN(t0)) bail("could not read g_now_ms (symbol stripped?) -- cannot verify the clock this way.");
s.target.runAsynch(); Thread.sleep(WALL_MS); s.target.halt();
var t1=num("g_now_ms");
var ticks=t1-t0, ratio=ticks/WALL_MS;
p("");
p("clock: g_now_ms advanced " + ticks + " in ~" + WALL_MS + " ms wall  -> tick/wall ratio = " + f(ratio,3));
p("  expect ~1.000 (SYSCLK 100 MHz). ~0.5 => SYSCLK is HALF (resonator/IMULT wrong); ~2.0 => DOUBLE.");
if(ratio < 0.80 || ratio > 1.25){
    bail("tick/wall ratio " + f(ratio,3) + " is off -- SYSCLK is likely wrong (check the 10 MHz " +
         "resonator + SYSCTL_IMULT(20)). Do NOT proceed to PWM/ADC stages until this reads ~1.0.");
}

p("");
p(">>> STAGE 1 OK: HAL up, SYSCLK timebase ~correct (ratio " + f(ratio,3) + "). Power stage untouched.");
p("    Next: s2_idle_ost.js (confirm PWM idle low + trip-zone safe-off).");
p("================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
