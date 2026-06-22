/*
 * cal_is02.js - drive is02_offset_gain_cal on launchxl over the debugger and read back the
 * analog front-end health: current/voltage ADC offsets, zero-current residual + noise, bus sense.
 *
 * Usage: dss.sh tools/flash/cal_is02.js <ccxml> <is02_offset_gain_cal.out>
 *
 * Why a DEDICATED script (not the generic prepare_drv8305_gate.js): is02 is the ONLY sensorless
 * lab that re-arms offset calibration forever -- after each 50000-ISR cal pass it latches the
 * offsets then immediately re-sets motorVars.flagEnableOffsetCalc=true (guarded by the lab-only
 * flagEnableOffsetCalibration, default true; is02:762-764). So flagEnableOffsetCalc is ~always 1,
 * and the generic gate-prep's "flagEnableOffsetCalc==0 => cal done" readiness gate would
 * false-FAIL on is02. is03/is04/is05/is06 self-clear once, so they still use prepare_drv8305_gate.js.
 *
 * What this does: same EN_GATE bring-up + safety gate as prepare_drv8305_gate.js, but first sets
 * flagEnableOffsetCalibration=false so the cal runs ONCE and the latched offsets freeze (PWM is
 * disabled at completion). Then it reads and reports the is02 outputs and judges nothing final --
 * is02 is an analog-front-end HEALTH CHECK at zero current (no motor / current-limited / no prop):
 * confirm offsets converge + are 3-phase symmetric, the zero-current residual is small + low-noise,
 * and VdcBus_V matches the metered bus. The current GAIN (USER_ADC_FULL_SCALE_CURRENT_A=47.14) is
 * NOT trimmed here -- that needs a known injected current and moves to is03.
 *
 * Hard-fails (pulls EN_GATE low, flagEnableSys=0, leaves target HALTED, exit 1) unless: parked at
 * the dead-wait, EN_GATE readback high, the one-shot cal completed (flagEnableOffsetCalc==0),
 * faultUse.all==0 (CMPSS fix holds on the is02 path too), VdcBus_V > 5. Offset symmetry / noise are
 * REPORTED for the user to judge (board-specific thresholds), not hard-gated.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s,e){ try { return Number(e.evaluate(s)); } catch(err){ return NaN; } }
function set(s,v,e){ try { e.evaluate(s+"="+v); return true; } catch(err){ return false; } }
function f3(x){ return (isNaN(x)?"nan":x.toFixed(4)); }
// CRITICAL: DSS expression.evaluate() TRUNCATES float32 to integer (3.14159 -> 3.0). For any
// float, read its address via &expr (ints are fine through evaluate) then reconstruct the 2-word
// C28x IEEE754 value exactly. getf() must be called after s/e are assigned below.
function rdf(a){ if(isNaN(a)) return NaN; var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
                 return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); }
function getf(nm){ try { return rdf(Number(e.evaluate("&"+nm))); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];
var VBUS_MIN = 5.0;

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function fail(why){
    p("");
    p("!!!!!!!!!!!! is02 offset cal FAILED !!!!!!!!!!!!");
    p("  reason: " + why);
    s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16);  // GPBCLEAR bit7 -> EN_GATE low
    set("motorVars.flagEnableSys", "0", e);
    p("  -> EN_GATE pulled low, flagEnableSys=0, target left HALTED. Fix before continuing.");
    p("================================================");
    try { s.target.disconnect(); } catch(x){}
    server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p("");
p("============ is02 offset/gain-cal front-end check ============");
// 1) run init -> "while(flagEnableSys==false)" dead-wait.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0 = num("motorVars.flagEnableSys",e), hh = num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (expect 0)  halHandle=" + hh + " (expect nonzero)");
if(!(hh > 0)) fail("halHandle invalid -- HAL_init did not complete (not at the dead-wait).");
if(es0 !== 0) fail("flagEnableSys != 0 -- not parked at the lab's enable-sys wait.");

// 2) make cal ONE-SHOT so flagEnableOffsetCalc settles to 0 and offsets freeze (is02-specific).
if(!set("flagEnableOffsetCalibration", "0", e))
    fail("could not set flagEnableOffsetCalibration=0 (one-shot cal).");
p("one-shot: flagEnableOffsetCalibration=" + num("flagEnableOffsetCalibration",e) + " (expect 0)");

// 3) assert EN_GATE (lab skips HAL_enableDRV on DRV8305_SPI; GPBSET bit7 = GPIO39).
p(">>> asserting EN_GATE (GPIO39 high) to wake the DRV8305 ...");
s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var gpbdat = s.memory.readData(Memory.Page.DATA, 0x7F08, 16, 1, false);
var g39 = ((gpbdat[0] & 0x80) != 0) ? 1 : 0;
p("    EN_GATE: GPBDAT bit7 (GPIO39) = " + g39 + " (expect 1)");
if(g39 !== 1) fail("EN_GATE readback low -- GPIO39 did not assert; gate not enabled.");

// 4) flagEnableSys=1 -> one-shot offset cal (~50000 ISR ~ 2.5s @ 20kHz). Poll until done.
if(!set("motorVars.flagEnableSys", "1", e)) fail("could not set flagEnableSys=1.");
p(">>> flagEnableSys=1; running one-shot offset cal ...");
// Sample the RAW current ADC result registers (counts) WHILE cal runs (PWM on -> ADC triggered).
// This is the real zero-current noise: after cal completes is02 disables PWM and the ADC freezes,
// so a post-cal adcData sample would falsely read zero noise. Current SOC0 result regs:
//   phase A = ADCB SOC0 = 0x0B20, phase B = ADCC SOC0 = 0x0B40, phase C = ADCA SOC0 = 0x0B00.
var IRAW = [0x0B20, 0x0B40, 0x0B00];
var clo=[65535,65535,65535], chi=[0,0,0];
function rdc(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0] & 0xFFFF; }
var oc = 1, waited = 0.0;
for(var k=0;k<8;k++){
    s.target.runAsynch(); Thread.sleep(700); s.target.halt(); waited += 0.7;
    for(var ph=0;ph<3;ph++){ var c=rdc(IRAW[ph]); if(c<clo[ph])clo[ph]=c; if(c>chi[ph])chi[ph]=c; }
    oc = num("motorVars.flagEnableOffsetCalc",e);
    if(oc === 0) break;
}
p("post-cal (" + waited.toFixed(1) + "s): flagEnableOffsetCalc=" + oc + " (expect 0)");
if(oc !== 0) fail("offset cal did not complete within " + waited.toFixed(1) + "s (flagEnableOffsetCalc still 1).");

// 5) global health gate.
var fu = num("motorVars.faultUse.all",e);   // integer field: evaluate() is fine
var vb = getf("motorVars.VdcBus_V");        // float: must use exact raw read
p("health: faultUse.all=" + fu + " (expect 0)  VdcBus_V=" + f3(vb) + " (expect > " + VBUS_MIN + ")");
if(fu !== 0)        fail("faultUse.all=" + fu + " -- a fault is latched (over-current/bus). Stage not healthy.");
if(!(vb > VBUS_MIN)) fail("VdcBus_V=" + f3(vb) + " below " + VBUS_MIN + " V -- bus absent or voltage sense wrong.");

// 6) REPORT the calibration outputs (judged by the user, not hard-gated).
var oi = [getf("motorVars.offsets_I_A.value[0]"), getf("motorVars.offsets_I_A.value[1]"), getf("motorVars.offsets_I_A.value[2]")];
var ov = [getf("motorVars.offsets_V_V.value[0]"), getf("motorVars.offsets_V_V.value[1]"), getf("motorVars.offsets_V_V.value[2]")];
p("");
p("--- current offsets motorVars.offsets_I_A (A, the zero-current ADC midpoint) ---");
p("    Ia=" + f3(oi[0]) + "  Ib=" + f3(oi[1]) + "  Ic=" + f3(oi[2]));
var imn = (oi[0]+oi[1]+oi[2])/3.0, ispread = Math.max(oi[0],oi[1],oi[2]) - Math.min(oi[0],oi[1],oi[2]);
p("    mean=" + f3(imn) + "  spread(max-min)=" + f3(ispread) + "  (3 CSAs should match closely)");
p("--- voltage offsets motorVars.offsets_V_V (fraction of VdcBus) ---");
p("    Va=" + f3(ov[0]) + "  Vb=" + f3(ov[1]) + "  Vc=" + f3(ov[2]));

// 7) zero-current noise: raw current ADC counts captured DURING cal (PWM on, ADC live). Bidirectional
//    CSA at zero current sits near mid-scale 2048; the count band is the front-end noise. Convert to
//    amps with the configured full scale (USER_ADC_FULL_SCALE_CURRENT_A = 47.14 A over 4096 counts).
var LSB_A = 47.14 / 4096.0;
p("--- zero-current raw ADC counts during cal (PWM on; mid-scale ~2048; band = noise) ---");
var pn = ["A","B","C"];
for(var ph=0;ph<3;ph++){
    var band = chi[ph]-clo[ph];
    p("    phase " + pn[ph] + ": [" + clo[ph] + " .. " + chi[ph] + "]  band=" + band +
      " LSB (~" + f3(band*LSB_A) + " A)");
}

p("");
p(">>> is02 front-end OK: gate enabled, one-shot cal done, no fault, VdcBus_V=" + f3(vb) + ".");
p("    Judge: offsets 3-phase symmetric? counts near mid-scale + small noise band? VdcBus_V == metered bus?");
p("    Do NOT trim USER_ADC_FULL_SCALE_CURRENT_A here (no injected current) -- defer gain to is03.");
p("    Voltage gain can be sanity-checked now: meter VM/PVDD vs VdcBus_V above.");
p("=============================================================");
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
