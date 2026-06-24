/*
 * cal_is02_3phganinv.js - zero-current offset cal + analog-front-end health for the
 * BOOSTXL-3PhGaNInv on LAUNCHXL-F280049C, over the debugger. Reports current/voltage ADC offsets,
 * the zero-current residual + noise band, and the bus sense.
 *
 * Usage: dss.sh tools/flash/cal_is02_3phganinv.js <ccxml> <is02_offset_gain_cal.out>
 *
 * KEY DIFFERENCE vs the DRV8305 cal_is02.js: this script DOES NOT enable the gate buffer. The GaN
 * board's current sense is an EXTERNAL INA240 (always powered, independent of the SN74AVC8T245 PWM
 * buffer), so a valid zero-current offset is acquired with GPIO39 held HIGH = buffer DISABLED. The
 * EPWM/ADC time-base still runs and triggers conversions; keeping the buffer off means the LMG5200
 * power stage can NEVER switch during cal -- strictly safer. (The DRV8305 CSA lives inside the gate
 * driver and needs EN_GATE high, which is why that script enables it; the GaN one must not.)
 *
 * nEn_uC is ACTIVE-LOW: GPIO39 HIGH = disabled, LOW = enabled. This script only ever drives/holds
 * GPIO39 HIGH.
 *
 * is02 re-arms offset cal forever (re-sets flagEnableOffsetCalc after each pass, guarded by the
 * lab-only flagEnableOffsetCalibration, default true). So we set flagEnableOffsetCalibration=false
 * first -> the cal runs ONCE and the latched offsets freeze.
 *
 * Hard-fails (forces GPIO39 HIGH, flagEnableSys=0, HALT, exit 1) unless: parked at the dead-wait,
 * GPIO39 idles HIGH, the one-shot cal completed, faultUse.all==0. VdcBus is NOT gated (USB-only
 * bring-up runs with the 24 V bus OFF -> ~0 V is expected); offsets/noise are REPORTED to judge.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s,e){ try { return Number(e.evaluate(s)); } catch(err){ return NaN; } }
function set(s,v,e){ try { e.evaluate(s+"="+v); return true; } catch(err){ return false; } }
function f3(x){ return (isNaN(x)?"nan":x.toFixed(4)); }
function rdf(a){ if(isNaN(a)) return NaN; var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
                 return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); }
function getf(nm){ try { return rdf(Number(e.evaluate("&"+nm))); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];
// GaN current full scale = 5 mOhm shunt x INA240 gain 20 = 0.1 V/A -> 3.3 V / 0.1 = 33.0 A over 4096
// counts (user.h: USER_ADC_FULL_SCALE_CURRENT_A = 33.0). NOT the DRV8305 47.14.
var FS_CURRENT_A = 33.0;
var VBUS_MIN = 5.0;   // above this, the is02 voltage offsets (V/dcBus) are meaningful; at ~0 V they are not.

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function gpio39(){ return ((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0; }
function gateDisableHigh(){ try { s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16); } catch(x){} } // GPBSET bit7 -> HIGH (disabled)

function fail(why){
    p(""); p("!!!!!!!!!!!! is02 offset cal FAILED !!!!!!!!!!!!"); p("  reason: " + why);
    gateDisableHigh();                       // GaN safe-off = GPIO39 HIGH
    set("motorVars.flagEnableSys","0",e);
    var g=gpio39();
    p("  -> GPIO39 forced HIGH (readback " + g + ", expect 1 = disabled), flagEnableSys=0, target HALTED.");
    p("================================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("============ is02 GaN zero-current offset cal (buffer stays DISABLED) ============");
// 1) run init -> dead-wait.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (expect 0)  halHandle=" + hh + " (expect nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init did not complete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at the enable-sys wait.");

// 2) confirm the buffer is OFF and KEEP it off. INA240 samples without it.
var g39=gpio39();
p("nEn_uC GPIO39 = " + g39 + " (expect 1 = buffer DISABLED; this script never enables it)");
if(g39!==1) fail("GPIO39 is LOW -- buffer enabled at the dead-wait; setup must leave it HIGH.");

// 3) one-shot cal so the latched offsets freeze (is02 otherwise re-arms forever).
if(!set("flagEnableOffsetCalibration","0",e)) fail("could not set flagEnableOffsetCalibration=0.");
p("one-shot: flagEnableOffsetCalibration=" + num("flagEnableOffsetCalibration",e) + " (expect 0)");

// 4) flagEnableSys=1 -> one-shot offset cal (~50000 ISR). Poll until done; sample raw current counts
//    DURING cal (ADC live) to capture the real zero-current noise band. SOC0 result regs:
//    phase A=ADCB SOC0=0x0B20, phase B=ADCC SOC0=0x0B40, phase C=ADCA SOC0=0x0B00.
if(!set("motorVars.flagEnableSys","1",e)) fail("could not set flagEnableSys=1.");
p(">>> flagEnableSys=1; running one-shot offset cal (buffer still DISABLED) ...");
var IRAW=[0x0B20,0x0B40,0x0B00], clo=[65535,65535,65535], chi=[0,0,0];
function rdc(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0] & 0xFFFF; }
var oc=1, waited=0.0;
for(var k=0;k<8;k++){
    s.target.runAsynch(); Thread.sleep(700); s.target.halt(); waited+=0.7;
    for(var ph=0;ph<3;ph++){ var c=rdc(IRAW[ph]); if(c<clo[ph])clo[ph]=c; if(c>chi[ph])chi[ph]=c; }
    oc=num("motorVars.flagEnableOffsetCalc",e); if(oc===0) break;
}
p("post-cal (" + waited.toFixed(1) + "s): flagEnableOffsetCalc=" + oc + " (expect 0)");
if(oc!==0) fail("offset cal did not complete within " + waited.toFixed(1) + "s.");

// 5) global health gate (no VdcBus floor: 24 V bus is OFF for USB-only bring-up).
var fu=num("motorVars.faultUse.all",e), vb=getf("motorVars.VdcBus_V");
p("health: faultUse.all=" + fu + " (expect 0)  VdcBus_V=" + f3(vb) + " (USB-only: ~0 expected)");
if(fu!==0) fail("faultUse.all=" + fu + " -- a fault latched during cal (over-current/trip).");

// 6) REPORT the calibration outputs (judged by the user, not hard-gated).
var oi=[getf("motorVars.offsets_I_A.value[0]"),getf("motorVars.offsets_I_A.value[1]"),getf("motorVars.offsets_I_A.value[2]")];
var ov=[getf("motorVars.offsets_V_V.value[0]"),getf("motorVars.offsets_V_V.value[1]"),getf("motorVars.offsets_V_V.value[2]")];
p("");
p("--- current offsets motorVars.offsets_I_A (A; GaN zero-current midpoint ~ +16.5 = 0.5*33.0) ---");
p("    Ia=" + f3(oi[0]) + "  Ib=" + f3(oi[1]) + "  Ic=" + f3(oi[2]));
var imn=(oi[0]+oi[1]+oi[2])/3.0, ispread=Math.max(oi[0],oi[1],oi[2])-Math.min(oi[0],oi[1],oi[2]);
p("    mean=" + f3(imn) + "  spread(max-min)=" + f3(ispread) + "  (3 INA240 channels should match closely)");
// is02 computes the voltage offset as FILTER(adcData.V_V * (1.0/dcBus_V)). At ~0 V bus (USB-only
// bring-up) 1/dcBus is Inf -> the voltage offsets are MEANINGLESS. Only report them as valid if a
// real bus is present; the CURRENT offsets + noise above are the valid 0 V outputs.
p("--- voltage offsets motorVars.offsets_V_V (fraction of VdcBus) ---");
if(vb > VBUS_MIN){
    p("    Va=" + f3(ov[0]) + "  Vb=" + f3(ov[1]) + "  Vc=" + f3(ov[2]) + "  (bus present: valid)");
} else {
    p("    INVALID at VdcBus_V=" + f3(vb) + " (~0 V): is02 voltage offset = V/dcBus -> Inf/NaN. Ignore.");
    p("    (raw: Va=" + f3(ov[0]) + "  Vb=" + f3(ov[1]) + "  Vc=" + f3(ov[2]) + "). Re-run with a known bus to trim voltage offsets.");
}

// 7) zero-current noise: raw counts captured during cal, converted with the GaN full scale.
var LSB_A = FS_CURRENT_A / 4096.0;
p("--- zero-current raw ADC counts during cal (mid-scale ~2048; band = noise) ---");
var pn=["A","B","C"];
for(var ph=0;ph<3;ph++){
    var band=chi[ph]-clo[ph];
    p("    phase " + pn[ph] + ": [" + clo[ph] + " .. " + chi[ph] + "]  band=" + band + " LSB (~" + f3(band*LSB_A) + " A @ 33.0 FS)");
}

// 8) clean exit: buffer was never enabled; ensure it is HIGH and flagEnableSys cleared.
gateDisableHigh(); set("motorVars.flagEnableSys","0",e);
var gEnd=gpio39();
p("");
if(gEnd!==1){
    p(">>> WARNING: GPIO39 readback=" + gEnd + " (expect 1) -- buffer NOT disabled at exit (it should never");
    p("    have been enabled here). Cut bus power / reset the board. Exiting nonzero.");
    p("=================================================================================");
    s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
    java.lang.System.exit(1);
}
p(">>> is02 front-end check done. Buffer stayed DISABLED throughout. GPIO39 readback=" + gEnd + " (1).");
p("    Judge: offsets ~+16.5 A and 3-phase symmetric? counts near 2048 with a small noise band?");
p("    Current GAIN (33.0 FS) needs a KNOWN injected current to trim -- not done here.");
p("    Next: with the 24 V bus at 0 V, scope_deadtime_3phganinv.js to verify GaN dead-time.");
p("=================================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
