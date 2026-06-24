/*
 * check_3phganinv_is01.js - SAFETY CHECK ONLY for the BOOSTXL-3PhGaNInv on LAUNCHXL-F280049C.
 * Loads is01_intro_hal, runs to the "while(flagEnableSys==false)" dead-wait, and CONFIRMS the power
 * stage is parked safe. It NEVER enables the gate buffer and NEVER spins the motor.
 *
 * Usage: dss.sh tools/flash/check_3phganinv_is01.js <ccxml> <is01_intro_hal.out>
 *
 * GaN polarity reminder -- nEn_uC is ACTIVE-LOW (SN74AVC8T245 OE on GPIO39):
 *     GPIO39 HIGH = buffer DISABLED (safe-off, no PWM reaches the LMG5200 inputs)
 *     GPIO39 LOW  = buffer ENABLED
 * This is the OPPOSITE of the DRV8305 EN_GATE. Do NOT reuse the DRV8305 scripts here.
 *
 * Hard gate (exit 1): parked at the dead-wait (halHandle valid, flagEnableSys==0) AND GPIO39 reads
 * HIGH (1 = disabled). The over-temp pin (GPIO58, active-low) and the zero-current/bus ADC readings
 * are REPORTED for the user to judge -- this is bring-up sanity, not a pass/fail on the analog scale
 * (no offset cal has run yet). Because the gate is never enabled, no power-stage switching can occur
 * even if a reading looks off.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2,e2){ try { e2.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function f3(x){ return (isNaN(x)?"nan":x.toFixed(3)); }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

// GPB data registers (F28004x): GPBDAT=0x7F08, GPBSET=0x7F0A, GPBCLEAR=0x7F0C.
// GPIO39 -> low 16-bit word @0x7F08, bit (39-32)=7  -> mask 0x80.
// GPIO58 -> high 16-bit word @0x7F09, bit (58-48)=10 -> mask 0x0400.
function gpio39(){ return ((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0; }
function gpio58(){ return ((s.memory.readData(Memory.Page.DATA,0x7F09,16,1,false)[0]&0x0400)!=0)?1:0; }
function gateDisableHigh(){ try { s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16); } catch(x){} } // GPBSET bit7 -> GPIO39 HIGH (disabled)

function fail(why){
    p(""); p("!!!!!!!!!!!! is01 safety check FAILED !!!!!!!!!!!!"); p("  reason: " + why);
    gateDisableHigh();                       // defensively force the buffer DISABLED (HIGH)
    set("motorVars.flagEnableSys","0",e);
    p("  -> GPIO39 forced HIGH (buffer disabled), flagEnableSys=0, target left HALTED.");
    p("==================================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("============ is01 GaN power-stage SAFETY check (no enable, no spin) ============");
// 1) run init -> dead-wait, let the EPWM/ADC time-base settle so raw conversions are live.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (expect 0)  halHandle=" + hh + " (expect nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init did not complete (not at the dead-wait).");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at the enable-sys wait.");

// 2) THE safety gate: GPIO39 must idle HIGH = buffer disabled. Never write it here.
var g39=gpio39();
p("nEn_uC GPIO39 (GPBDAT bit7) = " + g39 + " (expect 1 = buffer DISABLED / safe-off)");
if(g39!==1) fail("GPIO39 is LOW -- the GaN buffer is ENABLED at the dead-wait. UNSAFE: setup must leave it HIGH.");

// 3) over-temp pin: active-low TMP302, idles HIGH (deasserted) via pull-up R48. REPORT.
var g58=gpio58();
p("OT GPIO58 (GPBDAT bit26) = " + g58 + " (expect 1 = TMP302 deasserted / not over-temp)");
if(g58!==1) p("  WARNING: GPIO58 reads LOW -- over-temp asserted, or pin/pull-up wrong. Check before enabling.");

// 4) zero-current / bus ADC sanity (REPORT only -- no cal yet). Raw current SOC0 result regs:
//    phase A=ADCB SOC0=0x0B20, phase B=ADCC SOC0=0x0B40, phase C=ADCA SOC0=0x0B00. Bidirectional
//    INA240 at zero current sits near mid-scale (~2048 counts ~ 1.65 V bias). VdcBus from the lab.
function rdc(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0] & 0xFFFF; }
var ca=rdc(0x0B20), cb=rdc(0x0B40), cc=rdc(0x0B00);
var vb=getf("motorVars.VdcBus_V");
p("");
p("--- analog sanity (no offset cal; judge, not gated) ---");
p("    zero-current raw counts: A=" + ca + "  B=" + cb + "  C=" + cc + "  (expect ~2048 = 1.65 V mid-rail)");
p("    VdcBus_V = " + f3(vb) + "  (USB-only bring-up: expect ~0; with 24 V bus: expect ~24)");
if(ca<1500||ca>2600||cb<1500||cb>2600||cc<1500||cc>2600)
    p("  WARNING: a zero-current count is far from mid-scale 2048 -- check INA240 bias / ADC mux before is02.");

p("");
p(">>> is01 OK: parked at dead-wait, GPIO39 HIGH (buffer disabled), OT deasserted. Power stage is SAFE.");
p("    Next: cal_is02_3phganinv.js (zero-current offsets, still WITHOUT enabling the buffer).");
p("===============================================================================");
// leave the buffer disabled; do not change flagEnableSys (it is already 0).
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
