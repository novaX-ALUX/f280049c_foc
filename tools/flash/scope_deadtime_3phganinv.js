/*
 * scope_deadtime_3phganinv.js - GaN DEAD-TIME verification at ZERO BUS VOLTAGE for the
 * BOOSTXL-3PhGaNInv on LAUNCHXL-F280049C. Produces a clean, dcBus-INDEPENDENT 50% switching pattern
 * on all three half-bridges, enables the PWM buffer so the complementary EPWM edges reach the
 * LMG5200 inputs, and holds it so you can scope the high/low-side non-overlap. Then disables again.
 *
 * Usage: dss.sh tools/flash/scope_deadtime_3phganinv.js <ccxml> <is02_offset_gain_cal.out> [hold_s]
 *        hold_s defaults to 20 s. NOTE: this loads is02_offset_gain_cal.out, NOT an is03 build.
 *
 * Why is02 and NOT is03: is03's V/f path scales the voltage by oneOverDcBus = 1.0/dcBus_V
 * (is03_hardware_test.c). At 0 V bus that is Inf, and SVGEN then writes Inf/NaN/saturated PWM
 * compares -- NOT a reliable dead-band waveform. is02's offset-cal path instead HARD-SETS
 * pwmData.Vabc_pu = 0 and calls HAL_enablePWM (is02_offset_gain_cal.c runOffsetsCalculation), so
 * HAL_writePWMData produces exactly 50% duty ((sat(-0,+-0.5)+0.5)*period) with NO dcBus term. is02
 * re-arms offset cal forever (default flagEnableOffsetCalibration=true), so this 50% zero-vector runs
 * continuously -- a clean dead-band source independent of the (absent) bus.
 *
 * !!! THE 24 V (OR ANY) BUS MUST BE AT 0 V !!!
 * The LMG5200 has NO internal shoot-through protection -- the MCU dead-band is the ONLY protection,
 * and that is exactly what is unverified here. HARD-ABORTS if VdcBus_V reads above 2 V at start OR
 * rises above it during the hold, disabling the buffer IMMEDIATELY (GPBSET first, then flags).
 *
 * GaN polarity: nEn_uC is ACTIVE-LOW. enable = GPBCLEAR 0x7F0C bit7 (GPIO39 LOW, readback 0);
 *               disable = GPBSET 0x7F0A bit7 (GPIO39 HIGH, readback 1). Opposite of DRV8305.
 *
 * Scope hint: probe one phase's two LMG5200 PWM inputs (HI and LO of the same half-bridge). Confirm
 * they are NEVER high together; measure the gap = the MCU dead-band (HAL_PWM_DBRED_CNT/DBFED_CNT = 20
 * counts ~ 200 ns). Repeat per phase. Only after non-overlap is confirmed should any bus be applied.
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
var HOLD_S = (arguments.length > 2) ? Number(arguments[2]) : 20.0;
var BUS_MAX_SCOPE = 2.0;   // V -- bus MUST be ~0 for dead-time scoping; abort above this.

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function gpio39(){ return ((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0; }
function gateEnableLow(){ s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16); }   // GPBCLEAR bit7 -> LOW (enabled)
function gateDisableHigh(){ try { s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16); } catch(x){} } // GPBSET bit7 -> HIGH (disabled)

// Normal (gentle) stop: clear run/sys, let the bg loop force PWM safe, then disable the buffer.
function stopClean(){
    set("motorVars.flagEnableSys","0",e);
    s.target.runAsynch(); Thread.sleep(200); s.target.halt();
    gateDisableHigh();
}
// Fault stop: disable the buffer FIRST (no further switching), THEN soft-clear flags.
function fail(why){
    gateDisableHigh();                                  // IMMEDIATE safe-off
    var g=gpio39();                                     // verify the write took (this path never runs the target)
    p(""); p("!!!!!!!!!!!! dead-time scope ABORTED !!!!!!!!!!!!"); p("  reason: " + why);
    set("motorVars.flagEnableSys","0",e);               // memory write only; does NOT run the target
    if(g!==1) p("  !! GPIO39 readback=" + g + " (expect 1): disable did NOT take, buffer may be LIVE -- cut bus power / reset NOW.");
    p("  -> buffer disabled (GPIO39 readback " + g + ", expect 1), flagEnableSys=0, HALTED.");
    p("==================================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("============ GaN dead-time scope @ 0 V bus, via is02 50% zero-vector (hold " + HOLD_S + " s) ============");
p("    *** CONFIRM THE BUS SUPPLY IS OFF / AT 0 V BEFORE CONTINUING ***");
// 1) dead-wait.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (0)  halHandle=" + hh + " (nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init incomplete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at the enable-sys wait.");
if(gpio39()!==1) fail("GPIO39 is LOW at the dead-wait -- buffer already enabled; setup must leave it HIGH.");

// 2) THE bus gate: must be ~0 V. This is what makes enabling the buffer safe.
var vb0=getf("motorVars.VdcBus_V");
p("VdcBus_V = " + f3(vb0) + "  (MUST be < " + BUS_MAX_SCOPE + " V for dead-time scoping)");
if(!(vb0 < BUS_MAX_SCOPE)) fail("VdcBus_V=" + f3(vb0) + " V >= " + BUS_MAX_SCOPE + " -- bus is NOT at 0 V. Turn the supply OFF.");

// 3) start continuous offset cal => HAL_enablePWM + 50% zero-vector every ISR (re-armed forever).
//    Leave flagEnableOffsetCalibration at its default (true) so it never one-shots and freezes.
if(!set("motorVars.flagEnableSys","1",e)) fail("could not set flagEnableSys=1.");
p(">>> flagEnableSys=1; is02 offset cal running -> continuous 50% zero-vector PWM (dcBus-independent).");
s.target.runAsynch(); Thread.sleep(700); s.target.halt();   // let HAL_enablePWM + 50% take effect
var fu=num("motorVars.faultUse.all",e), vb1=getf("motorVars.VdcBus_V");
p("after PWM start: faultUse.all=" + fu + "  VdcBus_V=" + f3(vb1));
if(fu!==0) fail("faultUse.all=" + fu + " -- fault latched before enabling the buffer.");
if(!(vb1 < BUS_MAX_SCOPE)) fail("VdcBus_V rose to " + f3(vb1) + " V -- bus came on. Turn the supply OFF.");

// 4) ENABLE the buffer so the 50% EPWM edges reach the LMG5200 inputs.
p(">>> enabling buffer: nEn_uC LOW (GPBCLEAR GPIO39) ...");
gateEnableLow();
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var g39=gpio39();
p("    GPIO39 readback = " + g39 + " (expect 0 = buffer ENABLED)");
if(g39!==0) fail("GPIO39 did not go LOW -- buffer not enabled.");
p(">>> 50% PWM switching now (all phases, 0 V bus). SCOPE the high/low-side non-overlap per phase.");

// 5) hold; keep watching that the bus stays ~0 and no fault latches.
var samples = Math.max(1, Math.ceil(HOLD_S / 0.5));
for(var i=0;i<samples;i++){
    s.target.runAsynch(); Thread.sleep(500); s.target.halt();
    var fn=num("motorVars.faultUse.all",e), vbn=getf("motorVars.VdcBus_V");
    if((i % 4)===0) p("  t=" + (i*0.5).toFixed(1) + "s  VdcBus_V=" + f3(vbn) + "  faultUse=" + fn + "  (scope the dead-band now)");
    if(fn!==0) fail("faultUse.all=" + fn + " latched during the hold (t=" + (i*0.5).toFixed(1) + "s).");
    if(!(vbn < BUS_MAX_SCOPE)) fail("VdcBus_V rose to " + f3(vbn) + " V during the hold -- bus was switched on. Killing PWM.");
}

// 6) disable and park safe (gentle path -- no fault).
p(""); p(">>> hold done. Disabling: buffer DISABLED (nEn_uC HIGH), flagEnableSys=0.");
stopClean();
var gEnd=gpio39();
p("    final GPIO39 readback = " + gEnd + " (expect 1 = buffer disabled).");
if(gEnd!==1){
    p("  !! GPIO39 is NOT disabled at exit -- cut bus power / reset the board. Exiting nonzero.");
    p("===========================================================================");
    s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
    java.lang.System.exit(1);
}
p(">>> Judge from the scope: high and low side NEVER overlap? gap ~ 200 ns (20 counts)?");
p("    If overlap exists, RAISE HAL_PWM_DBRED_CNT/DBFED_CNT in hal.h and rebuild BEFORE any bus voltage.");
p("    Only after non-overlap is confirmed should you apply a current-limited 24 V bus (then is06).");
p("===========================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
