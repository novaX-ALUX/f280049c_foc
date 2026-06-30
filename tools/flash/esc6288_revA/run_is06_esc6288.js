/*
 * run_is06_esc6288.js - controlled FIRST SPIN of the AM-4116 on esc6288_revA via the is06 torque lab.
 * VALIDATED 2026-06-30: at 12 V / 5 A, no prop, a hand-flick during the continuous window bootstrapped
 * FAST and the rotor held ~4 krpm @ Iq=1.0 A with zero faults during the spin.
 * esc6288 has NO gate-enable pin -> safe-off is the EPWM trip-zone (OST). Arming: flagEnableSys ->
 * offset cal -> flagRunIdentAndOnLine=1 (the lab calls HAL_enablePWM, which un-trips OST). Torque
 * command via IdqSet_A.value[1]. Any latched fault, or a failed bring-up check, aborts to OST safe-off.
 *
 *   iq_A = 0     -> SCOPE MODE: arm at zero current. The half-bridges switch (~0 A) so you can scope the
 *                  dead-band / check for shoot-through BEFORE any torque. The motor must NOT spin.
 *   iq_A > 0     -> small torque. From a dead standstill the sensorless FAST estimator usually will NOT
 *                  start the rotor at this Iq; use the flick window to bootstrap it.
 *   flick_s > 0  -> after arming, FOC runs CONTINUOUSLY for flick_s seconds (no 500 ms halt-cycling, which
 *                  would desync FAST). Hand-flick the shaft once during it; the rotor catches and sustains.
 *
 * Guard: 0 <= iq_A <= 1.0 A (the ~40 mOhm 4116 reaches OC fast). NOTE: stopping a fast-spinning sensorless
 * motor over the debugger latches moduleOverCurrent (faultUse=16) on the gate-cut transient -- BENIGN
 * (protection firing; safe-off achieved; next loadProgram is clean). See cleanStop().
 * Usage: dss.sh run_is06_esc6288.js <ccxml> <is06_torque_control.out> [iq_A] [flick_s]   (default 0 = scope)
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function set(nm,v){ try { e.evaluate(nm+"="+v); return true; } catch(err){ return false; } }
function f(x,n){ return (isNaN(x)?"nan":x.toFixed(n)); }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }

// esc6288 safe-off = EPWM trip-zone one-shot (OST). No EN_GATE pin.
var TZFLG=[0x4093,0x4193,0x4293], TZFRC=[0x409B,0x419B,0x429B], OST=0x4;
function forceOST(){ for(var i=0;i<3;i++){ try{ s.memory.writeData(Memory.Page.DATA,TZFRC[i],OST,16); }catch(x){} } }
function ostBit(i){ return ((rd(TZFLG[i])&OST)!=0)?1:0; }
function ostStr(){ return ostBit(0)+"/"+ostBit(1)+"/"+ostBit(2); }
function ostAllSet(){ return ostBit(0)&&ostBit(1)&&ostBit(2); }

var ccxml=arguments[0], out=arguments[1];
var IQ = (arguments.length > 2) ? Number(arguments[2]) : 0.0;   // A; default 0 = scope mode
var VBUS_MIN=5.0;
if(isNaN(IQ) || IQ < 0.0 || IQ > 1.0){
    p("FATAL: iq_A=" + arguments[2] + " out of range -- require 0 <= iq_A <= 1.0 A (4116 bench guard).");
    java.lang.System.exit(1);
}
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

// IMMEDIATE safe-off for fault aborts: cut now, do not ramp (a real fault must drop gates at once).
function stopClean(){
    set("IdqSet_A.value[1]","0"); set("motorVars.flagRunIdentAndOnLine","0");
    s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    forceOST(); set("motorVars.flagEnableSys","0");
}
// GRACEFUL stop for the normal exit: hold Iq=0 while STILL ARMED so the FOC drives winding current
// to ~0 (motor coasts), THEN disarm + force OST. Avoids the inductive-freewheel OC trip you get from
// hard-cutting a spinning, current-carrying motor.
function cleanStop(){
    set("IdqSet_A.value[1]","0"); set("IdqSet_A.value[0]","0");   // command 0 torque, stay armed
    s.target.runAsynch(); Thread.sleep(1000); s.target.halt();    // FOC regulates winding current to ~0
    // gates are LIVE and the rotor still spins -> do NOT live-run again (frozen duty vs a moving
    // back-EMF rebuilds current). The instant we are halted with current ~0, force OST FIRST, then
    // just write the flags (no further run). This cuts the gates while current is ~0 = no freewheel spike.
    forceOST();
    set("motorVars.flagRunIdentAndOnLine","0"); set("motorVars.flagEnableSys","0");
}
function fail(why){
    p(""); p("!!!!!!!!!!!! is06 first-spin ABORTED !!!!!!!!!!!!"); p("  reason: " + why);
    stopClean();
    p("  -> Iq 0, run off, OST forced ("+ostStr()+"), flagEnableSys=0, target HALTED.");
    p("================================================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("======== esc6288 is06 FIRST SPIN  (Iq=" + IQ + " A" + (IQ===0?", SCOPE MODE - no spin":"") + ") ========");
// 1) dead-wait at the enable-sys gate.
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys"), hh=num("halHandle");
p("dead-wait: flagEnableSys=" + es0 + " (0)  halHandle=" + hh + " (nonzero)  OST=" + ostStr() + " (1/1/1 safe-off)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init incomplete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at the enable-sys wait.");
if(!ostAllSet()) fail("OST not set at startup -- gates NOT safe-off; abort before energizing.");

// 2) pin Iq/Id to 0 so the lab default IdqSet does not apply until we choose.
set("IdqSet_A.value[1]","0"); set("IdqSet_A.value[0]","0");

// 3) flagEnableSys=1 -> offset cal -> safety gate. (No EN_GATE on esc6288.)
if(!set("motorVars.flagEnableSys","1")) fail("could not set flagEnableSys=1.");
p(">>> flagEnableSys=1; offset cal (OST still forced, gates off during cal) ...");
var oc=1, waited=0.0;
for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt(); waited+=0.7;
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
var fu=num("motorVars.faultUse.all"), vb=getf("motorVars.VdcBus_V");
p("post-cal ("+waited.toFixed(1)+"s): offsetCalc="+oc+"  faultUse.all="+fu+"  VdcBus_V="+f(vb,3)+"  OST="+ostStr());
if(oc!==0)         fail("offset cal did not complete.");
if(fu!==0)         fail("faultUse.all="+fu+" -- fault latched before spin.");
if(!(vb>VBUS_MIN)) fail("VdcBus_V="+f(vb,3)+" below "+VBUS_MIN+" -- check the bus.");
if(vb > 50.0)      p("  WARNING: VdcBus_V="+f(vb,2)+" V is near the product OV threshold (~54 V).");
if(!ostAllSet())   fail("OST dropped during cal -- gates released the trip unexpectedly.");

// 4) arm: command Iq, set run flag (the lab un-trips OST via HAL_enablePWM).
set("IdqSet_A.value[1]", IQ);
p(">>> flagRunIdentAndOnLine=1; Iq="+IQ+" A. " + (IQ>0?"Nonzero Iq may spin the rotor (NO prop).":"Iq=0: gates switch at ~0 A, no spin."));
if(!set("motorVars.flagRunIdentAndOnLine","1")) fail("could not set flagRunIdentAndOnLine=1.");
s.target.runAsynch(); Thread.sleep(300); s.target.halt();
var armed=num("motorVars.flagRunIdentAndOnLine"), ost1=ostStr();
p("    armed flag="+armed+"  OST="+ost1+"  (expect OST 0/0/0 = gates LIVE)");
if(ostAllSet()) fail("OST still set after arm -- lab did not enable PWM (no output). Nothing energized.");
p("    >>> GATES LIVE -- SCOPE THE HALF-BRIDGE NOW (dead-band / no shoot-through).");

// 4b) optional continuous "flick window": FOC runs uninterrupted (no 500 ms halt cycling) so a
// hand-flick of the shaft reliably bootstraps the sensorless FAST estimator. Hardware CMPSS/OST
// still protect. Arg 4 = seconds (default 0 = skip).
var FLICK_S = (arguments.length > 3) ? Number(arguments[3]) : 0;
if(FLICK_S > 0 && IQ > 0){
    p("    >>> FLICK THE SHAFT BY HAND NOW (one direction) -- FOC runs continuously for "+FLICK_S+" s.");
    s.target.runAsynch(); Thread.sleep(FLICK_S*1000); s.target.halt();
    var spk=getf("motorVars.speed_Hz"), fnk=num("motorVars.faultUse.all");
    p("    after flick window: speed_Hz="+f(spk,1)+" elec ("+f(getf("motorVars.speed_krpm"),3)+" krpm)  faultUse="+fnk);
    if(fnk!==0) fail("faultUse.all="+fnk+" latched during the flick window.");
}

// 5) monitor ~9 s: currents, speed, flux, bus; abort on ANY latched fault.
// SKIP this 500 ms halt-cycling loop in flick/sustain mode -- halting freezes the FOC and desyncs
// FAST from the freely-coasting rotor, which brakes the spin. The continuous flick window above is
// the sustained-spin demo; hardware CMPSS/OST protect during it.
p("");
if(FLICK_S > 0 && IQ > 0){
    p(">>> sustained-spin mode: skipping the 500 ms sample loop (it would desync FAST). Spin held during");
    p("    the continuous "+FLICK_S+" s window above. Stopping now.");
} else
for(var i=0;i<18;i++){
    s.target.runAsynch(); Thread.sleep(500); s.target.halt();
    var fn=num("motorVars.faultUse.all");
    var iqc=getf("IdqSet_A.value[1]"), iqm=getf("Idq_in_A.value[1]");
    var sp=getf("motorVars.speed_Hz"), kr=getf("motorVars.speed_krpm");
    var vs=getf("motorVars.Vs_V"), vbn=getf("motorVars.VdcBus_V");
    p("  "+(i*0.5).toFixed(1)+" | "+fn+"        | "+f(iqc,2)+"   "+f(iqm,2)+"    | "+f(sp,1)+"  "+f(kr,3)+"  | "+f(vs,2)+"  "+f(vbn,2));
    if(fn!==0) fail("faultUse.all="+fn+" latched during spin (t="+(i*0.5).toFixed(1)+"s).");
}

// 6) stop -> safe-off. NOTE: cutting gates on a fast-spinning sensorless motor via the debugger
// trips moduleOverCurrent (faultUse=16) on the gate-cut transient -- the DSS halt/write latency (ms)
// is long vs the motor's electrical dynamics at ~4 krpm, so a frozen voltage vector vs the moving
// back-EMF rebuilds winding current before OST lands. This is BENIGN: the OC protection firing,
// safe-off (OST 1/1/1) is achieved, and the next loadProgram starts clean. The product's own real-time
// coast/brake (not debugger-halt-based) ramps down cleanly in flight.
p(""); p(">>> stopping: Iq->0, then disarm + force OST (safe-off).");
cleanStop();
var fend=num("motorVars.faultUse.all");
p("    final OST=" + ostStr() + " (expect 1/1/1 safe-off)  faultUse.all=" + fend +
  (fend===16 ? "  (=moduleOverCurrent: BENIGN gate-cut transient on a spinning motor; gates ARE off)" : ""));
p(">>> is06 first-spin window complete. Judge: clean switching on the scope? currents sane? no faults?");
p("================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
