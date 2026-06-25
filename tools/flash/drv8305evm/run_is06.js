/*
 * run_is06.js - small-Iq no-load sanity + KV cross-check for the 4116 on launchxl, over the
 * debugger: EN_GATE bring-up, safety gate, command a small Iq, let the free shaft spin, read the
 * FAST online flux estimate (convention-independent wind check) + no-load speed, fault-abort, clean
 * stop. is06 is torque control (Iq command via IdqSet_A.value[1]); FAST estimates angle/speed/flux.
 *
 * Usage: dss.sh tools/flash/drv8305evm/run_is06.js <ccxml> <is06_torque_control.out>  [iq_A]
 *
 * KV cross-check: motorVars.flux_VpHz is FAST's ONLINE flux estimate (labs.h: EST_getFlux_Wb*2pi).
 * Treat it as a TREND indicator, NOT proof -- it is not yet established that small-Iq steady state
 * resolves the ~9%-apart KVA(0.012)/KVB(0.011) winds; confirm later by cross-running BOTH builds on
 * the same motor and checking the readings actually separate. No-load speed is a secondary check and
 * is MODULATION-CAPPED (USER_MAX_VS_MAG_PU), not a direct KV.
 *
 * Safety: run from a current-limited bench supply with small Iq. On this board 24 V offset cal is
 * clean, while too-low VM can bias the DRV8305/CMPSS switching behavior; do not assume lower bus is
 * automatically safer. NO prop, motor clamped (free shaft spins to the voltage-limited speed).
 * Aborts (Iq 0, run off, EN_GATE low, flagEnableSys=0, exit 1) on any latched fault or failed
 * bring-up check.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2,e2){ try { e2.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function f(x,n){ return (isNaN(x)?"nan":x.toFixed(n)); }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];
var IQ = (arguments.length > 2) ? Number(arguments[2]) : 1.0;   // A, small
var VBUS_MIN=5.0, POLE_PAIRS=7;
// HARD Iq gate BEFORE touching hardware: the ~40 mOhm 4116 reaches an over-current fast, and the
// monitor only reads faultUse every 500 ms -- too late to catch a fat Iq command. Allow 0 A for
// startup-trip diagnosis, but reject anything above 1.0 A unless this guard is deliberately raised.
// No session is open yet, so a bad value never energizes the gate.
if(isNaN(IQ) || IQ < 0.0 || IQ > 1.0){
    p("FATAL: iq_A=" + arguments[2] + " out of range -- require 0 <= iq_A <= 1.0 A (4116 bench guard).");
    java.lang.System.exit(1);
}
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function gateLow(){ try { s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16); } catch(x){} }
function stopClean(){
    set("IdqSet_A.value[1]","0",e);
    set("motorVars.flagRunIdentAndOnLine","0",e);
    s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0",e);
}
function fail(why){
    p(""); p("!!!!!!!!!!!! is06 sanity ABORTED !!!!!!!!!!!!"); p("  reason: " + why);
    stopClean();
    p("  -> Iq 0, run off, EN_GATE low, flagEnableSys=0, target HALTED.");
    p("===============================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("============ is06 small-Iq sanity + KV cross-check (Iq=" + IQ + " A) ============");
// 1) dead-wait
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (0)  halHandle=" + hh + " (nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init incomplete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at enable-sys wait.");

// pin Iq to 0 before enabling so the default IdqSet_A (1.0) doesn't apply until we choose.
set("IdqSet_A.value[1]","0",e);
set("IdqSet_A.value[0]","0",e);

// 2) EN_GATE high. DSS cannot call target functions reliably in this environment.
p(">>> asserting EN_GATE (GPIO39) ...");
s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var g39=((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0;
p("    EN_GATE GPBDAT bit7 = " + g39 + " (1)");
if(g39!==1) fail("EN_GATE readback low -- gate not enabled.");

// 3) offset cal + safety gate
if(!set("motorVars.flagEnableSys","1",e)) fail("could not set flagEnableSys=1.");
p(">>> flagEnableSys=1; offset cal ...");
var oc=1, waited=0.0;
for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt(); waited+=0.7;
    oc=num("motorVars.flagEnableOffsetCalc",e); if(oc===0) break; }
var fu=num("motorVars.faultUse.all",e), vb=getf("motorVars.VdcBus_V");
p("post-cal ("+waited.toFixed(1)+"s): flagEnableOffsetCalc="+oc+"  faultUse.all="+fu+"  VdcBus_V="+f(vb,3));
if(oc!==0)         fail("offset cal did not complete.");
if(fu!==0)         fail("faultUse.all="+fu+" -- fault latched before spin.");
if(!(vb>VBUS_MIN)) fail("VdcBus_V="+f(vb,3)+" below "+VBUS_MIN+".");
if(vb < 18.0) p("  WARNING: VdcBus_V="+f(vb,2)+" V is below the 24 V bench point; low VM previously showed switching-noise sensitivity.");
if(vb > 28.0) p("  WARNING: VdcBus_V="+f(vb,2)+" V is close to the product over-voltage threshold.");

// 4) start torque: small Iq; nonzero commands may spin to the voltage-limited no-load speed.
set("IdqSet_A.value[1]", IQ, e);
p(">>> flagRunIdentAndOnLine=1; Iq="+IQ+" A. Nonzero Iq may spin the rotor (no prop, clamped).");
if(!set("motorVars.flagRunIdentAndOnLine","1",e)) fail("could not set flagRunIdentAndOnLine=1.");

// 5) monitor ~9 s: FAST online flux + speed + currents; abort on fault.
p("");
p("  t(s) | flux_VpHz | spd_Hz  spd_krpm | Iq_A   Vs_V  VdcBus | flt");
var fluxAcc=0, fluxN=0;
for(var i=0;i<18;i++){
    s.target.runAsynch(); Thread.sleep(500); s.target.halt();
    var fn=num("motorVars.faultUse.all",e);
    var fx=getf("motorVars.flux_VpHz"), sp=getf("motorVars.speed_Hz"), kr=getf("motorVars.speed_krpm");
    var iq=getf("Idq_in_A.value[1]"), vs=getf("motorVars.Vs_V"), vbn=getf("motorVars.VdcBus_V");
    p("  "+(i*0.5).toFixed(1)+" | "+f(fx,5)+" | "+f(sp,1)+"  "+f(kr,3)+" | "+f(iq,2)+"  "+f(vs,2)+"  "+f(vbn,2)+" | "+fn);
    if(fn!==0) fail("faultUse.all="+fn+" latched during spin (t="+(i*0.5).toFixed(1)+"s).");
    if(i>=8 && !isNaN(fx)){ fluxAcc+=fx; fluxN++; }   // average flux once settled
}

// 6) KV cross-check from the settled FAST online flux
var fluxAvg = (fluxN>0)? fluxAcc/fluxN : NaN;
var spF=getf("motorVars.speed_Hz"), krF=getf("motorVars.speed_krpm"), vbF=getf("motorVars.VdcBus_V");
p("");
p(">>> settled FAST online flux_VpHz = " + f(fluxAvg,5) +
  "   (kva/KV450 backfill 0.012 ; kvb/KV470 0.011072)");
p("    => indicative only, closer to " + ((Math.abs(fluxAvg-0.012) <= Math.abs(fluxAvg-0.011072))?"KV450 (kva)":"KV470 (kvb)") +
  " -- confirm by cross-running both builds on this motor.");
p("    no-load: speed_Hz=" + f(spF,1) + " elec  (" + f(krF,3) + " krpm mech, "+POLE_PAIRS+" pp) @ VdcBus="+f(vbF,2)+" V");
p("    (no-load speed is modulation-capped, a secondary check only.)");

// 7) clean stop
p(""); p(">>> stopping: Iq 0, run off, PWM off, EN_GATE low, flagEnableSys=0.");
stopClean();
p("================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
