/*
 * run_is06_3phganinv.js - small-Iq no-load sanity + KV cross-check for the AM-4116 on the
 * BOOSTXL-3PhGaNInv (LAUNCHXL-F280049C), over the debugger. Brings up the GaN PWM buffer, gates on
 * a clean offset cal, commands a small Iq, lets the free shaft spin, reads the FAST online flux +
 * no-load speed, fault-aborts, and stops clean. is06 is torque control (Iq via IdqSet_A.value[1]).
 *
 * Usage: dss.sh tools/flash/run_is06_3phganinv.js <ccxml> <is06_torque_control.out> [iq_A]
 *        iq_A defaults to 0.1 A. HARD max 0.5 A (GaN bring-up guard -- see below).
 *
 * GaN polarity (OPPOSITE of the DRV8305 run_is06.js -- do NOT reuse that one):
 *     nEn_uC is ACTIVE-LOW. enable  = GPBCLEAR 0x7F0C bit7 -> GPIO39 LOW,  readback 0.
 *                           disable = GPBSET   0x7F0A bit7 -> GPIO39 HIGH, readback 1.
 *
 * Why is06 (not is03/is05) on this board: the AM-4116 (~40 mOhm) cannot run is03 scalar V/f
 * (VOLT_MIN across Rs is an instant over-current) and is05 FAST ID exceeds this board's current
 * sense ceiling (5 mOhm x INA240 gain 20 = +-16.5 A, even tighter than the DRV8305 EVM). Small-Iq
 * is06 with FAST already supplying angle/speed/flux is the correct low-load sanity here; full-power
 * 4116 running belongs to esc6288. Params come from the legacy esc_drv8300 FAST ID (see motors/README).
 *
 * Safety: current-limited bench supply, small Iq, no prop, motor clamped (free shaft spins to the
 * voltage-limited speed). The dead-band MUST already be scope-verified at 0 V bus
 * (scope_deadtime_3phganinv.js) -- LMG5200 has no internal shoot-through protection. Aborts
 * (Iq 0, run off, buffer disabled, flagEnableSys=0, exit 1) on any latched fault or failed check.
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
var IQ = (arguments.length > 2) ? Number(arguments[2]) : 0.1;   // A, small (default 0.1)
var VBUS_MIN=5.0, POLE_PAIRS=7;
// HARD Iq gate BEFORE touching hardware. The ~40 mOhm 4116 reaches over-current fast and the monitor
// only reads faultUse every 500 ms. First-bring-up max is 0.5 A (lower than the DRV8305 script's
// 1.0 A) -- raise deliberately only once direction/phase order are confirmed. No session is open yet.
if(isNaN(IQ) || IQ < 0.0 || IQ > 0.5){
    p("FATAL: iq_A=" + arguments[2] + " out of range -- require 0 <= iq_A <= 0.5 A (GaN 4116 bring-up guard).");
    java.lang.System.exit(1);
}
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function gpio39(){ return ((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0; }
function gateEnableLow(){ s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16); }     // GPBCLEAR bit7 -> LOW (enabled)
function gateDisableHigh(){ try { s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16); } catch(x){} } // GPBSET bit7 -> HIGH (disabled)
function stopClean(){
    set("IdqSet_A.value[1]","0",e);
    set("motorVars.flagRunIdentAndOnLine","0",e);
    s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateDisableHigh(); set("motorVars.flagEnableSys","0",e);
}
function fail(why){
    gateDisableHigh();                                  // IMMEDIATE safe-off
    var g=gpio39();                                     // VERIFY the write took BEFORE running the target again
    p(""); p("!!!!!!!!!!!! is06 sanity ABORTED !!!!!!!!!!!!"); p("  reason: " + why);
    if(g!==1){
        // disable readback FAILED (gateDisableHigh swallows write errors) -- the buffer may still be
        // LIVE. Do NOT runAsynch(); leave the target halted with flags untouched.
        p("  !! GPIO39 readback=" + g + " (expect 1): disable did NOT take, buffer may be LIVE.");
        p("  !! NOT running the target; flags left as-is, target HALTED. Cut bus power / reset the board NOW.");
    } else {
        stopClean();                                    // confirmed off -> safe to soft-clean flags
        p("  -> buffer DISABLED (GPIO39=" + gpio39() + "), then Iq 0/run off/flagEnableSys=0, HALTED.");
    }
    p("===============================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("============ GaN is06 small-Iq sanity + KV cross-check (Iq=" + IQ + " A) ============");
// 1) dead-wait
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (0)  halHandle=" + hh + " (nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init incomplete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at enable-sys wait.");
if(gpio39()!==1) fail("GPIO39 is LOW at the dead-wait -- buffer already enabled; setup must leave it HIGH.");

// pin Iq to 0 before enabling so the default IdqSet_A doesn't apply until we choose.
set("IdqSet_A.value[1]","0",e);
set("IdqSet_A.value[0]","0",e);

// 2) enable the GaN buffer: nEn_uC LOW (GPBCLEAR). DSS cannot call target functions reliably here.
p(">>> enabling buffer: nEn_uC LOW (GPBCLEAR GPIO39) ...");
gateEnableLow();
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var g39=gpio39();
p("    GPIO39 readback = " + g39 + " (expect 0 = buffer ENABLED)");
if(g39!==0) fail("GPIO39 did not go LOW -- buffer not enabled.");

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
if(!(vb>VBUS_MIN)) fail("VdcBus_V="+f(vb,3)+" below "+VBUS_MIN+" -- bus absent. Apply current-limited 24 V.");
if(vb < 18.0) p("  WARNING: VdcBus_V="+f(vb,2)+" V is below the 24 V bench point.");
if(vb > 30.0) p("  WARNING: VdcBus_V="+f(vb,2)+" V is above the 24 V bench point (AM-4116 rated 24 V).");

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
p("    FIRST-RUN checks: is current direction/torque sign sane? rotation as expected? If reversed,");
p("    flip current_sf sign in hal.h (both read fns) + offset sign in user.h, or change PWM_PHASE_ORDER.");

// 7) clean stop
p(""); p(">>> stopping: Iq 0, run off, PWM off, buffer DISABLED (nEn_uC HIGH), flagEnableSys=0.");
stopClean();
var gEnd=gpio39();
p("    final GPIO39 readback = " + gEnd + " (expect 1 = buffer disabled).");
if(gEnd!==1){
    p("  !! GPIO39 is NOT disabled at exit -- cut bus power / reset the board. Exiting nonzero.");
    p("================================================================");
    s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
    java.lang.System.exit(1);
}
p("================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
