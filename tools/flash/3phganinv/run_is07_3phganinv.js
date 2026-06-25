/*
 * run_is07_3phganinv.js - speed-control first-spin + phase-order check for the AM-4116 on the
 * BOOSTXL-3PhGaNInv (LAUNCHXL-F280049C), over the debugger. Unlike is06 torque control, is07 gives a
 * real rotating speed reference -> the force-angle/ramp/FAST startup can commutate the motor from
 * standstill. Used to get the FIRST confirmed rotation and decide phase order:
 *   - spins toward +speedRef (estimated speed climbs to the ref)  -> phase order OK
 *   - spins backward / estimated speed goes opposite the ref      -> phase order reversed
 *   - stalls/buzzes, speed PI saturates Iq at the clamp           -> phase order scrambled (swap)
 *
 * Usage: dss.sh tools/flash/3phganinv/run_is07_3phganinv.js <ccxml> <is07_speed_control.out> [speedRef_Hz] [accel_Hzps]
 *        speedRef_Hz default 20 (electrical), HARD max 100. accel default 20 Hz/s.
 *
 * GaN polarity (do NOT reuse the DRV8305 scripts): nEn_uC ACTIVE-LOW.
 *   enable  = GPBCLEAR 0x7F0C bit7 (GPIO39 LOW,  readback 0)
 *   disable = GPBSET   0x7F0A bit7 (GPIO39 HIGH, readback 1)
 *
 * SAFETY: the speed loop generates Iq automatically, clamped to +-USER_MOTOR_MAX_CURRENT_A. For this
 * first GaN spin that clamp must be set LOW (bench build lowers the 4116 to ~1.5 A) so that if the
 * motor will NOT start (e.g. wrong phase order) the speed PI cannot wind up into a large stalled
 * current. This is ENFORCED: the script reads userParams.maxCurrent_A after init and HARD-ABORTS
 * (buffer still disabled) if it exceeds IMAX_GATE (2.0 A) -- speedRef alone does not bound Iq.
 * Run from a current-limited bench supply. Dead-band already scope-verified at 0 V bus.
 * Every fault path disables the buffer FIRST (GPBSET), then reads back; if the disable did not take it
 * does NOT run the target. A final GPIO39 != 1 at exit is a hard nonzero exit.
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
var SPEED_REF = (arguments.length > 2) ? Number(arguments[2]) : 20.0;   // Hz electrical
var ACCEL     = (arguments.length > 3) ? Number(arguments[3]) : 20.0;   // Hz/s
var VBUS_MIN=5.0, POLE_PAIRS=7;
var IMAX_GATE=2.0;   // hard first-spin Iq-clamp ceiling (A); refuse a high-current image
// HARD speed gate BEFORE touching hardware: keep the first spin slow.
if(isNaN(SPEED_REF) || Math.abs(SPEED_REF) > 100.0){
    p("FATAL: speedRef_Hz=" + arguments[2] + " out of range -- require |speedRef| <= 100 Hz (first-spin guard).");
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
    set("motorVars.speedRef_Hz","0",e);
    set("motorVars.flagRunIdentAndOnLine","0",e);
    s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateDisableHigh(); set("motorVars.flagEnableSys","0",e);
}
function fail(why){
    gateDisableHigh();                                  // IMMEDIATE safe-off
    var g=gpio39();                                     // verify BEFORE running the target again
    p(""); p("!!!!!!!!!!!! is07 spin ABORTED !!!!!!!!!!!!"); p("  reason: " + why);
    if(g!==1){
        p("  !! GPIO39 readback=" + g + " (expect 1): disable did NOT take, buffer may be LIVE.");
        p("  !! NOT running the target; flags left as-is, target HALTED. Cut bus power / reset NOW.");
    } else {
        stopClean();
        p("  -> buffer DISABLED (GPIO39=" + gpio39() + "), speedRef 0/run off/flagEnableSys=0, HALTED.");
    }
    p("===============================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("============ GaN is07 speed-control first spin (speedRef=" + SPEED_REF + " Hz elec, accel=" + ACCEL + " Hz/s) ============");
// 1) dead-wait
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (0)  halHandle=" + hh + " (nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init incomplete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at enable-sys wait.");
if(gpio39()!==1) fail("GPIO39 is LOW at the dead-wait -- buffer already enabled; setup must leave it HIGH.");

// HARD current gate (buffer still DISABLED here): the speed PI winds Iq up to
// +-userParams.maxCurrent_A, so speedRef alone does NOT bound current. A stalled / wrong-phase
// rotor would pull the full clamp. Refuse to spin unless the loaded image was built with a low
// clamp. userParams.maxCurrent_A is populated by USER_setParams() during the dead-wait init.
var imax=getf("userParams.maxCurrent_A");
p("max-current clamp: userParams.maxCurrent_A=" + f(imax,2) + " A  (first-spin gate: <= " + IMAX_GATE.toFixed(1) + " A)");
if(isNaN(imax) || imax > IMAX_GATE)
    fail("userParams.maxCurrent_A=" + f(imax,2) + " A exceeds the " + IMAX_GATE.toFixed(1)
       + " A first-spin limit -- rebuild with a low USER_MOTOR_MAX_CURRENT_A (~1.5 A) bench image.");

// pin speed ref to 0 before enabling so the lab's init default (50 Hz) doesn't apply.
set("motorVars.speedRef_Hz","0",e);

// 2) enable the GaN buffer: nEn_uC LOW (GPBCLEAR).
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
if(!(vb>VBUS_MIN)) fail("VdcBus_V="+f(vb,3)+" below "+VBUS_MIN+" -- bus absent.");

// 4) command the speed ramp: accel must be nonzero, then set the speed reference.
set("motorVars.accelerationMax_Hzps", ACCEL, e);
set("motorVars.speedRef_Hz", SPEED_REF, e);
p(">>> flagRunIdentAndOnLine=1; speedRef="+SPEED_REF+" Hz. WATCH THE ROTOR (direction!).");
if(!set("motorVars.flagRunIdentAndOnLine","1",e)) fail("could not set flagRunIdentAndOnLine=1.");

// 5) monitor ~10 s: ref-ramp vs estimated speed + Iq + faults. Abort on fault.
p("");
p("  t(s) | spdRef spdTraj | spdEst_Hz spd_krpm | Iq_A   Vs_V  VdcBus | flt");
for(var i=0;i<20;i++){
    s.target.runAsynch(); Thread.sleep(500); s.target.halt();
    var fn=num("motorVars.faultUse.all",e);
    var st=getf("motorVars.speedTraj_Hz"), se=getf("motorVars.speed_Hz"), kr=getf("motorVars.speed_krpm");
    var iq=getf("Idq_in_A.value[1]"), vs=getf("motorVars.Vs_V"), vbn=getf("motorVars.VdcBus_V");
    p("  "+(i*0.5).toFixed(1)+" | "+f(SPEED_REF,0)+"  "+f(st,1)+" | "+f(se,1)+"  "+f(kr,3)+" | "+f(iq,2)+"  "+f(vs,2)+"  "+f(vbn,2)+" | "+fn);
    if(fn!==0) fail("faultUse.all="+fn+" latched during spin (t="+(i*0.5).toFixed(1)+"s).");
}

// 6) verdict hint from the estimated speed vs reference
var seF=getf("motorVars.speed_Hz"), krF=getf("motorVars.speed_krpm");
p("");
p(">>> settled estimated speed = " + f(seF,1) + " Hz elec  (" + f(krF,3) + " krpm mech, "+POLE_PAIRS+" pp), ref " + SPEED_REF + " Hz");
p("    Judge from the ROTOR: forward to ref => phase order OK; backward => reversed (swap two leads or");
p("    change PWM_PHASE_ORDER); buzz/stall with Iq pinned at the clamp => scrambled order (try another).");

// 7) clean stop
p(""); p(">>> stopping: speedRef 0, run off, PWM off, buffer DISABLED (nEn_uC HIGH), flagEnableSys=0.");
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
