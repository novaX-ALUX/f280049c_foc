/*
 * run_is05.js - drive is05_motor_id (FAST current-controlled identification) on launchxl over the
 * debugger: EN_GATE bring-up, safety gate, run the ID state machine to completion, read back the
 * identified Rs / Ls_d / Ls_q / flux, with fault-abort and a clean stop.
 *
 * Usage: dss.sh tools/flash/drv8305evm/run_is05.js <ccxml> <is05_motor_id.out>
 *
 * FAST commands CURRENT (RES_EST_CURRENT 1.0 A, IND_EST_CURRENT -1.0 A; clamp MAX_CURRENT 5.0 A),
 * so it is inherently safe for an ultra-low-Rs motor (unlike is03 V/f, where 4 V across 20 mO is an
 * instant overcurrent). The rotor SPINS during the RampUp / flux stages -- no prop, restrained.
 * Progress is motorVars.estState (EST_State enum). Identified params land in motorVars.Rs_Ohm,
 * Ls_d_H, Ls_q_H, flux_VpHz; flagMotorIdentified latches and flagRunIdentAndOnLine self-clears.
 *
 * Bench: current-limited supply (2.5 A here). If ID faults on a bus sag during RampUp, raise the
 * limit toward USER_MOTOR_MAX_CURRENT_A (5 A) and retry. Aborts (run off, EN_GATE low,
 * flagEnableSys=0, exit 1) on any latched fault or a failed bring-up check.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2,e2){ try { e2.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function f(x,n){ return (isNaN(x)?"nan":x.toFixed(n)); }
// DSS evaluate() truncates float32 -> read floats exactly via &expr + 2-word IEEE754.
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var EST=["Error","Idle","RoverL","Rs","RampUp","IdRated","RatedFlux_OL","RatedFlux",
         "RampDown","LockRotor","MotorIdentified","OnLine"];
function estName(i){ return (i>=0 && i<EST.length) ? EST[i] : ("st"+i); }

var ccxml=arguments[0], out=arguments[1];
var VBUS_MIN=5.0;
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(180000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function gateLow(){ try { s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16); } catch(x){} }
function stopClean(){
    set("motorVars.flagRunIdentAndOnLine","0",e);
    s.target.runAsynch(); Thread.sleep(200); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0",e);
}
function fail(why){
    p(""); p("!!!!!!!!!!!! is05 motor ID ABORTED !!!!!!!!!!!!"); p("  reason: " + why);
    stopClean();
    p("  -> run off, EN_GATE low, flagEnableSys=0, target HALTED.");
    p("================================================");
    try { s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p(""); p("============ is05 FAST motor identification ============");
// 1) dead-wait
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (0)  halHandle=" + hh + " (nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init incomplete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at enable-sys wait.");

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
if(fu!==0)         fail("faultUse.all="+fu+" -- fault latched before ID.");
if(!(vb>VBUS_MIN)) fail("VdcBus_V="+f(vb,3)+" below "+VBUS_MIN+".");

// 4) start identification. Motor will spin during RampUp/flux.
p(">>> flagRunIdentAndOnLine=1; running FAST identification. ROTOR WILL SPIN (no prop).");
if(!set("motorVars.flagRunIdentAndOnLine","1",e)) fail("could not set flagRunIdentAndOnLine=1.");

// 5) monitor the state machine up to ~75s; abort on fault; stop when identified.
p("");
p("  t(s) | estState        | Rs_mOhm  Lsd_uH  Lsq_uH  flux_VpHz | spd_Hz  Vs_V  magI_A | flt");
var identified=0;
for(var i=0;i<50;i++){
    s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
    var fn=num("motorVars.faultUse.all",e);
    var st=num("motorVars.estState",e);
    var rs=getf("motorVars.Rs_Ohm"), lsd=getf("motorVars.Ls_d_H"), lsq=getf("motorVars.Ls_q_H");
    var fx=getf("motorVars.flux_VpHz"), sp=getf("motorVars.speed_Hz"), vs=getf("motorVars.Vs_V");
    var mi=getf("motorVars.magneticCurrent_A");
    p("  "+(i*1.5).toFixed(1)+" | "+estName(st)+"        ".substring(0,Math.max(0,15-estName(st).length))+
      " | "+f(rs*1000,2)+"  "+f(lsd*1e6,1)+"  "+f(lsq*1e6,1)+"  "+f(fx,4)+
      " | "+f(sp,1)+"  "+f(vs,2)+"  "+f(mi,2)+" | "+fn);
    if(fn!==0) fail("faultUse.all="+fn+" latched during ID (t="+(i*1.5).toFixed(1)+"s).");
    if(num("motorVars.flagMotorIdentified",e)===1){ identified=1; break; }
}
if(!identified) fail("ID did not complete within "+(50*1.5)+"s (still running). Check current limit / bus.");

// 6) report identified parameters (exact)
var rs=getf("motorVars.Rs_Ohm"), lsd=getf("motorVars.Ls_d_H"), lsq=getf("motorVars.Ls_q_H");
var fx=getf("motorVars.flux_VpHz"), fwb=getf("motorVars.flux_Wb");
p("");
p(">>> MOTOR IDENTIFIED <<<");
p("  Rs       = " + f(rs,5)   + " Ohm   (" + f(rs*1000,2) + " mOhm)");
p("  Ls_d     = " + f(lsd*1e6,2) + " uH");
p("  Ls_q     = " + f(lsq*1e6,2) + " uH");
p("  flux     = " + f(fx,5)   + " V/Hz   (" + f(fwb,6) + " Wb)");
p("  -> motors/am_4116_kv450.h: USER_MOTOR_Rs_Ohm=" + f(rs,5) +
   ", Ls_d_H=" + f(lsd,8) + ", Ls_q_H=" + f(lsq,8) + ", RATED_FLUX_VpHz=" + f(fx,5));
p("  (back these into the motor profile; re-verify with a closed-loop lab.)");

// 7) clean stop
p(""); p(">>> stopping: run off, PWM off, EN_GATE low, flagEnableSys=0.");
stopClean();
p("=======================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
