/*
 * run_is03.js - drive is03_hardware_test (scalar V/f open-loop spin) on launchxl over the debugger,
 * with EN_GATE bring-up, a safety gate, fault-abort, and a clean stop. First-spin bench helper.
 *
 * Usage: dss.sh tools/flash/run_is03.js <ccxml> <is03_hardware_test.out>  [speedRef_Hz] [duration_s] [accel_Hzps] [align_s]
 *
 * is03 is open-loop V/f: ANGLE_GEN forces the commutation angle, VS_FREQ sets the voltage from the
 * commanded frequency (no current loop, flag_bypassMotorId). It spins once flagRunIdentAndOnLine=1
 * and offset cal is done. There is NO estimated-speed feedback in this lab -- the rotor direction is
 * confirmed VISUALLY; this script provides the electrical readout (balanced 3-phase current, no
 * fault, sane bus). motorVars.accelerationMax_Hzps is 0 unless set, so the ramp would never move --
 * we set it. speedRef defaults to a gentle 10 Hz electrical (AM-4116 7 pole pairs -> ~86 rpm mech).
 *
 * Safety: current-limited supply (bench was 2.5 A), no prop, motor mechanically restrained. The V/f
 * voltage at 10-20 Hz is ~1-2% of full scale; the PSU current limit bounds a stall. Aborts (run off,
 * EN_GATE low, flagEnableSys=0, exit 1) on any latched fault or a failed bring-up check.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2,e2){ try { e2.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function f3(x){ return (isNaN(x)?"nan":x.toFixed(3)); }
// DSS evaluate() truncates float32 -> read floats exactly via &expr + 2-word IEEE754.
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];
var SPEED_REF = (arguments.length > 2) ? Number(arguments[2]) : 10.0;  // Hz electrical
var DURATION_S = (arguments.length > 3) ? Number(arguments[3]) : 8.0;
var ACCEL = (arguments.length > 4) ? Number(arguments[4]) : 5.0;
var ALIGN_S = (arguments.length > 5) ? Number(arguments[5]) : 0.0;
var VBUS_MIN = 5.0;

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function gateLow(){ try { s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16); } catch(x){} }
function stopClean(){
    set("motorVars.flagRunIdentAndOnLine","0",e);
    s.target.runAsynch(); Thread.sleep(200); s.target.halt();   // let bg loop disable PWM
    gateLow();
    set("motorVars.flagEnableSys","0",e);
}
function fail(why){
    p("");
    p("!!!!!!!!!!!! is03 run ABORTED !!!!!!!!!!!!");
    p("  reason: " + why);
    stopClean();
    p("  -> run off, EN_GATE low, flagEnableSys=0, target HALTED.");
    p("==========================================");
    try { s.target.disconnect(); } catch(x){}
    server.stop(); s.terminate();
    java.lang.System.exit(1);
}

p("");
p("============ is03 V/f open-loop spin (speedRef=" + SPEED_REF + " Hz elec) ============");
// 1) dead-wait
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var es0=num("motorVars.flagEnableSys",e), hh=num("halHandle",e);
p("at dead-wait: flagEnableSys=" + es0 + " (0)  halHandle=" + hh + " (nonzero)");
if(!(hh>0)) fail("halHandle invalid -- HAL_init incomplete.");
if(es0!==0) fail("flagEnableSys != 0 -- not parked at the enable-sys wait.");

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
p("post-cal ("+waited.toFixed(1)+"s): flagEnableOffsetCalc="+oc+"  faultUse.all="+fu+"  VdcBus_V="+f3(vb));
if(oc!==0)          fail("offset cal did not complete.");
if(fu!==0)          fail("faultUse.all="+fu+" -- fault latched before spin.");
if(!(vb>VBUS_MIN))  fail("VdcBus_V="+f3(vb)+" below "+VBUS_MIN+".");

// 4) optionally hold a static voltage vector before ramping the open-loop angle.
set("motorVars.accelerationMax_Hzps", ACCEL, e);   // 0 by default -> ramp would never move
set("motorVars.speedRef_Hz", 0.0, e);
p(">>> flagRunIdentAndOnLine=1; static alignment hold "+ALIGN_S+"s, then ramp to "+SPEED_REF+" Hz at "+ACCEL+" Hz/s.");
if(!set("motorVars.flagRunIdentAndOnLine","1",e)) fail("could not set flagRunIdentAndOnLine=1.");
if(ALIGN_S > 0.0) {
    var holdSamples = Math.ceil(ALIGN_S / 0.5);
    for(var h=0; h<holdSamples; h++) {
        s.target.runAsynch(); Thread.sleep(500); s.target.halt();
        var hf=num("motorVars.faultUse.all",e);
        if(hf!==0) fail("faultUse.all="+hf+" latched during static alignment.");
    }
}
set("motorVars.speedRef_Hz", SPEED_REF, e);
p(">>> ramping now. WATCH THE ROTOR.");

// 5) monitor ~8 s: per-phase current envelope, commanded freq, bus, faults. Abort on any fault.
var lo=[1e9,1e9,1e9], hi=[-1e9,-1e9,-1e9];
p("");
p("  t(s) | spdTraj |   Ia      Ib      Ic    | VdcBus | faultUse");
var samples = Math.max(1, Math.ceil(DURATION_S / 0.5));
for(var i=0;i<samples;i++){
    s.target.runAsynch(); Thread.sleep(500); s.target.halt();
    var ia=getf("adcData.I_A.value[0]"), ib=getf("adcData.I_A.value[1]"), ic=getf("adcData.I_A.value[2]");
    var ic_arr=[ia,ib,ic];
    for(var ph=0;ph<3;ph++){ if(ic_arr[ph]<lo[ph])lo[ph]=ic_arr[ph]; if(ic_arr[ph]>hi[ph])hi[ph]=ic_arr[ph]; }
    var st=getf("motorVars.speedTraj_Hz"), vbn=getf("motorVars.VdcBus_V"), fn=num("motorVars.faultUse.all",e);
    p("  "+(i*0.5).toFixed(1)+" | "+f3(st)+" | "+f3(ia)+"  "+f3(ib)+"  "+f3(ic)+" | "+f3(vbn)+" | "+fn);
    if(fn!==0) fail("faultUse.all="+fn+" latched during spin (t="+(i*0.5).toFixed(1)+"s).");
}

// 6) report current envelope (rough phase-current amplitude at steady V/f)
p("");
p("--- phase-current envelope over the run (A, balanced 3-phase expected) ---");
var pn=["A","B","C"];
for(var ph=0;ph<3;ph++)
    p("    phase "+pn[ph]+": ["+f3(lo[ph])+" .. "+f3(hi[ph])+"]  pk-pk="+f3(hi[ph]-lo[ph]));

// 7) clean stop
p("");
p(">>> stopping: run off, PWM off, EN_GATE low, flagEnableSys=0.");
stopClean();
p(">>> is03 spin complete. Judge: did the rotor turn smoothly? currents balanced + bounded?");
p("    Direction is visual. If wrong, swap any two motor phases (phase-order fix) for closed-loop labs.");
p("=================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
