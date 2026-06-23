/* FREE-RUN forced-angle drag spin at HIGHER current (under-powering test). Everything in the drive
 * chain is validated (DRV8305 6-PWM/CSA-normal, 7mOhm shunt scale correct, mapping correct); the
 * motor still won't follow rotation at <=1.5A. The legacy "0.4A" was DC input, not phase current, so
 * the real drag current is higher. Align with a strong Id, then ramp the forced angle slowly, ONE
 * uninterrupted run (no halt chopping). WATCH: does the rotor now get dragged SMOOTHLY around?
 * Usage: dss.sh tools/flash/run_draghi.js <ccxml> <is04.out> [id_A=3.0] [refHz=2] [runSec=16]
 * Safety: current-limited supply, no prop, clamped. Aborts on fault. Register EN_GATE. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2){ try { return Number(e.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2){ try { e.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm)); var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }
function f(x,n){ return (isNaN(x)?"nan":(x>=0?" ":"")+x.toFixed(n)); }

var ccxml=arguments[0], out=arguments[1];
var ID  = (arguments.length>2)?Number(arguments[2]):3.0;
var REF = (arguments.length>3)?Number(arguments[3]):2.0;
var RUN = (arguments.length>4)?Number(arguments[4]):16.0;
if(isNaN(ID)||ID<=0||ID>4.0){ p("FATAL: id out of (0,4]"); java.lang.System.exit(1); }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function gateLow(){ try { s.memory.writeData(Memory.Page.DATA,0x7F0C,0x80,16); } catch(x){} }
function stopClean(){ set("IdqSet_A.value[0]","0"); set("IdqSet_A.value[1]","0");
    set("motorVars.flagRunIdentAndOnLine","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!!! ABORT: "+why); stopClean();
    try{s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }

p(""); p("==== HIGH-current drag spin (Id="+ID+"A, ramp->"+REF+"Hz, run "+RUN+"s, NO halts) ====");
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
if(oc!==0) fail("cal incomplete."); if(num("motorVars.faultUse.all")!==0) fail("fault before run.");

// align with strong Id (one short run)
set("motorVars.accelerationMax_Hzps","0"); set("motorVars.speedRef_Hz","0");
set("IdqSet_A.value[0]", ID); set("IdqSet_A.value[1]","0"); set("motorVars.flagRunIdentAndOnLine","1");
p(">>> ALIGN 2s: Id="+ID+"A. Rotor should snap HARD and hold firmly now.");
s.target.runAsynch(); Thread.sleep(2000); s.target.halt();
if(num("motorVars.faultUse.all")!==0) fail("fault during align.");
// rotate: ONE uninterrupted run
set("motorVars.accelerationMax_Hzps","2"); set("motorVars.speedRef_Hz", REF);
p(">>> ROTATE: forced angle 0->"+REF+"Hz, FREE for "+RUN+"s. WATCH: dragged smoothly around now?");
s.target.runAsynch(); Thread.sleep(RUN*1000); s.target.halt();
var fm=getf("estOutputData.fm_lp_rps");
var feHz=fm/(2.0*Math.PI);   // fm_lp_rps -> electrical Hz (do NOT *p; p is only for mech rpm = elec*60/p)
p("");
p(">>> AFTER: cmdFreq="+f(getf("motorVars.speedTraj_Hz"),2)+"Hz  fault="+num("motorVars.faultUse.all"));
p("    Idq_in="+f(getf("Idq_in_A.value[0]"),2)+"/"+f(getf("Idq_in_A.value[1]"),2)
  +"  Vs="+f(getf("motorVars.Vs_V"),2)+"  Iabc="+f(getf("adcData.I_A.value[0]"),2)+" "
  +f(getf("adcData.I_A.value[1]"),2)+" "+f(getf("adcData.I_A.value[2]"),2));
p("    >>> FAST estimate: fm_lp="+f(fm,2)+" rad/s  ~= "+f(feHz,2)+" Hz elec  (cmd "+REF+"Hz)");
p("    >>> if fm WAKES toward "+REF+"Hz -> FAST locks on a synced rotor (good). if ~0 -> FAST still blind.");
p(">>> JUDGE (visual): rotor dragged SMOOTHLY around at "+ID+"A, or still twitch/slip?");
stopClean();
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
