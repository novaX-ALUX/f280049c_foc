/* run_is05_cont.js - is05 FAST motor ID on esc6288, CONTINUOUS (no mid-ID halts -- halting desyncs the
 * fragile Ls phase, per Codex). Arms, runs uninterrupted for SECS (default 120, the EST flux+Ls wait
 * tables need ~100s), then halts ONCE to read the result. ID current ~4A (RES/IND_EST). No prop, CC bench.
 * Usage: dss.sh run_is05_cont.js <ccxml> <is05.out> [secs=120] */
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
var TZFRC=[0x409B,0x419B,0x429B], TZFLG=[0x4093,0x4193,0x4293], OST=0x4;
function forceOST(){ for(var i=0;i<3;i++){ try{ s.memory.writeData(Memory.Page.DATA,TZFRC[i],OST,16);}catch(x){} } }
function ostStr(){ var r=""; for(var i=0;i<3;i++) r+=(((rd(TZFLG[i])&OST)!=0)?1:0)+(i<2?"/":""); return r; }
function ostAllSet(){ return ((rd(TZFLG[0])&OST)!=0)&&((rd(TZFLG[1])&OST)!=0)&&((rd(TZFLG[2])&OST)!=0); }
var ccxml=arguments[0], out=arguments[1];
var SECS=(arguments.length>2)?Number(arguments[2]):120;
var env=ScriptingEnvironment.instance(); env.setScriptTimeout((SECS+60)*1000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function safeOff(){ set("motorVars.speedRef_Hz","0"); set("motorVars.flagRunIdentAndOnLine","0");
    s.target.runAsynch(); Thread.sleep(400); s.target.halt(); forceOST(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!! is05 ABORTED: "+why); safeOff();
    p("  -> OST forced ("+ostStr()+"), disarmed."); try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }
p(""); p("======== esc6288 is05 FAST ID, CONTINUOUS "+SECS+"s (no mid-ID halts) ========");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
if(num("motorVars.flagEnableSys")!==0) fail("not parked at enable-sys wait.");
if(!ostAllSet()) fail("OST not set at startup.");
set("motorVars.flagEnableSys","1");
p(">>> flagEnableSys=1; offset cal ...");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt(); oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0)break; }
var fu=num("motorVars.faultUse.all"), vb=getf("motorVars.VdcBus_V");
p("post-cal: offsetCalc="+oc+" faultUse="+fu+" VdcBus="+f(vb,2)+" OST="+ostStr());
if(oc!==0) fail("offset cal did not complete."); if(fu!==0) fail("fault pre-ID: "+fu); if(!(vb>5)) fail("VdcBus<5.");
p(">>> flagRunIdentAndOnLine=1; running ID UNINTERRUPTED for "+SECS+"s. WATCH: DC inject -> spin ~167rpm -> Ls.");
set("motorVars.flagRunIdentAndOnLine","1");
s.target.runAsynch(); Thread.sleep(SECS*1000); s.target.halt();   // ONE continuous run, no mid halts
var idn=num("motorVars.flagMotorIdentified"), fn=num("motorVars.faultUse.all");
p("");
p(">>> after "+SECS+"s continuous: flagMotorIdentified="+idn+"  faultUse="+fn+"  speed_Hz="+f(getf("motorVars.speed_Hz"),1));
p("    Rs        = "+f(getf("motorVars.Rs_Ohm"),6)+" Ohm    (profile 0.0213 Y phase-neutral; post-cal ID ~0.0223)");
p("    Ls_d/Ls_q = "+f(getf("motorVars.Ls_d_H")*1e6,3)+" / "+f(getf("motorVars.Ls_q_H")*1e6,3)+" uH   (profile 30.0, post-cal ID ~30)");
p("    flux      = "+f(getf("motorVars.flux_VpHz"),6)+" V/Hz   (profile 0.012)");
p("    IdRated   = "+f(getf("motorVars.IdRated_A"),3)+" A");
if(idn===1) p(">>> *** MOTOR IDENTIFIED — full FAST ID complete! ***");
else p(">>> not yet identified after "+SECS+"s. If speed is sane + no fault, re-run with a longer window; if Ls is garbage, see PORT_TODO Ls note.");
p(""); p(">>> stopping (safe-off)."); safeOff();
p("    final OST="+ostStr()+" faultUse="+num("motorVars.faultUse.all"));
p("================================================================");
try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate();
