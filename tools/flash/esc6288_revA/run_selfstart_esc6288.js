/* run_selfstart_esc6288.js - validate the PRODUCT open-loop I/f self-start (g_su state machine).
 * Needs product built with --define=ESC6288_BENCH_THROTTLE. Sets g_bench_iq_A + g_bench_arm=1 to arm +
 * command a small torque (bypassing CAN); the startup SM runs ALIGN->RAMP->handoff->RUN in the ISR.
 * Success = g_su.state reaches RUN(3), fault=0, motor spinning. Continuous (no mid halts). No prop, 12V.
 * Usage: dss.sh run_selfstart_esc6288.js <ccxml> <product.out> [iq_A=1.0] [run_s=4] */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function set(nm,v){ try { e.evaluate(nm+"="+v); return true; } catch(err){ return false; } }
function f(x,n){ return (isNaN(x)?"nan":(x>=0?" ":"")+x.toFixed(n)); }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
var TZFRC=[0x409B,0x419B,0x429B], TZFLG=[0x4093,0x4193,0x4293], OST=0x4;
function forceOST(){ for(var i=0;i<3;i++){ try{ s.memory.writeData(Memory.Page.DATA,TZFRC[i],OST,16);}catch(x){} } }
function ostStr(){ var r=""; for(var i=0;i<3;i++) r+=(((rd(TZFLG[i])&OST)!=0)?1:0)+(i<2?"/":""); return r; }
function ostAllSet(){ return ((rd(TZFLG[0])&OST)!=0)&&((rd(TZFLG[1])&OST)!=0)&&((rd(TZFLG[2])&OST)!=0); }
var ST=["IDLE","ALIGN","RAMP","BLEND","RUN","FAULT"];
var ccxml=arguments[0], out=arguments[1];
var IQ=(arguments.length>2)?Number(arguments[2]):1.0;
var RUNS=(arguments.length>3)?Number(arguments[3]):4;
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function safeOff(){ set("g_bench_arm","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt(); forceOST(); }
function fail(why){ p(""); p("!!! SELF-START ABORTED: "+why); safeOff(); p("  -> OST "+ostStr());
    try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }
function rdsu(tag){ var st=num("g_su.state"), fr=getf("g_su.freq_Hz"), fl=num("g_su.fault");
    var kr=getf("motorVars.speed_krpm"), idm=getf("Idq_in_A.value[0]"), iqm=getf("Idq_in_A.value[1]");
    var fu=num("motorVars.faultUse.all");
    p("  ["+tag+"] su.state="+st+"("+(ST[st]||"?")+") freq="+f(fr,1)+"Hz fault="+fl+" | speed="+f(kr*1000,0)+"rpm Id="+f(idm,2)+" Iq="+f(iqm,2)+" | OST="+ostStr()+" faultUse="+fu);
    if(fu!==0) fail("faultUse="+fu+" ("+tag+")"); return {st:st,kr:kr,fl:fl}; }
p(""); p("======== esc6288 PRODUCT open-loop I/f SELF-START test (Iq="+IQ+"A) ========");
s.target.runAsynch(); Thread.sleep(3000); s.target.halt();   // let product self-arm sys + offset cal
if(!(num("halHandle")>0)) fail("halHandle invalid.");
var oc=num("motorVars.flagEnableOffsetCalc"); var en=num("g_su.enable");
p("pre-arm: offsetCalc="+oc+" g_su.enable="+en+" g_su.state="+num("g_su.state")+" OST="+ostStr()+" VdcBus="+f(getf("motorVars.VdcBus_V"),2));
if(oc!==0) fail("offset cal not done."); if(en!==1) fail("g_su.enable != 1 (I/f startup off)."); if(!ostAllSet()) fail("OST not set pre-arm.");
p(">>> g_bench_iq_A="+IQ+", g_bench_arm=1 -> arm + trigger startup. Running "+RUNS+"s continuous...");
set("g_bench_iq_A", IQ); set("g_bench_arm","1");
s.target.runAsynch(); Thread.sleep(RUNS*1000); s.target.halt();
p(">>> after "+RUNS+"s:");
var r=rdsu("end");
p("");
if(r.st===4 && r.fl===0 && Math.abs(r.kr)>0.05){
    p(">>> *** PRODUCT SELF-START OK: reached RUN (FAST handed off), spinning "+f(r.kr*1000,0)+" rpm, no fault ***");
} else if(r.st===5){
    p(">>> STARTUP STALLED (SU_FAULT): open-loop ramp did not cohere / slipped (safe-off). Tune g_su (slower accel / more id_ramp_A / lower handoff_Hz).");
} else {
    p(">>> state="+(ST[r.st]||r.st)+" speed="+f(r.kr*1000,0)+"rpm -- not a clean handoff; inspect.");
}
p(""); p(">>> disarm + safe-off."); safeOff();
p("    final OST="+ostStr()+" g_su.state="+num("g_su.state"));
p("================================================================");
try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate();
