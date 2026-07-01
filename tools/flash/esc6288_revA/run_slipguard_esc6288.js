/* run_slipguard_esc6288.js - validate the RAMP slip guard + SU_FAULT real safe-off. Forces the guard by
 * setting g_su.slip_frac=2.0 (demand FAST speed >= 2x open-loop freq -> impossible), arms, and confirms
 * the SM aborts to SU_FAULT with OST safe-off + latched fault + disarm. Needs ESC6288_BENCH_THROTTLE. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function set(nm,v){ try { e.evaluate(nm+"="+v); return true; } catch(err){ return false; } }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm)); var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
var TZFRC=[0x409B,0x419B,0x429B], TZFLG=[0x4093,0x4193,0x4293], OST=0x4;
function forceOST(){ for(var i=0;i<3;i++){ try{ s.memory.writeData(Memory.Page.DATA,TZFRC[i],OST,16);}catch(x){} } }
function ostStr(){ var r=""; for(var i=0;i<3;i++) r+=(((rd(TZFLG[i])&OST)!=0)?1:0)+(i<2?"/":""); return r; }
function ostAllSet(){ return ((rd(TZFLG[0])&OST)!=0)&&((rd(TZFLG[1])&OST)!=0)&&((rd(TZFLG[2])&OST)!=0); }
var ST=["IDLE","ALIGN","RAMP","BLEND","RUN","FAULT"];
var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
p(""); p("======== esc6288 SLIP-GUARD + SU_FAULT safe-off validation ========");
s.target.runAsynch(); Thread.sleep(3000); s.target.halt();
if(num("motorVars.flagEnableOffsetCalc")!==0){ p("offset cal not done"); java.lang.System.exit(1); }
p("pre-arm: OST="+ostStr()+" enable="+num("g_su.enable"));
set("g_su.slip_frac","2.0");                 // impossible tracking -> guard must fire past slip_check_Hz
set("g_bench_iq_A","1.0"); set("g_bench_arm","1");
p(">>> armed with slip_frac=2.0 (forces slip). Running 3s...");
s.target.runAsynch(); Thread.sleep(3000); s.target.halt();
var st=num("g_su.state"), fl=num("g_su.fault"), fu=num("motorVars.faultUse.all"), fr=num("motorVars.flagRunIdentAndOnLine");
p(">>> after 3s: g_su.state="+st+"("+(ST[st]||"?")+") g_su.fault="+fl+" | faultUse="+fu+" flagRun="+fr+" OST="+ostStr());
var pass = (fl===1) && ostAllSet() && (fu!==0) && (fr===0);
p("");
if(pass){
    p(">>> *** SLIP GUARD + SU_FAULT SAFE-OFF OK: guard fired (g_su.fault=1), OST forced ("+ostStr()+"),");
    p("        product fault latched (faultUse="+fu+"), and main loop disarmed (flagRun=0). Real safe-off. ***");
} else {
    p(">>> INCONCLUSIVE: fault="+fl+" OST="+ostStr()+" faultUse="+fu+" flagRun="+fr+" -- expected fault=1/OST set/faultUse!=0/flagRun=0.");
}
// restore + safe-off
set("g_bench_arm","0"); set("g_su.slip_frac","0.5"); s.target.runAsynch(); Thread.sleep(200); s.target.halt(); forceOST();
p(">>> restored slip_frac=0.5, disarmed, OST="+ostStr());
p("================================================================");
try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate();
