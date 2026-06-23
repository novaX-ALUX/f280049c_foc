/* is05 RampUp-ONLY controlled validation (codex). Forces the RAMPUP d-axis drag current via the
 * bench global g_rampup_id_A, runs the ID FREE (no halt chopping) just long enough to cover Rs +
 * RampUp, then halts and reports. Does NOT complete the full (destructive) ID -- stops after RampUp.
 * Pass/judge: (1) rotor spins up CONTINUOUSLY (your eyes), (2) speedEst tracks RampUp (~20Hz, not
 * garbage), (3) Vs not saturated & faultUse=0.
 * Usage: dss.sh tools/flash/run_is05_rampup.js <ccxml> <is05.out> <rampupId_A> [runSec=16]
 * Safety: current-limited supply, no prop, clamped. Register EN_GATE. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2){ try { return Number(e.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2){ try { e.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function addrOf(nm){ return Number(e.evaluate("&"+nm)); }
function getf(nm){ try { var a=addrOf(nm); var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }
function setfA(a,v){ var b=java.lang.Float.floatToIntBits(v);
    s.memory.writeData(Memory.Page.DATA,a,b&0xFFFF,16); s.memory.writeData(Memory.Page.DATA,a+1,(b>>16)&0xFFFF,16); }
function f(x,n){ return (isNaN(x)?"nan":(x>=0?" ":"")+x.toFixed(n)); }
var STNM={1:"IDLE",2:"ROVERL",3:"Rs",4:"RAMPUP",5:"CONSTSPD",6:"IdRated",7:"FLUX_OL",8:"RATEDFLUX",
          9:"RAMPDOWN",10:"LOCKROTOR",11:"Ls",12:"Rr",13:"*MOTOR_ID*",14:"ONLINE"};
function stnm(v){ return STNM[v]?STNM[v]:("?"+v); }

var ccxml=arguments[0], out=arguments[1];
var RID=Number(arguments[2]);
var RUN=(arguments.length>3)?Number(arguments[3]):16;
if(isNaN(RID)||RID<=0||RID>3.5){ p("FATAL: rampupId out of (0,3.5]"); java.lang.System.exit(1); }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout((RUN+60)*1000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function gateLow(){ try { s.memory.writeData(Memory.Page.DATA,0x7F0C,0x80,16); } catch(x){} }
function stopClean(){ set("motorVars.flagRunIdentAndOnLine","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0"); }

p(""); p("==== is05 RampUp-ONLY (forced drag Id="+RID+"A) ====");
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)){ p("halHandle invalid"); stopClean(); s.terminate(); }
s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
if(oc!==0){ p("cal incomplete"); stopClean(); s.terminate(); }
if(num("motorVars.faultUse.all")!==0){ p("fault before ID"); stopClean(); s.terminate(); }

setfA(addrOf("g_rampup_id_A"), RID);
p("   set g_rampup_id_A = "+f(getf("g_rampup_id_A"),2)+" A (readback)");
set("motorVars.flagEnableForceAngle","1");
set("motorVars.speedRef_Hz","20");
set("motorVars.flagRunIdentAndOnLine","1");
p(">>> ID started. Polling lightly for RAMPUP (Rs/ROVERL are DC, chopping harmless)...");
var reached=false;
for(var i=0;i<60;i++){            // up to ~48s to reach RAMPUP
    s.target.runAsynch(); Thread.sleep(800); s.target.halt();
    var st0=num("motorVars.estState");
    if(num("motorVars.faultUse.all")!==0){ p("   fault in state "+stnm(st0)); break; }
    if(st0===4){ reached=true; p("   -> RAMPUP reached at ~"+(i*0.8).toFixed(0)+"s. Now FREE-run "+RUN+"s. WATCH ROTOR!"); break; }
    if(st0>4){ p("   -> already past RAMPUP (state "+stnm(st0)+") -- RampUp was too quick to catch."); break; }
}
if(reached){ s.target.runAsynch(); Thread.sleep(RUN*1000); s.target.halt(); }

var st=num("motorVars.estState"), fn=num("motorVars.faultUse.all"), sysEn=num("motorVars.flagEnableSys");
p("");
p(">>> AFTER "+RUN+"s:");
p("   estState   = "+stnm(st)+"   faultUse="+fn+"   sysEn="+sysEn);
p("   speedEst   = "+f(getf("motorVars.speedEst_Hz"),1)+" Hz   speed_ref="+f(getf("motorVars.speed_Hz"),1)+" Hz");
p("   Idq_in     = "+f(getf("Idq_in_A.value[0]"),2)+"/"+f(getf("Idq_in_A.value[1]"),2)+"   Vs="+f(getf("motorVars.Vs_V"),2));
p("   Iabc       = "+f(getf("adcData.I_A.value[0]"),2)+" "+f(getf("adcData.I_A.value[1]"),2)+" "+f(getf("adcData.I_A.value[2]"),2));
p("   --- RAMPUP capture (is04-vs-is05 compare) ---");
p("   angle freq (actual) = "+f(getf("g_dbg_angFreq"),2)+" Hz   speed_ref = "+f(getf("g_dbg_speedRef"),2)+" Hz");
p("   RampUp Id/Iq        = "+f(getf("g_dbg_id"),2)+" / "+f(getf("g_dbg_iq"),2)+" A   speedEst = "+f(getf("g_dbg_speedEst"),2)+" Hz");
p("   (is04 worked at: angle freq ~20Hz, Id~3A. Compare: which differs?)");
p(">>> PASS if: rotor spun up continuously (eyes) + speedEst ~ RampUp speed (not 0/garbage) + Vs not maxed + flt=0.");
stopClean();
p("==================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
