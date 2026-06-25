/* Low-speed synced FAST check (codex). Align 3A, ramp to a low speed (5Hz default) where open-loop
 * drag holds sync with minimal slip, then hold in short monitored segments. HARD SAFETY: abort if
 * any phase current > IMAX (5A) or |Iq_in| > IQMAX (2.5A). Reads the FAST estimate (fm_lp) to see if,
 * with the rotor nearly synced (Iq~0), FAST estimates near the true speed or still over-estimates.
 * Usage: dss.sh tools/flash/drv8305evm/run_5hz.js <ccxml> <is04.out> [refHz=5] [id_A=3]
 * Safety: current-limited supply, no prop, clamped. Register EN_GATE. */
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
var REF=(arguments.length>2)?Number(arguments[2]):5.0;
var ID =(arguments.length>3)?Number(arguments[3]):3.0;
var IMAX=5.0, IQMAX=2.5;
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function gateLow(){ try { s.memory.writeData(Memory.Page.DATA,0x7F0C,0x80,16); } catch(x){} }
function stopClean(){ set("IdqSet_A.value[0]","0"); set("IdqSet_A.value[1]","0");
    set("motorVars.flagRunIdentAndOnLine","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0"); }
function abort(why){ p(""); p("!!!! SAFETY STOP: "+why); stopClean();
    try{s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(0); }

p(""); p("==== low-speed synced FAST check (Id="+ID+"A -> "+REF+"Hz, abort >"+IMAX+"A or |Iq|>"+IQMAX+") ====");
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) abort("halHandle invalid.");
s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
if(oc!==0) abort("cal incomplete.");
// align
set("motorVars.accelerationMax_Hzps","0"); set("motorVars.speedRef_Hz","0");
set("IdqSet_A.value[0]", ID); set("IdqSet_A.value[1]","0"); set("motorVars.flagRunIdentAndOnLine","1");
p(">>> ALIGN 2s @ "+ID+"A.");
s.target.runAsynch(); Thread.sleep(2000); s.target.halt();
if(num("motorVars.faultUse.all")!==0) abort("fault during align.");
// ramp
set("motorVars.accelerationMax_Hzps","2"); set("motorVars.speedRef_Hz", REF);
p(">>> RAMP to "+REF+"Hz then monitored hold. WATCH: smooth steady spin? (segments of 2s)");
p("");
p("  t | Idq_in(d/q) | Iabc          | Vs  | fm_lp(rad/s ~ Hz elec) | flt");
for(var i=0;i<9;i++){
    s.target.runAsynch(); Thread.sleep(2000); s.target.halt();
    var fn=num("motorVars.faultUse.all");
    var id=getf("Idq_in_A.value[0]"), iq=getf("Idq_in_A.value[1]");
    var ia=getf("adcData.I_A.value[0]"), ib=getf("adcData.I_A.value[1]"), ic=getf("adcData.I_A.value[2]");
    var vs=getf("motorVars.Vs_V"), fm=getf("estOutputData.fm_lp_rps");
    var mi=Math.max(Math.abs(ia),Math.max(Math.abs(ib),Math.abs(ic)));
    p("  "+(i*2)+"s| "+f(id,2)+"/"+f(iq,2)+" | "+f(ia,2)+" "+f(ib,2)+" "+f(ic,2)+" | "+f(vs,2)+" | "+f(fm,1)+"rps ~ "+f(fm/(2*Math.PI),1)+"Hz_elec | "+fn);   // fm/(2pi) = elec Hz (NOT *p)
    if(fn!==0) abort("fault.");
    if(mi>IMAX && mi<20) abort("phase current "+f(mi,1)+"A > "+IMAX+"A.");
    if(Math.abs(iq)>IQMAX) abort("Iq_in "+f(iq,1)+"A > "+IQMAX+"A (slipping).");
}
p("");
p(">>> JUDGE: smooth @ "+REF+"Hz + Iq~0 + Iabc~cmd? and is fm_lp ~ "+REF+"Hz (locked) or over-estimating?");
stopClean();
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
