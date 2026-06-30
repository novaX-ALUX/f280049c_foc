/* cal_is02_esc6288.js - is02 ADC offset cal on esc6288 (no spin, no EN_GATE; gates stay OST-tripped,
 * the external INA181 sense is always live so zero-current offset cal runs with gates off). Reports
 * the computed current/voltage offsets, fault/bus health. Forces OST safe-off on exit. */
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
var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function fail(why){ p(""); p("!!! is02 cal FAILED: "+why); forceOST(); set("motorVars.flagEnableSys","0");
    p("  -> OST forced ("+ostStr()+"), flagEnableSys=0."); try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }
p(""); p("======== is02 ADC offset cal (esc6288, no spin) ========");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
var hh=num("halHandle"), es0=num("motorVars.flagEnableSys");
p("dead-wait: halHandle="+hh+" (nonzero)  flagEnableSys="+es0+" (0)  OST="+ostStr());
if(!(hh>0)) fail("halHandle invalid.");
set("flagEnableOffsetCalibration","0");   // one-shot cal so offsets freeze
if(!set("motorVars.flagEnableSys","1")) fail("could not set flagEnableSys=1.");
p(">>> flagEnableSys=1; one-shot offset cal (gates stay OST-tripped) ...");
var IRAW=[0x0B20,0x0B00,0x0B40], clo=[65535,65535,65535], chi=[0,0,0];   // esc6288 IA/IB/IC SOC0
var oc=1, waited=0.0;
for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt(); waited+=0.7;
    for(var ph=0;ph<3;ph++){ var c=rd(IRAW[ph]); if(c<clo[ph])clo[ph]=c; if(c>chi[ph])chi[ph]=c; }
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
p("post-cal ("+waited.toFixed(1)+"s): flagEnableOffsetCalc="+oc+" (expect 0)  OST="+ostStr());
if(oc!==0) fail("offset cal did not complete.");
var fu=num("motorVars.faultUse.all"), vb=getf("motorVars.VdcBus_V");
p("health: faultUse.all="+fu+" (0)  VdcBus_V="+f(vb,3)+" (>5)");
if(fu!==0) fail("faultUse.all="+fu+" latched.");
p("");
p("--- current offsets motorVars.offsets_I_A (A, zero-current midpoint) ---");
p("    Ia="+f(getf("motorVars.offsets_I_A.value[0]"),3)+"  Ib="+f(getf("motorVars.offsets_I_A.value[1]"),3)+"  Ic="+f(getf("motorVars.offsets_I_A.value[2]"),3));
p("    raw zero-current counts during cal: IA["+clo[0]+".."+chi[0]+"] IB["+clo[1]+".."+chi[1]+"] IC["+clo[2]+".."+chi[2]+"]  (expect ~2048, tight spread)");
p("--- voltage offsets motorVars.offsets_V_V (V) ---");
p("    Va="+f(getf("motorVars.offsets_V_V.value[0]"),3)+"  Vb="+f(getf("motorVars.offsets_V_V.value[1]"),3)+"  Vc="+f(getf("motorVars.offsets_V_V.value[2]"),3));
p("");
forceOST(); set("motorVars.flagRunIdentAndOnLine","0"); set("motorVars.flagEnableSys","0");
p(">>> is02 cal done. safe-off: OST="+ostStr()+". Judge offsets ~0.5*FS midpoint, tight 3-phase spread.");
p("================================================================");
try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate();
