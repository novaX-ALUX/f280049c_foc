/* CSA-output (SOx / ISEN_x) scale check with a multimeter. Holds a FIXED forced angle and steps
 * Id = 0, 1A, 2A with LONG holds so you can probe BOOSTXL J2 pins 14/16/18 (ISEN_A/B/C) to GND:
 *   zero current  -> all three ~1.65 V
 *   Id = 1 A      -> max-current phase ~ +70 mV from 1.65, the other two ~ -35 mV
 *   Id = 2 A      -> roughly double
 * (70 mV/A = 7 mOhm shunt x 10 V/V CSA gain.) Firmware adcData.I_A is printed for cross-check.
 * Usage: dss.sh tools/flash/drv8305evm/run_socal.js <ccxml> <is04.out> [holdSec=22]
 * Safety: current-limited, no prop, clamped. Aborts on fault. Register EN_GATE. */
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

var ccxml=arguments[0], out=arguments[1];
var HOLD=(arguments.length>2)?Number(arguments[2]):22;
var LEVELS=[0.0,1.0,2.0];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(180000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function gateLow(){ try { s.memory.writeData(Memory.Page.DATA,0x7F0C,0x80,16); } catch(x){} }
function stopClean(){ set("IdqSet_A.value[0]","0"); set("IdqSet_A.value[1]","0");
    set("motorVars.flagRunIdentAndOnLine","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!!! ABORT: "+why); stopClean();
    try{s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }

p(""); p("==== SOx/ISEN multimeter scale check (J2 pin14/16/18 vs GND) ====");
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
if(oc!==0) fail("cal incomplete."); if(num("motorVars.faultUse.all")!==0) fail("fault before run.");

var aAddr; try { aAddr=addrOf("angleGen.angle_rad"); } catch(x){ fail("no angleGen.angle_rad"); }
set("motorVars.accelerationMax_Hzps","500"); set("motorVars.speedRef_Hz","0");
set("IdqSet_A.value[0]","0"); set("IdqSet_A.value[1]","0"); set("motorVars.flagRunIdentAndOnLine","1");
s.target.runAsynch(); Thread.sleep(300); s.target.halt(); setfA(aAddr,0.0);

for(var li=0; li<LEVELS.length; li++){
    var Id=LEVELS[li];
    set("IdqSet_A.value[0]", Id); setfA(aAddr,0.0);
    s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
    if(num("motorVars.faultUse.all")!==0) fail("fault at Id="+Id);
    setfA(aAddr,0.0);
    var ia=getf("adcData.I_A.value[0]"), ib=getf("adcData.I_A.value[1]"), ic=getf("adcData.I_A.value[2]");
    p("");
    p("########## Id = "+Id+" A  ##########");
    p("   firmware adcData.I_A = "+f(ia,2)+" "+f(ib,2)+" "+f(ic,2)+" A");
    if(Id===0) p("   EXPECT SOx (J2 14/16/18 vs GND): all three ~1.65 V");
    else p("   EXPECT SOx: one ~1.65+"+(Id*0.07).toFixed(3)+"V, two ~1.65-"+(Id*0.035).toFixed(3)+"V  (70mV/A)");
    p("   >>> MEASURE J2 pin14(ISEN_A) / pin16(ISEN_B) / pin18(ISEN_C) vs GND NOW ("+HOLD+"s) <<<");
    s.target.runAsynch(); Thread.sleep(HOLD*1000); s.target.halt();
    if(num("motorVars.faultUse.all")!==0) fail("fault during hold Id="+Id);
}
p("");
p(">>> Report the 3 SOx voltages at each Id. ~70mV/A around 1.65V => analog scale OK (7mOhm x 10).");
stopClean();
p("==================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
