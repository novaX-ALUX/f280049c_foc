/* Current-sense SCALE calibration. Holds a FIXED forced angle (field along the alpha/phase-A axis)
 * and commands Id = 1A, 2A, 3A in turn, holding each steadily so you can read a DC CLAMP meter on the
 * phase wires (or scope a low-side shunt / DRV8305 SOx). Compare the meter to the firmware's
 * adcData.I_A printed here: at angle 0, Id splits as Ia=+Id, Ib=Ic=-Id/2. If the clamp on the
 * max-current phase reads ~Id -> scale correct; if it reads N*Id -> USER_CURRENT_SF is off by N.
 * Fixed angle => DC current => safe to halt briefly to read (PWM free-runs, current persists).
 * Usage: dss.sh tools/flash/run_curcal.js <ccxml> <is04.out> [holdSec=8]
 * Safety: current-limited supply, no prop, clamped. Aborts on fault. Register EN_GATE. */
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
var HOLD=(arguments.length>2)?Number(arguments[2]):8;
var LEVELS=[1.0,2.0,3.0];
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

p(""); p("==== current-sense scale calibration (fixed angle, Id steps) ====");
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
    s.target.runAsynch(); Thread.sleep(1500); s.target.halt();   // settle
    if(num("motorVars.faultUse.all")!==0) fail("fault at Id="+Id);
    setfA(aAddr,0.0);
    var ia=getf("adcData.I_A.value[0]"), ib=getf("adcData.I_A.value[1]"), ic=getf("adcData.I_A.value[2]");
    var idin=getf("Idq_in_A.value[0]"), iqin=getf("Idq_in_A.value[1]"), vs=getf("motorVars.Vs_V");
    p("");
    p("#### Id cmd = "+Id+" A   (expect on phases: one ~+"+Id+", two ~-"+(Id/2)+") ####");
    p("   firmware adcData.I_A = "+f(ia,2)+" "+f(ib,2)+" "+f(ic,2)+" A   Idq_in="+f(idin,2)+"/"+f(iqin,2)+"   Vs="+f(vs,2));
    p("   >>> READ DC CLAMP on each phase wire NOW ("+HOLD+"s). Compare clamp vs the firmware values above.");
    s.target.runAsynch(); Thread.sleep(HOLD*1000); s.target.halt();   // FOC actively holds Id -> steady DC for the clamp
    if(num("motorVars.faultUse.all")!==0) fail("fault during hold Id="+Id);
}
p("");
p(">>> JUDGE: clamp on the max-current phase vs firmware value:");
p(">>>   match  -> current scale OK;   clamp = N x firmware -> USER_CURRENT_SF off by N.");
stopClean();
p("==================================================================");
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
