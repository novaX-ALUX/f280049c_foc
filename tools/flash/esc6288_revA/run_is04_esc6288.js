/* run_is04_esc6288.js - is04 signal-chain test on esc6288: OPEN-LOOP forced-angle spin with a SPEED
 * RAMP (align at speedRef=0, then ramp 1->20 Hz so the rotor synchronizes). Current loop bounds Iq.
 * No prop, current-limited. OST safe-off. */
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
var IQ=(arguments.length>2)?Number(arguments[2]):0.5;
if(isNaN(IQ)||IQ<0||IQ>1.0){ p("FATAL: iq out of range 0..1.0"); java.lang.System.exit(1); }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function safeOff(){ set("motorVars.speedRef_Hz","0"); set("IdqSet_A.value[1]","0"); set("motorVars.flagRunIdentAndOnLine","0");
    s.target.runAsynch(); Thread.sleep(300); s.target.halt(); forceOST(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!! is04 ABORTED: "+why); safeOff();
    p("  -> OST forced ("+ostStr()+"), disarmed."); try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }
p(""); p("======== esc6288 is04 signal-chain (open-loop RAMP spin, Iq="+IQ+" A) ========");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
if(num("motorVars.flagEnableSys")!==0) fail("not parked at enable-sys wait.");
if(!ostAllSet()) fail("OST not set at startup.");
set("IdqSet_A.value[1]","0"); set("IdqSet_A.value[0]","0"); set("motorVars.speedRef_Hz","0");
set("motorVars.flagEnableSys","1");
p(">>> flagEnableSys=1; offset cal ...");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt(); oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0)break; }
var fu=num("motorVars.faultUse.all"), vb=getf("motorVars.VdcBus_V");
p("post-cal: offsetCalc="+oc+"  faultUse="+fu+"  VdcBus="+f(vb,2)+"  OST="+ostStr());
if(oc!==0) fail("offset cal did not complete."); if(fu!==0) fail("fault pre-spin: "+fu); if(!(vb>5)) fail("VdcBus<5.");
set("motorVars.speedRef_Hz","0"); set("IdqSet_A.value[1]",IQ);
p(">>> arm: Iq="+IQ+" A, speedRef=0 (align rotor) ...");
set("motorVars.flagRunIdentAndOnLine","1");
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(ostAllSet()) fail("OST still set after arm -- PWM not enabled.");
p("    OST="+ostStr()+" gates LIVE; aligned. >>> RAMPING forced speed up -- WATCH the motor start to spin.");
function chkf(t){ var fn=num("motorVars.faultUse.all"); if(fn!==0) fail("faultUse="+fn+" at "+t); return fn; }
p(""); p("  speedRef | faultUse | Iq_meas Id_meas | spd_Hz(FAST) | Vs_V");
var ramp=[1,2,4,6,9,12,16,20];
for(var r=0;r<ramp.length;r++){
    set("motorVars.speedRef_Hz",ramp[r]);
    s.target.runAsynch(); Thread.sleep(900); s.target.halt();
    var fn=chkf("ramp "+ramp[r]+"Hz");
    p("  "+(ramp[r]+" Hz   ").substr(0,7)+" | "+fn+"       | "+f(getf("Idq_in_A.value[1]"),2)+"   "+f(getf("Idq_in_A.value[0]"),2)+"   | "+f(getf("motorVars.speed_Hz"),1)+"   | "+f(getf("motorVars.Vs_V"),2));
}
p(">>> holding ~20 Hz (~170 rpm) for 3 s -- steady open-loop spin.");
s.target.runAsynch(); Thread.sleep(3000); s.target.halt(); chkf("hold");
var dn=[12,6,2,0];
for(var d2=0;d2<dn.length;d2++){ set("motorVars.speedRef_Hz",dn[d2]); s.target.runAsynch(); Thread.sleep(600); s.target.halt(); }
p(""); p(">>> stopping: Iq->0, disarm, OST.");
safeOff();
p("    final OST="+ostStr()+"  faultUse="+num("motorVars.faultUse.all"));
p(">>> is04 ramp done. Judge: did the motor ramp up to a smooth ~170 rpm spin?");
p("================================================================");
try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate();
