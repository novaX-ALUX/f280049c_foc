/* run_is07_esc6288.js - is07 SPEED control on esc6288. Caps the speed-PI Iq output via
 * pi_spd.outMax/outMin and aborts if Iq pins at the cap while speed fails to approach speedRef.
 *
 * WIP / DOES NOT SELF-START YET (2026-06-30): arming the speed loop directly from standstill does NOT
 * cold-start the 0.012 V/Hz 4116 -- FAST cannot lock without rotor motion, speed_Hz oscillates and the
 * speed PI pins Iq at the cap. Proper sensorless self-start needs an explicit Id ALIGN + open-loop I/f
 * RAMP before closing the loop -- see "Sensorless cold-start" in PORT_TODO.md and the launchxl
 * I/f-experiment scripts (tools/flash/drv8305evm/run_if_rampB.js). For a confirmed esc6288 spin, use
 * run_is06_esc6288.js with the hand-flick bootstrap.
 * Usage: dss.sh run_is07_esc6288.js <ccxml> <is07.out> [spdRef_Hz=30] [iqMax_A=4] [accelStart=5] */
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
var SPD=(arguments.length>2)?Number(arguments[2]):30.0;
var IQMAX=(arguments.length>3)?Number(arguments[3]):4.0;
var ACCS=(arguments.length>4)?Number(arguments[4]):5.0;
if(isNaN(SPD)||Math.abs(SPD)>120||isNaN(IQMAX)||IQMAX<=0||IQMAX>5.0){ p("FATAL: args (|spd|<=120, 0<iqMax<=5)."); java.lang.System.exit(1); }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(180000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function capIq(){ set("pi_spd.outMax",IQMAX); set("pi_spd.outMin",-IQMAX); }
function safeOff(){ set("motorVars.speedRef_Hz","0"); set("motorVars.flagRunIdentAndOnLine","0");
    s.target.runAsynch(); Thread.sleep(300); s.target.halt(); forceOST(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!! is07 ABORTED: "+why); safeOff();
    p("  -> OST forced ("+ostStr()+"), disarmed."); try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }
p(""); p("======== esc6288 is07 SPEED self-start (target="+SPD+" Hz, Iq cap +-"+IQMAX+" A, accelStart="+ACCS+") ========");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
if(num("motorVars.flagEnableSys")!==0) fail("not parked at enable-sys wait.");
if(!ostAllSet()) fail("OST not set at startup.");
set("motorVars.speedRef_Hz","0"); set("motorVars.flagEnableSys","1");
p(">>> flagEnableSys=1; offset cal ...");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt(); oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0)break; }
var fu=num("motorVars.faultUse.all"), vb=getf("motorVars.VdcBus_V");
p("post-cal: offsetCalc="+oc+"  faultUse="+fu+"  VdcBus="+f(vb,2)+"  OST="+ostStr());
if(oc!==0) fail("offset cal did not complete."); if(fu!==0) fail("fault pre-spin: "+fu); if(!(vb>5)) fail("VdcBus<5.");
set("motorVars.flagEnableForceAngle","1");
set("motorVars.accelerationStart_Hzps",ACCS); set("motorVars.accelerationMax_Hzps",Math.max(ACCS,15));
capIq();
p("self-start cfg: forceAngle="+num("motorVars.flagEnableForceAngle")+"  accelStart="+f(getf("motorVars.accelerationStart_Hzps"),1)+"  pi_spd.outMax="+f(getf("pi_spd.outMax"),2)+" (CAP CHECK -- must be "+IQMAX+")");
if(Math.abs(getf("pi_spd.outMax")-IQMAX)>0.1) fail("Iq cap did NOT take (pi_spd.outMax!="+IQMAX+") -- refuse to arm uncapped.");
set("motorVars.speedRef_Hz",SPD);
p(">>> flagRunIdentAndOnLine=1; speedRef="+SPD+" Hz. Force-angle self-start (NO flick).");
set("motorVars.flagRunIdentAndOnLine","1"); capIq();
s.target.runAsynch(); Thread.sleep(300); s.target.halt();
if(ostAllSet()) fail("OST still set after arm -- PWM not enabled.");
p("    OST="+ostStr()+" gates LIVE. >>> WATCH the rotor self-start and ramp up.");
p("");
p("  t(s) | faultUse | speedRef speed_Hz | Iq_meas(cap "+IQMAX+") | Vs_V VdcBus");
for(var i=0;i<14;i++){
    s.target.runAsynch(); Thread.sleep(1000); s.target.halt(); capIq();
    var fn=num("motorVars.faultUse.all");
    var sr=getf("motorVars.speedRef_Hz"), sp=getf("motorVars.speed_Hz"), iqm=getf("Idq_in_A.value[1]"), vs=getf("motorVars.Vs_V"), vbn=getf("motorVars.VdcBus_V");
    p("  "+(i+1)+"  | "+fn+"       | "+f(sr,1)+"    "+f(sp,1)+"   | "+f(iqm,2)+"   | "+f(vs,2)+" "+f(vbn,2));
    if(fn!==0) fail("faultUse="+fn+" during spin-up (t="+(i+1)+"s).");
    // runaway/stall abort: Iq pinned near cap AND speed not approaching speedRef
    if(i>=4 && Math.abs(iqm)>(IQMAX*0.8) && Math.abs(sp-sr)>15) fail("NOT CONVERGING: Iq pinned ~"+f(iqm,1)+"A, speed "+f(sp,1)+" not approaching "+f(sr,1)+" after "+(i+1)+"s -- force-commutation not catching.");
}
var spF=getf("motorVars.speed_Hz");
p(">>> speed_Hz="+f(spF,1)+" (target "+SPD+")  "+(Math.abs(spF-SPD)<8?"== LOCKED ==":"-- not at target --"));
p(">>> ramping speedRef down to 0 ...");
var dn=[Math.round(SPD*0.5),Math.round(SPD*0.2),0];
for(var d2=0;d2<dn.length;d2++){ set("motorVars.speedRef_Hz",dn[d2]); s.target.runAsynch(); Thread.sleep(1500); s.target.halt(); }
p(""); p(">>> stopping."); safeOff();
p("    final OST="+ostStr()+"  faultUse="+num("motorVars.faultUse.all"));
p("================================================================");
try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate();
