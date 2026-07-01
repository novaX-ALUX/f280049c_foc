/* run_iftest_esc6288.js - esc6288 OPEN-LOOP I/f startup test on is04 (signal_chain_test).
 * is04's FOC runs on the open-loop ANGLE_GEN angle (ramps at speedTraj_Hz), current = IdqSet, FAST
 * estimator runs in parallel. Sequence: ALIGN (speedRef=0, Id held -> rotor locks to angleFoc=0) ->
 * I/f RAMP (ramp speedRef at accel, angle accelerates, rotor follows) -> read whether FAST tracks
 * (motorVars.speed_Hz ~= speedTraj) => handoff-ready. Current-limited to IdqSet (safe). No prop.
 * Usage: dss.sh run_iftest_esc6288.js <ccxml> <is04.out> [id_A=3] [accel_Hzps=5] [target_Hz=50] [align_s=2] */
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
var ccxml=arguments[0], out=arguments[1];
var ID=(arguments.length>2)?Number(arguments[2]):3.0;
var ACC=(arguments.length>3)?Number(arguments[3]):5.0;
var TGT=(arguments.length>4)?Number(arguments[4]):50.0;
var ALIGN=(arguments.length>5)?Number(arguments[5]):2.0;
if(ID<=0||ID>5) { p("FATAL id out of (0,5]"); java.lang.System.exit(1); }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function safeOff(){ set("IdqSet_A.value[0]","0"); set("IdqSet_A.value[1]","0"); set("motorVars.speedRef_Hz","0");
    set("motorVars.flagRunIdentAndOnLine","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt(); forceOST(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!! I/f TEST ABORTED: "+why); safeOff(); p("  -> OST "+ostStr()+", disarmed.");
    try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }
function rdst(tag){ var sp=getf("motorVars.speed_Hz"), st=getf("motorVars.speedTraj_Hz");
    var af=getf("angleFoc_rad"), ae=getf("angleEst_rad");
    var idm=getf("Idq_in_A.value[0]"), iqm=getf("Idq_in_A.value[1]"), fn=num("motorVars.faultUse.all");
    p("  ["+tag+"] speedTraj(cmd)="+f(st,1)+" FAST_speed="+f(sp,1)+" | angFoc="+f(af,2)+" angEst="+f(ae,2)+
      " | Id="+f(idm,2)+" Iq="+f(iqm,2)+" | fault="+fn);
    if(fn!==0) fail("faultUse="+fn+" ("+tag+")"); return {sp:sp,st:st,idm:idm}; }
p(""); p("======== esc6288 OPEN-LOOP I/f startup test (is04): Id="+ID+"A ramp "+ACC+"Hz/s -> "+TGT+"Hz ========");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
if(num("motorVars.flagEnableSys")!==0) fail("not parked at enable-sys wait.");
if(!ostAllSet()) fail("OST not set.");
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt(); oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0)break; }
var vb=getf("motorVars.VdcBus_V");
p("post-cal: offsetCalc="+oc+" faultUse="+num("motorVars.faultUse.all")+" VdcBus="+f(vb,2)+"V OST="+ostStr());
if(oc!==0) fail("offset cal incomplete."); if(num("motorVars.faultUse.all")!==0) fail("fault pre-run.");
// ALIGN: fixed open-loop angle, hold Id
set("motorVars.speedRef_Hz","0"); set("motorVars.accelerationMax_Hzps","0");
set("IdqSet_A.value[0]", ID); set("IdqSet_A.value[1]","0");
set("motorVars.flagRunIdentAndOnLine","1");
s.target.runAsynch(); Thread.sleep(ALIGN*1000); s.target.halt();
if(ostAllSet()) fail("OST still set after arm -- PWM not enabled.");
p(">>> ALIGN done ("+ALIGN+"s). OST="+ostStr()+" (gates live):");
var al=rdst("align"); 
if(Math.abs(al.idm)<ID*0.5) fail("align current NOT flowing (Id_meas="+f(al.idm,2)+" << "+ID+") -- is04 current path issue.");
p("    -> align current IS flowing (Id_meas="+f(al.idm,2)+"A). Rotor locked. Now RAMP (continuous):");
// I/f RAMP: continuous, no mid halts (halting freezes the open-loop angle -> rotor slip)
set("motorVars.accelerationMax_Hzps", ACC); set("motorVars.speedRef_Hz", TGT);
var rampS = TGT/ACC + 3.0;   // ramp time + settle
s.target.runAsynch(); Thread.sleep(rampS*1000); s.target.halt();
p(">>> after continuous ramp+settle ("+f(rampS,1)+"s):");
var r=rdst("end");
p("");
var lockErr = Math.abs(r.sp - r.st);
if(r.st>TGT*0.9 && lockErr < TGT*0.2 && r.sp>TGT*0.5){
    p(">>> *** OPEN-LOOP I/f SPUN UP + FAST TRACKING (FAST_speed "+f(r.sp,1)+" ~= cmd "+f(r.st,1)+") -> HANDOFF-READY ***");
} else if(r.st>TGT*0.9){
    p(">>> open-loop reached cmd "+f(r.st,1)+"Hz but FAST_speed="+f(r.sp,1)+" (err "+f(lockErr,1)+"): rotor may be spinning open-loop but FAST not yet locked, OR rotor slipped. Watch the motor + tune (slower accel / more Id).");
} else {
    p(">>> did not reach target. Check for stall/slip.");
}
p(""); p(">>> stopping (safe-off)."); safeOff();
p("    final OST="+ostStr()+" faultUse="+num("motorVars.faultUse.all"));
p("================================================================");
try{s.target.halt();s.target.disconnect();}catch(x){} server.stop(); s.terminate();
