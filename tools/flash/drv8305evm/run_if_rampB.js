/* Stage 1 experiment B: ONE continuous gentle open-loop ramp (no per-step re-align) from standstill
 * up to a high target electrical frequency, with the 5-channel in-ISR recorder logging the whole
 * trajectory. Answers the HIGH-side question the 5-13Hz scan could not: is there ANY frequency where
 * FAST fm_lp starts tracking the commanded open-loop freq BEFORE the open-loop drag slips? i.e. does
 * an I-f -> FAST handoff window exist at all on this bench, just at a higher speed.
 *
 * Requires the is04 BENCH-FORK recorder (g_rec_arm/idx/decim + g_rec_fm/id/iq/cmd/ipk[]).
 * UNITS: fm_lp_Hz(elec) = fm_lp_rps / (2*pi), directly comparable to the live commanded freq g_rec_cmd.
 *
 * Usage: dss.sh tools/flash/drv8305evm/run_if_rampB.js <ccxml> <is04.out> \
 *          [target_Hz=35] [id_A=3.0] [accel_Hzps=2] [decim=1500] [dwell_s=3] [slipIq_A=4] [poles=7] [isr_Hz=20000]
 *   REC_N=256 @ decim=1500 / 20kHz ISR -> 75ms/sample -> ~19s capture; accel=2 reaches 35Hz in ~17.5s.
 * Real-time over-current protection is hardware CMPSS (it latches faultUse bit4 and the bg loop disables
 * the drive -> recorder then logs zeros, marking exactly where the drag died). Post-run fault classify. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2){ try { return Number(e.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2){ try { e.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm)); var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }
function readArr(nm,n){ var base=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA, base, 16, n*2, false); var a=[];
    for(var i=0;i<n;i++){ a.push(java.lang.Float.intBitsToFloat(((w[2*i+1]&0xFFFF)<<16)|(w[2*i]&0xFFFF))); } return a; }
function f(x,n){ return (isNaN(x)?"nan":(x>=0?" ":"")+x.toFixed(n)); }

var ccxml=arguments[0], out=arguments[1];
var TGT   = (arguments.length>2)?Number(arguments[2]):35.0;
var ID    = (arguments.length>3)?Number(arguments[3]):3.0;
var ACCEL = (arguments.length>4)?Number(arguments[4]):2.0;
var DECIM = (arguments.length>5)?Number(arguments[5]):1500;
var DWELL = (arguments.length>6)?Number(arguments[6]):3.0;
var SLIPIQ= (arguments.length>7)?Number(arguments[7]):4.0;
var POLES = (arguments.length>8)?Number(arguments[8]):7;
var ISRHZ = (arguments.length>9)?Number(arguments[9]):20000.0;
var TOL=0.20, TWO_PI=2.0*Math.PI;
if(isNaN(ID)||ID<=0||ID>4.0){ p("FATAL: id out of (0,4]"); java.lang.System.exit(1); }

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(300000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function gateLow(){ try { s.memory.writeData(Memory.Page.DATA,0x7F0C,0x80,16); } catch(x){} }
function stopClean(){ set("g_rec_arm","0"); set("IdqSet_A.value[0]","0"); set("IdqSet_A.value[1]","0");
    set("motorVars.flagRunIdentAndOnLine","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!!! ABORT: "+why); stopClean();
    try{s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }

s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
var REC_N=num("IF_REC_N"); if(isNaN(REC_N)||REC_N<=0){ REC_N=256; }
var Ts=DECIM/ISRHZ, capSec=REC_N*Ts;
p(""); p("==== I-f experiment B: continuous ramp 0 -> "+TGT+"Hz_elec @ "+ACCEL+"Hz/s, Id="+ID+"A ====");
p("     "+REC_N+" samples @ "+f(1.0/Ts,1)+"Hz = "+f(capSec,1)+"s capture (decim="+DECIM+", ISR="+ISRHZ+")");

s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var g39=((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0;
if(g39!==1) fail("EN_GATE readback low.");
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
if(oc!==0) fail("offset cal incomplete."); if(num("motorVars.faultUse.all")!==0) fail("fault before run.");

// align dwell (recorder off), then arm + continuous ramp (one free-run)
set("g_rec_arm","0"); set("g_rec_idx","0"); set("g_rec_decim", DECIM);
set("motorVars.accelerationMax_Hzps","0"); set("motorVars.speedRef_Hz","0");
set("IdqSet_A.value[0]", ID); set("IdqSet_A.value[1]","0"); set("motorVars.flagRunIdentAndOnLine","1");
s.target.runAsynch(); Thread.sleep(DWELL*1000); s.target.halt();
if(num("motorVars.faultUse.all")!==0) fail("fault during align.");

set("g_rec_idx","0"); set("g_rec_arm","1");
set("motorVars.accelerationMax_Hzps", ACCEL); set("motorVars.speedRef_Hz", TGT);
p(">>> RAMPING "+f(capSec+0.5,1)+"s, FREE (no halts). CMPSS is the real-time OC guard.");
s.target.runAsynch(); Thread.sleep((capSec+0.5)*1000); s.target.halt();

var fn=num("motorVars.faultUse.all");
var nGot=num("g_rec_idx"); if(isNaN(nGot)||nGot<1){ fail("recorder captured nothing."); }
var fm=readArr("g_rec_fm",nGot), idv=readArr("g_rec_id",nGot), iqv=readArr("g_rec_iq",nGot),
    cmd=readArr("g_rec_cmd",nGot), ipk=readArr("g_rec_ipk",nGot);

p(""); p("  t(s)  cmdHz  fm_Hz  err%  | Id    Iq    ipk | lock");
p("  ------------------------------------+----------------+-----");
var bestLen=0,bestS=-1,curLen=0,curS=-1;
for(var i=0;i<nGot;i++){
    var c=cmd[i], feHz=fm[i]/TWO_PI, err=(c>0.5)?((feHz-c)/c*100.0):NaN;
    var lock=(c>3.0)&&(!isNaN(err))&&(Math.abs(err)<=TOL*100.0)&&(Math.abs(iqv[i])<=SLIPIQ);
    if(lock){ if(curS<0){curS=i;curLen=0;} curLen++; if(curLen>bestLen){bestLen=curLen;bestS=curS;} }
    else { curS=-1; curLen=0; }
    p("  "+f(i*Ts,2)+"  "+f(c,1)+"  "+f(feHz,1)+"  "+f(err,0)+"% | "
      +f(idv[i],2)+" "+f(iqv[i],2)+" "+f(ipk[i],2)+" | "+(lock?"LOCK":""));
}
p("");
var MIN_LOCK = Math.max(8, Math.ceil(1.0/Ts));   // a real window must be SUSTAINED (>=1s and >=8 samples)
var dt=bestLen*Ts, fa=(bestS>=0)?cmd[bestS]:NaN, fb=(bestS>=0)?cmd[bestS+bestLen-1]:NaN;
p(">>> longest contiguous lock = "+bestLen+" samples ("+f(dt,2)+"s)"
  +((bestS>=0)?(", cmd "+f(fa,1)+" -> "+f(fb,1)+"Hz"):"")+"; sustained threshold = "+MIN_LOCK+" samples (1s).");
if(bestLen>=MIN_LOCK){
    p(">>> SUSTAINED LOCK BAND found ("+f(fa,1)+"-"+f(fb,1)+"Hz, Iq synced) == I-f->FAST HANDOFF WINDOW. Stage 2 viable.");
} else {
    p(">>> NO SUSTAINED lock band (longest is just a transient coincidence). fm_lp never held cmd within");
    p(">>> +-"+(TOL*100)+"% with Iq synced for >=1s. If drag slipped (Iq blew up / ipk huge) before fm_lp ever");
    p(">>> locked -> NO standard BEMF handoff window on this bench.");
}
p("    final: fault="+fn+(fn!==0?" (bit4=0x10 HW over-current)":"")
  +"  Vs="+f(getf("motorVars.Vs_V"),2)+"  cmdTraj="+f(getf("motorVars.speedTraj_Hz"),2)+"Hz");
stopClean();
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
