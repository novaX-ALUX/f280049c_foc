/* Stage 1.5 I-f recorder readout: arm the in-ISR ring buffer (BENCH-FORK in is04), drive ONE
 * open-loop frequency plateau, then read back the time-series of fm_lp / Id_in / Iq_in WITHOUT any
 * mid-run halt. This answers the question the single-point run_if_char.js could not: does the FAST
 * speed estimate SETTLE near the commanded electrical frequency for a sustained window, or is it
 * perpetual noise? And does Id_in actually hold the drag current through the plateau?
 *
 * Requires the is04 BENCH-FORK recorder globals: g_rec_arm/g_rec_idx/g_rec_decim/g_rec_fm[]/_id[]/_iq[].
 * UNITS: fm_lp_Hz(elec) = fm_lp_rps / (2*pi)  -- directly comparable to the commanded freq (NOT *p).
 *
 * Usage: dss.sh tools/flash/drv8305evm/run_if_rec.js <ccxml> <is04.out> \
 *          [cmd_Hz=10] [id_A=3.0] [accel_Hzps=4] [decim=400] [dwell_s=3] [poles=7] [isr_Hz=20000]
 *   decim=400 @ 20kHz ISR -> 50 Hz sample rate; buffer IF_REC_N=256 -> ~5.1 s capture.
 *   NOTE: dropping USER_PWM_FREQ_kHz changes the ISR rate (ISR = PWM_kHz*1000/NUM_PWM_TICKS_PER_ISR_TICK).
 *   Pass the matching isr_Hz (e.g. 10000 for a 20 kHz-PWM build with ticks=2) so the capture window and
 *   time axis stay correct. The convergence verdict itself is sample-rate independent.
 * Safety: current-limited supply, no prop, clamped. Aborts on fault / overcurrent / slip after capture. */
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
var CMD   = (arguments.length>2)?Number(arguments[2]):10.0;
var ID    = (arguments.length>3)?Number(arguments[3]):3.0;
var ACCEL = (arguments.length>4)?Number(arguments[4]):4.0;
var DECIM = (arguments.length>5)?Number(arguments[5]):400;
var DWELL = (arguments.length>6)?Number(arguments[6]):3.0;
var POLES = (arguments.length>7)?Number(arguments[7]):7;
var ISRHZ = (arguments.length>8)?Number(arguments[8]):20000.0;   // match the build's ISR rate
var IMAX=6.0, IQMAX=4.0, TWO_PI=2.0*Math.PI;
if(isNaN(ID)||ID<=0||ID>4.0){ p("FATAL: id out of (0,4]"); java.lang.System.exit(1); }
if(isNaN(CMD)||CMD<=0){ p("FATAL: cmd_Hz must be >0"); java.lang.System.exit(1); }

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

// resolve recorder size + sample period
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
var REC_N=num("IF_REC_N"); if(isNaN(REC_N)||REC_N<=0){ REC_N=256; }   // fallback to known size
var Ts=DECIM/ISRHZ;                    // sample period (s) = decim / ISR rate
var capSec=REC_N*Ts;
p(""); p("==== I-f RECORDER: cmd="+CMD+"Hz_elec, Id="+ID+"A, accel="+ACCEL+"Hz/s ====");
p("     "+REC_N+" samples @ "+f(1.0/Ts,0)+"Hz = "+f(capSec,2)+"s capture (decim="+DECIM+")");

// EN_GATE + offset cal (rotor still -> halt-poll harmless)
s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var g39=((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0;
if(g39!==1) fail("EN_GATE readback low.");
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
if(oc!==0) fail("offset cal incomplete."); if(num("motorVars.faultUse.all")!==0) fail("fault before run.");

// (1) align dwell at open-loop d-axis
set("g_rec_arm","0"); set("g_rec_idx","0"); set("g_rec_decim", DECIM);
set("motorVars.accelerationMax_Hzps","0"); set("motorVars.speedRef_Hz","0");
set("IdqSet_A.value[0]", ID); set("IdqSet_A.value[1]","0"); set("motorVars.flagRunIdentAndOnLine","1");
s.target.runAsynch(); Thread.sleep(DWELL*1000); s.target.halt();
if(num("motorVars.faultUse.all")!==0) fail("fault during align.");

// (2) arm recorder, ramp to plateau, ONE uninterrupted free-run spanning the whole capture
set("g_rec_idx","0"); set("g_rec_arm","1");
set("motorVars.accelerationMax_Hzps", ACCEL); set("motorVars.speedRef_Hz", CMD);
p(">>> CAPTURING "+f(capSec+0.4,1)+"s (ramp 0->"+CMD+"Hz @ "+ACCEL+"Hz/s, then plateau). FREE, no halts.");
s.target.runAsynch(); Thread.sleep((capSec+0.4)*1000); s.target.halt();

var fn=num("motorVars.faultUse.all");
var nGot=num("g_rec_idx"); if(isNaN(nGot)||nGot<1){ fail("recorder captured nothing (idx="+nGot+")."); }
var fm=readArr("g_rec_fm", nGot), idv=readArr("g_rec_id", nGot), iqv=readArr("g_rec_iq", nGot);

p(""); p("  t(s) | fm_lp_Hz  err%  | Id_in  Iq_in   (cmd "+CMD+"Hz, "+nGot+" samples)");
p("  -----+----------------+----------------");
var tailFrom=Math.floor(nGot*0.6), conv=0, tailN=0, smin=1e9, smax=-1e9, ssum=0;
for(var i=0;i<nGot;i++){
    var feHz=fm[i]/TWO_PI, err=(feHz-CMD)/CMD*100.0, t=i*Ts;
    var inTail=(i>=tailFrom);
    if(inTail){ tailN++; ssum+=feHz; if(feHz<smin)smin=feHz; if(feHz>smax)smax=feHz;
        if(Math.abs(err)<=20.0) conv++; }
    // print every sample (decimated capture already keeps this readable)
    p("  "+f(t,2)+" | "+f(feHz,1)+"  "+f(err,0)+"%  | "+f(idv[i],2)+" "+f(iqv[i],2)+(inTail?" *":""));
}
var smean=(tailN>0)?ssum/tailN:NaN;
p("");
p(">>> TAIL (last 40%, marked *): fm_lp_Hz mean="+f(smean,1)+" min="+f(smin,1)+" max="+f(smax,1)
  +"  -> "+conv+"/"+tailN+" within +-20% of "+CMD+"Hz");
p(">>> READ: if tail fm_lp HOLDS near "+CMD+"Hz with small spread -> a real handoff window exists here.");
p(">>>       if tail fm_lp keeps wandering / spread is wide -> FAST not observable at this freq (no window).");
p(">>>       also check Id_in: should stay ~"+ID+"A; if it collapses/oscillates the DRAG is slipping, not FAST.");
p("    final: fault="+fn+"  Vs="+f(getf("motorVars.Vs_V"),2)+"  cmdTraj="+f(getf("motorVars.speedTraj_Hz"),2)+"Hz");

// post-capture safety classification (between-run, hardware CMPSS is the real-time guard)
if(fn!==0) fail("fault latched during capture.");
var ipk=Math.max(Math.abs(getf("adcData.I_A.value[0]")),
        Math.max(Math.abs(getf("adcData.I_A.value[1]")),Math.abs(getf("adcData.I_A.value[2]"))));
if(ipk>IMAX && ipk<20) p("  [warn] final phase peak "+f(ipk,1)+"A > "+IMAX+"A.");
stopClean();
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
