/* Stage 1 I-f characterization: graded open-loop frequency plateaus at a fixed drag current,
 * measuring whether FAST (running in parallel for diagnostics in is04) converges to the commanded
 * electrical frequency at each plateau. This finds the empirical handoff window for I-f -> FAST.
 *
 * is04 is itself an I-f bench: ANGLE_GEN drives the open-loop electrical angle at speedTraj_Hz,
 * Idq_ref_A = IdqSet_A closes the current loop on that angle, and EST_run() runs in parallel WITHOUT
 * feeding the FOC angle -- so estOutputData.fm_lp_rps tells us "what FAST would estimate here".
 *
 * UNITS (verified against is05:921 / is07:913 / labs.h:962):
 *   speedRef_Hz / speedTraj_Hz are ELECTRICAL Hz. FAST feedback is fm_lp_rps * (1/2pi), compared
 *   DIRECTLY to speed_ref_Hz. So:  fm_lp_Hz(elec) = fm_lp_rps / (2*pi)  -- do NOT multiply by p.
 *   p is used ONLY for display:    mech_rpm = elec_Hz * 60 / p.
 *   (The earlier run_draghi.js used fm*7/(2pi) -- that was WRONG, inflated 7x.)
 *
 * METHOD: for each plateau freq, run K COMPLETE short runs (align dwell -> ramp -> plateau -> single
 * halt read). NOT halt/resume on one plateau (halt perturbs the loop). Default plateaus 5/8/10/12/16.
 * 20/25 Hz are NOT default -- pass them explicitly as a guarded opt-in only after low freqs are stable
 * (BENCH_NOTES: 20 Hz open-loop tends to slip).
 *
 * Usage: dss.sh tools/flash/drv8305evm/run_if_char.js <ccxml> <is04.out> \
 *          [id_A=3.0] [freqs="5,8,10,12,16"] [K=3] [accel_Hzps=8] [dwell_s=2] [plateau_s=4] [poles=7]
 * Safety: current-limited supply, no prop, clamped. Per-run abort on overcurrent / slip / fault.
 *   Note: software abort is a BETWEEN-run check; real-time over-current protection is hardware CMPSS.
 *   Keep plateaus short and limits low; ramp low freqs first. */
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
var ID    = (arguments.length>2)?Number(arguments[2]):3.0;
var FREQS = (arguments.length>3)?(""+arguments[3]).split(","):["5","8","10","12","16"];
var K     = (arguments.length>4)?Number(arguments[4]):3;
var ACCEL = (arguments.length>5)?Number(arguments[5]):8.0;
var DWELL = (arguments.length>6)?Number(arguments[6]):2.0;
var PLAT  = (arguments.length>7)?Number(arguments[7]):4.0;
var POLES = (arguments.length>8)?Number(arguments[8]):7;
var IMAX=6.0, IQMAX=4.0;                 // between-run abort thresholds
var TOL=0.20;                            // |fm_lp_Hz - cmd|/cmd within this == "converged"
if(isNaN(ID)||ID<=0||ID>4.0){ p("FATAL: id out of (0,4]"); java.lang.System.exit(1); }
if(isNaN(K)||K<1){ K=1; }

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(600000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function gateLow(){ try { s.memory.writeData(Memory.Page.DATA,0x7F0C,0x80,16); } catch(x){} }
function stopClean(){ set("IdqSet_A.value[0]","0"); set("IdqSet_A.value[1]","0");
    set("motorVars.flagRunIdentAndOnLine","0"); s.target.runAsynch(); Thread.sleep(300); s.target.halt();
    gateLow(); set("motorVars.flagEnableSys","0"); }
function fail(why){ p(""); p("!!!! ABORT: "+why); stopClean();
    try{s.target.disconnect();}catch(x){} server.stop(); s.terminate(); java.lang.System.exit(1); }

p(""); p("==== I-f characterization (Stage 1): Id="+ID+"A, K="+K+"/freq, accel="+ACCEL+"Hz/s ====");
p("     plateaus(elec Hz): "+FREQS.join(", ")+"   [20/25 only as explicit guarded opt-in]");

// boot + EN_GATE + offset cal (rotor still -> halt-poll harmless here)
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) fail("halHandle invalid.");
s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);              // EN_GATE high (GPIO39)
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var g39=((s.memory.readData(Memory.Page.DATA,0x7F08,16,1,false)[0]&0x80)!=0)?1:0;
if(g39!==1) fail("EN_GATE readback low -- gate not enabled.");
set("motorVars.flagEnableSys","1");
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc"); if(oc===0) break; }
if(oc!==0) fail("offset cal incomplete."); if(num("motorVars.faultUse.all")!==0) fail("fault before run.");

// pole-pair cross-check (display only; primary criterion never uses p)
var pTgt=num("userParams.motor_numPolePairs");
if(!isNaN(pTgt) && pTgt>0){ if(pTgt!==POLES){ p("  [note] target userParams.motor_numPolePairs="+pTgt
    +" != arg "+POLES+"; using target for rpm display."); POLES=pTgt; } }
else { p("  [note] could not read target pole pairs; using arg p="+POLES+" for rpm display only."); }
p("");

var TWO_PI=2.0*Math.PI;
p(" freq | run |  Id_in   Iq_in | Iabc peak (A)        | Vs_V  | fm_lp_Hz(elec) err%  ~mech_rpm | flt");
p(" -----+-----+----------------+---------------------+-------+--------------------------------+----");

for(var fi=0; fi<FREQS.length; fi++){
    var cmd=Number(FREQS[fi]);
    if(isNaN(cmd)||cmd<=0){ p("  (skip bad freq '"+FREQS[fi]+"')"); continue; }
    var runSec = cmd/ACCEL + PLAT;          // ramp time + plateau hold, then read at plateau
    var conv=0;
    for(var ki=0; ki<K; ki++){
        // (1) align dwell: hold open-loop d-axis at Id, speedRef=0
        set("motorVars.accelerationMax_Hzps","0"); set("motorVars.speedRef_Hz","0");
        set("IdqSet_A.value[0]", ID); set("IdqSet_A.value[1]","0");
        set("motorVars.flagRunIdentAndOnLine","1");
        s.target.runAsynch(); Thread.sleep(DWELL*1000); s.target.halt();
        if(num("motorVars.faultUse.all")!==0) fail("fault during align (freq "+cmd+", run "+ki+").");
        // (2) ramp to plateau + (3) hold -- ONE uninterrupted free-run, NO mid-run halt
        set("motorVars.accelerationMax_Hzps", ACCEL); set("motorVars.speedRef_Hz", cmd);
        s.target.runAsynch(); Thread.sleep(runSec*1000); s.target.halt();
        // (4) single read at end of plateau
        var idm=getf("Idq_in_A.value[0]"), iqm=getf("Idq_in_A.value[1]");
        var ia=getf("adcData.I_A.value[0]"), ib=getf("adcData.I_A.value[1]"), ic=getf("adcData.I_A.value[2]");
        var vs=getf("motorVars.Vs_V"), fm=getf("estOutputData.fm_lp_rps");
        var fn=num("motorVars.faultUse.all");
        var feHz=fm/TWO_PI;                 // ELECTRICAL Hz (NOT *p)
        var rpm=feHz*60.0/POLES;            // mechanical rpm (p used only here)
        var ipk=Math.max(Math.abs(ia),Math.max(Math.abs(ib),Math.abs(ic)));
        var err=(cmd>0)?((feHz-cmd)/cmd*100.0):NaN;
        var okc=(!isNaN(err)&&Math.abs(err)<=TOL*100.0); if(okc) conv++;
        p(f(cmd,0)+"Hz | "+(ki+1)+"/"+K+" | "+f(idm,2)+" "+f(iqm,2)+" | "
          +f(ia,2)+" "+f(ib,2)+" "+f(ic,2)+"  | "+f(vs,2)+" | "
          +f(feHz,1)+"  "+f(err,0)+"%  "+f(rpm,0)+"rpm "+(okc?"<conv>":"      ")+" | "+fn);
        // (between-run safety; real-time guarding is hardware CMPSS)
        if(fn!==0) fail("fault latched (freq "+cmd+", run "+(ki+1)+").");
        if(ipk>IMAX && ipk<20) fail("phase peak "+f(ipk,1)+"A > "+IMAX+"A (freq "+cmd+").");
        if(Math.abs(iqm)>IQMAX) fail("Iq_in "+f(iqm,1)+"A > "+IQMAX+"A -> slipping (freq "+cmd+").");
    }
    p("      => "+cmd+"Hz: "+conv+"/"+K+" runs converged (|err|<="+(TOL*100)+"%)");
}

p("");
p(">>> SUMMARY: a contiguous band where most runs show <conv> is the empirical I-f->FAST handoff window.");
p(">>> If 'stable drag' and 'FAST converged' have NO overlap -> stop at Stage 1, record + propose fixes.");
stopClean();
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
