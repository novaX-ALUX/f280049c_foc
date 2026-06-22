/*
 * diag_oc_latch.js - classify the launchxl moduleOverCurrent fault on the running product main.
 *
 * Usage: dss.sh tools/flash/diag_oc_latch.js <ccxml> <product.out>
 *
 * Loads + runs the product main through the first offset cal (bus-cap inrush), then forces a
 * SECOND offset cal with the bus already charged and zero motor current, and samples the
 * over-current bit WHILE that second cal's 50%-duty PWM is running. This is the decisive test:
 *   - moduleOverCurrent == 1 mid-(2nd)-cal -> the CMPSS comparator trips with a SETTLED bus and
 *       zero current => the CMPSS positive input is misconfigured (e.g. value=4 selecting a
 *       disabled-PGA output). A software one-shot clear is INSUFFICIENT; fix HAL_setupCMPSSs.
 *   - moduleOverCurrent == 0 mid-(2nd)-cal -> the comparator reads the real (zero) current;
 *       no CMPSS routing problem (and the first trip, if any, was only a power-up inrush latch).
 * History on this board: PRE-fix it read 1 mid-cal (CMPSS5/3/1 + value=4 floated on the disabled
 * PGA outputs -> active mis-trip). AFTER the CMPSS3/1/6 routing fix it reads 0 mid-cal.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function ev(s,e){ try { return e.evaluate(s)+""; } catch(err){ return "??"; } }
function set(s,v,e){ try { e.evaluate(s+"="+v); } catch(err){ p("set fail "+s); } }

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
s.target.runAsynch(); Thread.sleep(3500); s.target.halt();   // 1st cal done (inrush)

p("");
p("=========== RE-CAL (settled bus, 0 current) OC TEST ===========");
p("post 1st cal: moduleOverCurrent=" + ev("motorVars.faultNow.bit.moduleOverCurrent",e) +
  "  faultUse.all=" + ev("motorVars.faultUse.all",e));
// force a SECOND offset cal: bus already charged -> no inrush, zero current, PWM 50% on.
set("motorVars.faultNow.all","0",e);
set("offsetCalcCount","0",e);
set("motorVars.flagEnableOffsetCalc","1",e);
p(">>> forced 2nd cal; sampling OC mid-cal (settled bus, 0 A) ...");
for (var i=1;i<=3;i++){
  s.target.runAsynch(); Thread.sleep(700); s.target.halt();
  p("  t=" + (i*0.7).toFixed(1) + "s  offsetCalc=" + ev("motorVars.flagEnableOffsetCalc",e) +
    "  moduleOverCurrent=" + ev("motorVars.faultNow.bit.moduleOverCurrent",e));
}
p("=> 1 mid-cal -> CMPSS input misconfigured (fix HAL_setupCMPSSs); 0 -> inrush latch only");
p("===============================================================");
s.target.runAsynch(); s.target.disconnect(); server.stop(); s.terminate();
