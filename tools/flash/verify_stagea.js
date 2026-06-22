/* Stage-A liveness verify: connect, load, run, halt, read product variables. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

function p(s){ System.out.println(s); }
function ev(s, e){ try { return e.evaluate(s); } catch(err){ return "??("+s+")"; } }

var ccxml = arguments[0];
var out   = arguments[1];

var env = ScriptingEnvironment.instance();
env.setScriptTimeout(60000);
var server = env.getServer("DebugServer.1");
server.setConfig(ccxml);
var session = server.openSession("*", "C28xx_CPU1");

p(">>> connecting C28xx_CPU1 ...");
session.target.connect();
p(">>> loading "+out);
session.memory.loadProgram(out);

var e = session.expression;
p(">>> running 3.5s (offset cal + ticks) ...");
session.target.runAsynch();
Thread.sleep(3500);
session.target.halt();
var t1 = ev("g_now_ms", e);
p(">>> resume 1.0s to confirm tick increments ...");
session.target.runAsynch();
Thread.sleep(1000);
session.target.halt();
var t2 = ev("g_now_ms", e);

p("");
p("================ STAGE A READOUT ================");
p("g_now_ms (t1 / t2)        : " + t1 + " / " + t2 + "   (delta ~1000 expected)");
p("flagEnableSys             : " + ev("motorVars.flagEnableSys", e) + "   (expect 1)");
p("flagEnableOffsetCalc      : " + ev("motorVars.flagEnableOffsetCalc", e) + "   (expect 0 after cal)");
p("flagRunIdentAndOnLine     : " + ev("motorVars.flagRunIdentAndOnLine", e) + "   (expect 0, not armed)");
p("faultUse.all              : " + ev("motorVars.faultUse.all", e) + "   (expect 0)");
p("VdcBus_V                  : " + ev("motorVars.VdcBus_V", e) + "   (expect ~24)");
p("g_dn.node_id              : " + ev("g_dn.node_id", e) + "   (expect 25, static)");
p("g_dn.armed                : " + ev("g_dn.armed", e) + "   (expect 0/false)");
p("================================================");

p(">>> leaving target running, disconnecting ...");
session.target.runAsynch();
session.target.disconnect();
server.stop();
session.terminate();
