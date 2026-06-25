/* Is the integer-quantization a DSS float-read artifact or real firmware? Write known non-integer
 * floats to a variable (target halted, no ISR to overwrite) and read them back via BOTH the DSS
 * expression evaluator AND a raw 2-word IEEE754 reconstruction. If expression readback is quantized
 * but the raw reconstruction is exact, DSS's expression float read is lossy -> use raw reads. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2,e2){ try { e2.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
// read a C28x float32 (2x16-bit words, low word first) at data address a, exact IEEE754.
function rdf(s2,a){
    var w=s2.memory.readData(Memory.Page.DATA, a, 16, 2, false);
    var bits=((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF);
    return java.lang.Float.intBitsToFloat(bits);
}
function addr(s2,e2){ return Number(e2.evaluate("&adcData.dcBus_V")); }

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(60000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
s.target.halt();

var A = addr(s,e);
p("");
p("====== float read precision: DSS expression vs raw 2-word IEEE754 ======");
p("&adcData.dcBus_V = " + A);
var tests = ["3.14159", "16.0306", "-23.5700", "0.0125", "44.2999"];
for(var i=0;i<tests.length;i++){
    set("adcData.dcBus_V", tests[i], e);
    var viaExpr = num("adcData.dcBus_V", e);
    var viaRaw  = rdf(s, A);
    p("  wrote " + tests[i] + "  -> expr=" + viaExpr.toFixed(5) + "   raw=" + viaRaw.toFixed(5));
}
p("=> expr quantized + raw exact => DSS expr float read is lossy; read floats via raw words.");
p("=======================================================================");
s.target.disconnect(); server.stop(); s.terminate();
