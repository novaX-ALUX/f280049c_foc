/* Liveness test: poll raw bus ADC count + firmware adcData.dcBus_V while the user sweeps the PSU.
 * If adcData.dcBus_V tracks the raw count (and the metered bus), the firmware analog read path is
 * LIVE -> the perfectly-round zero-current values were just phase-locked stability, front end OK.
 * If adcData.dcBus_V stays pinned while the raw count moves, the read path is frozen -> real bug. */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2,e2){ try { e2.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function rd(s2,a){ return s2.memory.readData(Memory.Page.DATA, a, 16, 1, false)[0] & 0xFFFF; }
// DSS expression.evaluate() truncates float32 to integer; read floats exactly via &expr + 2-word
// IEEE754 (else this liveness proof itself shows integer-quantized bus / a collapsed adcData.I_A).
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

// EN_GATE + flagEnableSys; leave flagEnableOffsetCalibration default(1) so cal re-arms and PWM
// (hence EPWM6 SOCA / ADC triggering) stays on for the whole sweep window.
s.target.runAsynch(); Thread.sleep(1000); s.target.halt();
s.memory.writeData(Memory.Page.DATA, 0x7F0A, 0x80, 16);
set("motorVars.flagEnableSys", "1", e);

p("");
p("====== BUS SWEEP liveness (sweep PSU 25V -> ~15V -> 25V now) ======");
p("  t(s) | rawBus(0x0B22) | adcData.dcBus_V  motorVars.VdcBus_V | adcData.I_A[0]");
for(var i=0;i<40;i++){
    s.target.runAsynch(); Thread.sleep(500); s.target.halt();
    var rb=rd(s,0x0B22);
    var dv=getf("adcData.dcBus_V"), vv=getf("motorVars.VdcBus_V"), i0=getf("adcData.I_A.value[0]");
    p("  "+(i*0.5).toFixed(1)+" | "+rb+" | "+dv.toFixed(4)+"  "+vv.toFixed(4)+" | "+i0.toFixed(4));
}
p("=> dcBus_V tracks rawBus => read path LIVE; dcBus_V pinned while rawBus moves => frozen.");
p("==================================================================");
s.memory.writeData(Memory.Page.DATA, 0x7F0C, 0x80, 16);
set("motorVars.flagEnableSys", "0", e);
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
