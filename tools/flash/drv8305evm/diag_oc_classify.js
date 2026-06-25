/* Classify the is06 FOC-enable over-current trip. Reproduce it, then read the LATCHED trip sources
 * (they persist post-trip):
 *   - DRV8305 nFAULT (GPIO13, active-low: 0 => DRV8305 itself flagged a fault, e.g. VDS/shoot-through)
 *   - per-phase CMPSS COMPSTS latch: high (0x2) / low (0x200) -- which comparator + which phase tripped
 *   - raw current counts (decayed post-trip, but a high value would mean real current)
 * CMPSS (J1_J2): phaseA=CMPSS3(0x5CC0) phaseB=CMPSS1(0x5C80) phaseC=CMPSS6(0x5D20); COMPSTS off 0x2.
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(s2,e2){ try { return Number(e2.evaluate(s2)); } catch(err){ return NaN; } }
function set(s2,v2,e2){ try { e2.evaluate(s2+"="+v2); return true; } catch(err){ return false; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0] & 0xFFFF; }
function getf(nm){ try { var a=Number(e.evaluate("&"+nm));
    var w=s.memory.readData(Memory.Page.DATA,a,16,2,false);
    return java.lang.Float.intBitsToFloat(((w[1]&0xFFFF)<<16)|(w[0]&0xFFFF)); } catch(err){ return NaN; } }

var ccxml=arguments[0], out=arguments[1];
var IQ = (arguments.length > 2) ? Number(arguments[2]) : 1.0;   // Iq=0 allowed (lowest-risk baseline)
if(isNaN(IQ) || IQ < 0.0 || IQ > 1.0){ p("FATAL: iq out of [0,1.0]"); java.lang.System.exit(1); }
var env=ScriptingEnvironment.instance(); env.setScriptTimeout(90000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1"); s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;
function gateLow(){ try { s.memory.writeData(Memory.Page.DATA,0x7F0C,0x80,16); } catch(x){} }

p(""); p("=========== OC trip classification ===========");
s.target.runAsynch(); Thread.sleep(1200); s.target.halt();
set("IdqSet_A.value[1]","0",e); set("IdqSet_A.value[0]","0",e);
s.memory.writeData(Memory.Page.DATA,0x7F0A,0x80,16);   // EN_GATE high
s.target.runAsynch(); Thread.sleep(50); s.target.halt();
var nf0=(rd(0x7F00)&0x2000)?1:0;
p("after EN_GATE (pre-cal): nFAULT(GPIO13)=" + nf0 + " (1=ok)  rawI A="+rd(0x0B20)+" B="+rd(0x0B40)+" C="+rd(0x0B00));
// confirm the live CMPSS DAC thresholds (DACHVALS off 0x6 / DACLVALS off 0x12): expect 3612 / 484 for +-18A
p("CMPSS DAC thresholds (live): A/CMPSS3 H="+rd(0x5CC6)+" L="+rd(0x5CD2)+
  "  B/CMPSS1 H="+rd(0x5C86)+" L="+rd(0x5C92)+"  C/CMPSS6 H="+rd(0x5D26)+" L="+rd(0x5D32));
set("motorVars.flagEnableSys","1",e);
var oc=1; for(var k=0;k<8;k++){ s.target.runAsynch(); Thread.sleep(700); s.target.halt();
    oc=num("motorVars.flagEnableOffsetCalc",e); if(oc===0) break; }
p("post-cal: faultUse="+num("motorVars.faultUse.all",e)+"  nFAULT="+((rd(0x7F00)&0x2000)?1:0)+
  "  rawI A="+rd(0x0B20)+" B="+rd(0x0B40)+" C="+rd(0x0B00));

// trigger FOC -> reproduce the trip (Iq from arg)
p(">>> FOC enable, Iq=" + IQ + " A ...");
set("IdqSet_A.value[1]", IQ, e);
set("motorVars.flagRunIdentAndOnLine","1",e);
s.target.runAsynch(); Thread.sleep(300); s.target.halt();

var nf=(rd(0x7F00)&0x2000)?1:0;
p("");
p("post-FOC-enable:");
p("  faultUse.all      = " + num("motorVars.faultUse.all",e) + " (16=moduleOverCurrent)");
p("  Vs_V              = " + getf("motorVars.Vs_V").toFixed(3) + "   Idq_in d/q = " +
  getf("Idq_in_A.value[0]").toFixed(3) + " / " + getf("Idq_in_A.value[1]").toFixed(3) + " A");
p("  nFAULT (GPIO13)   = " + nf + "   (0 => DRV8305 flagged its OWN fault: VDS/shoot-through/UV)");
p("  rawI counts       = A:"+rd(0x0B20)+"  B:"+rd(0x0B40)+"  C:"+rd(0x0B00)+"  (zero-current ~2073)");
var nm=["A/CMPSS3","B/CMPSS1","C/CMPSS6"]; var base=[0x5CC0,0x5C80,0x5D20];
p("  CMPSS COMPSTS latches (which comparator tripped):");
for(var i=0;i<3;i++){ var st=rd(base[i]+0x2);
  p("    "+nm[i]+": COMPSTS=0x"+java.lang.Integer.toHexString(st)+
    "  Hlatch="+((st&0x2)?1:0)+"  Llatch="+((st&0x200)?1:0)); }
var epnm=["A/EPWM6","B/EPWM5","C/EPWM3"]; var epbase=[0x4500,0x4400,0x4200];
p("  EPWM DCF state:");
for(var j=0;j<3;j++){
  p("    "+epnm[j]+": CMPA="+rd(epbase[j]+0x6B)+
    " TZFLG=0x"+java.lang.Integer.toHexString(rd(epbase[j]+0x22))+
    " TZOSTFLG=0x"+java.lang.Integer.toHexString(rd(epbase[j]+0x2C))+
    " DCFCTL=0x"+java.lang.Integer.toHexString(rd(epbase[j]+0xC7))+
    " OFF="+rd(epbase[j]+0xC9)+" WIN="+rd(epbase[j]+0xCB)+
    " OFFCNT="+rd(epbase[j]+0xCA)+" WINCNT="+rd(epbase[j]+0xCC));
}
p("=> nFAULT=0 => DRV8305 real fault (dead-time/VDS). nFAULT=1 + a CMPSS latch => C28x CMPSS side.");
p("==============================================");
set("motorVars.flagRunIdentAndOnLine","0",e); set("IdqSet_A.value[1]","0",e);
s.target.runAsynch(); Thread.sleep(200); s.target.halt();
gateLow(); set("motorVars.flagEnableSys","0",e);
s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
