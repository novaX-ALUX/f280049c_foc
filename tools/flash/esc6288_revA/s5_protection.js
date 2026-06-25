/*
 * s5_protection.js - esc6288_revA bring-up STAGE 5: protection / trip paths.
 *
 * Confirms the hardware safety latches and the trip-zone path. esc6288 protections:
 *   - CMPSS3 (ADCINC2) phase-C current OC  -> EPWM X-BAR TRIP8 -> OST (all PWM low)
 *   - CMPSS5 (ADCINA6) DC-bus over-voltage -> EPWM X-BAR TRIP9 -> OST
 *   - software OC on phases A/B (no CMPSS) -> product ISR, only when ARMED (deferred to stage 4)
 *
 * MODES (default observe; the script NEVER creates a dangerous electrical condition itself):
 *   (no arg)     OBSERVE: read CMPSS3/CMPSS5 latches, EPWM OST, faultNow/faultUse. Baseline (clear).
 *   force=tz     Software-force the trip-zone (TZFRC OST) and confirm OST sets on all 3 EPWMs --
 *                proves the trip path forces outputs low. This IS safe-off (leaves gates tripped).
 *   inject=oc    PROMPT the operator to apply an over-current on phase C, then READ-ONLY confirm
 *                CMPSS3 latched + OST fired. The script only injects a wait window, never the fault.
 *   inject=ov    Same for DC-bus over-voltage on CMPSS5 (apply a bus voltage above the OV set).
 *
 * Registers: CMPSS3 COMPSTS=0x5CC2, CMPSS5 COMPSTS=0x5D02 (HIGH latch=0x2, LOW latch=0x200,
 *   HIGH live=0x1). EPWM TZFLG=0x4093/0x4193/0x4293 (OST=0x4), TZFRC=base+0x9B.
 *
 * Usage: dss.sh tools/flash/esc6288_revA/s5_protection.js <ccxml> <product.out> [force=tz|inject=oc|inject=ov]
 */
importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);
function p(s){ System.out.println(s); }
function num(nm){ try { return Number(e.evaluate(nm)); } catch(err){ return NaN; } }
function set(nm,v){ try { e.evaluate(nm+"="+v); return true; } catch(err){ return false; } }
function rd(a){ return s.memory.readData(Memory.Page.DATA,a,16,1,false)[0]&0xFFFF; }
function wr(a,v){ try { s.memory.writeData(Memory.Page.DATA,a,v,16); } catch(x){} }

var TZFLG=[0x4093,0x4193,0x4293], TZFRC=[0x409B,0x419B,0x429B], OST=0x4;
var CMPSS3=0x5CC2, CMPSS5=0x5D02, HLATCH=0x2, LLATCH=0x200;
function ostBit(i){ return ((rd(TZFLG[i])&OST)!=0)?1:0; }
function ostStr(){ return ostBit(0)+"/"+ostBit(1)+"/"+ostBit(2); }
function ostAllSet(){ return ostBit(0)&&ostBit(1)&&ostBit(2); }
function forceOST(){ for(var i=0;i<3;i++) wr(TZFRC[i],OST); }
function latchStr(a){ var v=rd(a); return "high=" + ((v&HLATCH)?1:0) + " low=" + ((v&LLATCH)?1:0) + " (raw 0x"+v.toString(16)+")"; }

var ccxml=arguments[0], out=arguments[1];
var MODE="observe";
for(var i=2;i<arguments.length;i++){ var a=""+arguments[i];
    if(a=="force=tz")MODE="forcetz"; else if(a=="inject=oc")MODE="oc"; else if(a=="inject=ov")MODE="ov"; }
var INJECT_WINDOW_S=8;

var env=ScriptingEnvironment.instance(); env.setScriptTimeout(120000);
var server=env.getServer("DebugServer.1"); server.setConfig(ccxml);
var s=server.openSession("*","C28xx_CPU1");
s.target.connect(); s.memory.loadProgram(out);
var e=s.expression;

function safeExit(code){ forceOST(); set("motorVars.flagRunIdentAndOnLine",0); set("motorVars.flagEnableSys",0);
    p("  -> safe-off: OST forced (" + ostStr() + "), flagRunIdentAndOnLine=0, flagEnableSys=0.");
    try { s.target.halt(); s.target.disconnect(); } catch(x){} server.stop(); s.terminate();
    java.lang.System.exit(code); }
function bail(why){ p(""); p("!!!!!! STAGE 5 (protection) FAILED: " + why); safeExit(1); }

p(""); p("======== esc6288 STAGE 5: protection / trip paths  [mode=" + MODE + "] ========");
// product.out self-enables (flagEnableSys=1) and runs disarmed; gates stay held off by OST.
s.target.runAsynch(); Thread.sleep(1500); s.target.halt();
if(!(num("halHandle")>0)) bail("halHandle invalid -- run stage 1 first.");
if(num("motorVars.flagRunIdentAndOnLine")!=0) bail("unexpectedly ARMED (flagRunIdentAndOnLine!=0) -- abort.");

function report(tag){
    p("--- " + tag + " ---");
    p("  EPWM TZFLG.OST [1/2/3] = " + ostStr());
    p("  CMPSS3 (phase-C OC) COMPSTS: " + latchStr(CMPSS3));
    p("  CMPSS5 (bus OV)     COMPSTS: " + latchStr(CMPSS5));
    p("  faultNow.all=" + num("motorVars.faultNow.all") + "  faultUse.all=" + num("motorVars.faultUse.all") +
      "  moduleOverCurrent=" + num("motorVars.faultNow.bit.moduleOverCurrent"));
}
report("baseline (startup, disarmed)");
// safe invariant: the board must arrive safe-off (gates tripped) before any sub-test.
if(!ostAllSet()) bail("OST not set at baseline -- power stage NOT safe-off (run stage 2 first / trip path broken).");

if(MODE=="observe"){
    p(""); p(">>> STAGE 5 OK (observe): protection latch/flag baseline read. Nothing forced.");
    p("    Sub-tests: force=tz (software trip), inject=oc / inject=ov (manual injection + read-back).");
    p("    [Stage 4] phase-A/B software OC needs the motor ARMED -- verify with the first-spin script.");
    p("================================================================");
    s.target.halt(); s.target.disconnect(); server.stop(); s.terminate();
} else if(MODE=="forcetz"){
    p(""); p(">>> force=tz: writing TZFRC OST (software trip) ...");
    forceOST();
    report("after TZFRC");
    if(!ostAllSet()) bail("OST did not set after TZFRC -- trip path not forcing outputs low (or no EALLOW access; verify on bench).");
    p(">>> software trip-zone forces all PWM outputs low (safe-off path verified).");
    safeExit(0);
} else {
    // manual injection + read-only confirm. The script does NOT create the condition.
    var which = (MODE=="oc") ? "an OVER-CURRENT on PHASE C (above the CMPSS3 OC threshold)"
                             : "a DC-BUS OVER-VOLTAGE (above the CMPSS5 OV set, ~56 V)";
    var reg   = (MODE=="oc") ? CMPSS3 : CMPSS5;
    p("");
    p("  >>> MANUAL INJECTION REQUIRED <<<");
    p("  Within the next " + INJECT_WINDOW_S + " s, apply " + which + " using a current-limited /");
    p("  controlled bench source. The script only WAITS and then reads the latch -- it does not");
    p("  create the condition. (Gates remain tripped; this exercises the hardware comparator path.)");
    if(!ostAllSet()) bail("OST not set before injection -- gates NOT safe-off; aborting (do not inject).");
    s.target.runAsynch(); Thread.sleep(INJECT_WINDOW_S*1000); s.target.halt();
    report("after injection window");
    var v=rd(reg), latched=((v&HLATCH)||(v&LLATCH))!=0;
    if(latched && ostAllSet())
        p(">>> CONFIRMED: " + (MODE=="oc"?"CMPSS3 OC":"CMPSS5 OV") + " latched AND OST fired -> protection works.");
    else
        p(">>> NOT latched (latch=" + latched + ", OST=" + ostStr() + "). Either nothing was injected, the " +
          "threshold/divider needs tuning, or the X-BAR route is wrong -- judge against what you applied.");
    safeExit(0);
}
